// The display on a real screen: the screen itself, a window drawing whatever
// Frame it's handed, and the renderer tying windows to the sim. Each test
// opens real windows for a few frames, so each skips where no screen can be
// started (no display server, as on a headless CI runner).
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "achilles/build_config.hpp"
#include "display/contract/frame.hpp"
#include "display/contract/input.hpp"
#include "display/renderer.hpp"
#include "display/views/camera/camera_view.hpp"
#include "display/views/lidar_view.hpp"
#include "display/views/reading_view.hpp"
#include "display/window/screen.hpp"
#include "display/window/window.hpp"
#include "display/window_set.hpp"
#include "domain/sensing/camera.hpp"
#include "domain/sensing/gyroscope.hpp"
#include "domain/sensing/lidar.hpp"
#include "interface/simulation.hpp"

using achilles::display::CameraView;
using achilles::display::Frame;
using achilles::display::Input;
using achilles::display::LidarView;
using achilles::display::ReadingView;
using achilles::display::Renderer;
using achilles::display::Screen;
using achilles::display::Window;
using achilles::display::WindowId;
using achilles::domain::sensing::Camera;
using achilles::domain::sensing::Gyroscope;
using achilles::domain::sensing::Lidar;
using achilles::interface::Simulation;

namespace {

constexpr float kDt = 0.002F;

// A renderer, or nothing (the test skips) where no screen can be started.
std::optional<Renderer> OpenRenderer(const char* title) {
  try {
    return std::optional<Renderer>(std::in_place, title);
  } catch (const std::runtime_error&) {
    return std::nullopt;
  }
}

void LoadMaze(Simulation& sim) {
  const std::string dir = std::string(ACHILLES_SOURCE_DIR) + "/examples/maze/";
  ASSERT_TRUE(sim.LoadArowFile(dir + "maze.arow"));
  ASSERT_TRUE(sim.LoadConfigFile(dir + "sim_config.yaml"));
  ASSERT_TRUE(sim.Init(1));
}

// A frame with something of everything a window draws.
struct Everything {
  std::vector<std::uint8_t> rgb =
      std::vector<std::uint8_t>(std::size_t{3} * 4 * 2, 200);
  std::vector<float> values = std::vector<float>(std::size_t{4} * 2, 0.5F);
  Frame frame;

  explicit Everything(double time) {
    frame.aspect = 2.0F;
    frame.text.emplace_back("everything");
    frame.images.push_back(
        {.rgb = rgb, .width = 4, .height = 2, .where = {0, 0, 0.5F, 1}}
    );
    frame.images.push_back(
        {.values = values,
         .width = 4,
         .height = 2,
         .scale = 1.0F,
         .where = {0.5F, 0, 0.5F, 1}}
    );
    frame.points.push_back({.color = {1, 0, 0}, .xy = {0.5F, 0.5F}});
    frame.lines.push_back({.color = {0, 1, 0}, .xy = {0, 0, 1, 1}});
    // One trace on a plot of its own, two sharing one.
    frame.traces.push_back({.label = "alone", .value = 1.0F, .time = time});
    frame.traces.push_back(
        {.label = "a", .value = 2.0F, .time = time, .plot = "shared"}
    );
    frame.traces.push_back(
        {.label = "b", .value = -1.0F, .time = time, .plot = "shared"}
    );
  }
};

}  // namespace

// Only one screen at a time: GLFW and the GUI are process-wide.
TEST(Screen, OnlyOneAtATime) {
  std::optional<Renderer> first = OpenRenderer("first");
  if (!first.has_value()) {
    GTEST_SKIP() << "no screen can be started here";
  }
  EXPECT_THROW(Screen("second"), std::runtime_error);
  first.reset();
  // Once it's gone, another can start.
  EXPECT_TRUE(OpenRenderer("again").has_value());
}

// A window draws everything a frame can hold, frame after frame -- then a
// frame with less in it -- and hands back what it took in: nothing pressed
// by anyone, and the time since its last frame.
TEST(Window, DrawsEveryPartOfAFrameAndHandsBackItsInput) {
  std::optional<Screen> screen;
  try {
    screen.emplace("window test");
  } catch (const std::runtime_error&) {
    GTEST_SKIP() << "no screen can be started here";
  }
  {
    Window window("Everything", 400.0F, 300.0F);
    EXPECT_EQ(window.Title(), "Everything");
    EXPECT_TRUE(window.IsOpen());
    for (int frame = 0; frame < 5; ++frame) {
      const Everything everything(0.1 * frame);
      screen->BeginFrame();
      const Input input = window.Present(everything.frame);
      screen->EndFrame();
      EXPECT_GE(input.seconds, 0.0F);
      EXPECT_FALSE(input.keys_pressed[0]);
      EXPECT_TRUE(window.IsOpen());
    }
    // Then a frame with less in it: no images, no traces.
    Frame less;
    less.text.emplace_back("less");
    screen->BeginFrame();
    const Input input = window.Present(less);
    screen->EndFrame();
    EXPECT_GE(input.seconds, 0.0F);
    EXPECT_TRUE(window.IsOpen());
    // Its textures go in the screen's context.
    screen->MakeCurrent();
  }
}

// Two windows of one title are still two windows.
TEST(Window, TwoOfOneTitleAreTwo) {
  std::optional<Screen> screen;
  try {
    screen.emplace("window test");
  } catch (const std::runtime_error&) {
    GTEST_SKIP() << "no screen can be started here";
  }
  {
    Window first("Same", 200.0F, 100.0F);
    Window second("Same", 200.0F, 100.0F);
    // Each draws its own traces: two windows, each with its own history.
    for (int frame = 0; frame < 3; ++frame) {
      Frame one;
      one.traces.push_back(
          {.label = "one", .value = 1.0F, .time = static_cast<double>(frame)}
      );
      Frame two;
      two.traces.push_back(
          {.label = "two", .value = 2.0F, .time = static_cast<double>(frame)}
      );
      screen->BeginFrame();
      (void)first.Present(one);
      (void)second.Present(two);
      screen->EndFrame();
    }
    EXPECT_TRUE(first.IsOpen());
    EXPECT_TRUE(second.IsOpen());
    screen->MakeCurrent();
  }
}

// A renderer with no window up has nothing to draw: the run is over.
TEST(Renderer, NothingUpMeansDone) {
  std::optional<Renderer> renderer = OpenRenderer("empty");
  if (!renderer.has_value()) {
    GTEST_SKIP() << "no screen can be started here";
  }
  EXPECT_FALSE(renderer->Draw());
}

// Windows attached to a running sim draw frame after frame as it steps, can
// be swapped and detached mid-run, and once the last is gone, the run is
// over.
TEST(Renderer, WindowsComeAndGoWhileTheSimRuns) {
  Simulation sim;
  LoadMaze(sim);
  std::optional<Renderer> renderer = OpenRenderer("maze");
  if (!renderer.has_value()) {
    GTEST_SKIP() << "no screen can be started here";
  }
  const WindowId scanner =
      renderer->Attach(LidarView(sim.Sensor<Lidar>("lidar")), "Lidar");
  const WindowId gyro = renderer->Attach(
      ReadingView<Gyroscope>(sim.Sensor<Gyroscope>("gyroscope")), "Gyroscope"
  );
  std::optional<WindowId> camera;
  if constexpr (achilles::kWithGpu) {
    camera = renderer->Attach(
        CameraView(sim.Sensor<Camera>("overhead_camera")), "Overhead camera"
    );
  }

  for (int frame = 0; frame < 10; ++frame) {
    for (int tick = 0; tick < 8; ++tick) {
      renderer->ApplyWrites();
      ASSERT_TRUE(sim.Step(kDt));
    }
    ASSERT_TRUE(renderer->Draw()) << "frame " << frame;
  }
  EXPECT_NE(renderer->Find<LidarView>(scanner), nullptr);
  EXPECT_EQ(renderer->Find<LidarView>(gyro), nullptr);

  // Swapped mid-run: the gyro's window now shows the lidar too.
  renderer->Swap(gyro, LidarView(sim.Sensor<Lidar>("lidar")));
  EXPECT_NE(renderer->Find<LidarView>(gyro), nullptr);
  ASSERT_TRUE(sim.Step(kDt));
  EXPECT_TRUE(renderer->Draw());

  // Detached mid-run: the rest keep drawing.
  renderer->Detach(scanner);
  EXPECT_EQ(renderer->Find<LidarView>(scanner), nullptr);
  ASSERT_TRUE(sim.Step(kDt));
  EXPECT_TRUE(renderer->Draw());

  renderer->Detach(gyro);
  if (camera.has_value()) {
    EXPECT_TRUE(renderer->Draw());
    renderer->Detach(*camera);
  }
  EXPECT_FALSE(renderer->Draw());
}
