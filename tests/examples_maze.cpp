// The maze demo's scene (examples/maze): the robot loads with all six of
// its sensors, and its lidar, at the start, sees the start cell's walls
// where maze.arow's drawing puts them.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <numbers>
#include <string>

#include "achilles/build_config.hpp"
#include "domain/sensing/camera.hpp"
#include "domain/sensing/encoder.hpp"
#include "domain/sensing/gyroscope.hpp"
#include "domain/sensing/lidar.hpp"
#include "interface/simulation.hpp"

using achilles::domain::sensing::Encoder;
using achilles::domain::sensing::Gyroscope;
using achilles::domain::sensing::Lidar;
using achilles::interface::Simulation;

namespace {

constexpr float kDt = 0.004F;
constexpr float kPi = std::numbers::pi_v<float>;
constexpr std::size_t kBeams = 360;  // sensors/lidar.arow

std::filesystem::path ExampleDir() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "examples" / "maze";
}

class Maze : public ::testing::Test {
 protected:
  void SetUp() override {
    // maze.arow carries an overhead camera, which renders only on the GPU.
    if constexpr (!achilles::kWithGpu) {
      GTEST_SKIP() << "cameras render only on the GPU";
    }
    ASSERT_TRUE(sim_.LoadArowFile((ExampleDir() / "maze.arow").string()));
    ASSERT_TRUE(sim_.LoadConfigFile((ExampleDir() / "sim_config.yaml").string())
    );
    ASSERT_TRUE(sim_.Init(1));
    sim_.SetIntegrator(Simulation::Integrator::kSemiImplicitEuler);
    ASSERT_TRUE(sim_.Step(kDt));
  }

  // The range of the beam nearest `azimuth` (radians, counterclockwise
  // from the robot's forward).
  float RangeAt(float azimuth) const {
    const auto scan = sim_.Sensor<Lidar>("lidar").Read().value;
    const float step = 2.0F * kPi / kBeams;
    const auto beam =
        static_cast<std::size_t>(std::lround(((azimuth + kPi) / step) - 0.5F)) %
        kBeams;
    return scan.ranges[beam];
  }

  Simulation sim_;
};

}  // namespace

TEST_F(Maze, LoadsTheRobotsSensors) {
  EXPECT_NO_THROW((void)sim_.Sensor<Lidar>("lidar"));
  EXPECT_NO_THROW((void)sim_.Sensor<Gyroscope>("gyroscope"));
  for (const char* wheel :
       {"left_front", "left_rear", "right_front", "right_rear"}) {
    EXPECT_NO_THROW((void)sim_.Sensor<Encoder>(std::string(wheel) + "_encoder")
    );
  }
}

// The start cell, centered on (-3.75, -3.75) and facing +y, is walled in
// on three sides 0.75 m from its center -- less a wall's 0.05 m half
// thickness -- and open to the east for two cells.
TEST_F(Maze, TheLidarSeesTheStartCellsWalls) {
  constexpr float kTolerance = 0.03F;  // range noise, resolution, settling
  EXPECT_NEAR(RangeAt(0.0F), 0.70F, kTolerance);          // ahead, +y
  EXPECT_NEAR(RangeAt(kPi / 2), 0.70F, kTolerance);       // left, -x
  EXPECT_NEAR(RangeAt(-kPi + 0.01F), 0.70F, kTolerance);  // behind, -y
  EXPECT_NEAR(RangeAt(-kPi / 2), 2.20F, kTolerance);      // right, +x
}

// The overhead camera looks straight down from 10 m over the maze's center,
// world +x to the image's right and +y up it: open floor reads 10 m deep, and
// the robot, in the bottom-left start cell, reads nearer -- where a mirrored
// or upside-down image would put bare floor.
TEST_F(Maze, TheOverheadCameraSeesTheRobotFromAbove) {
  using achilles::domain::sensing::Camera;
  // Its first image arrives a sample late, pipelined: two samples at 30 a
  // second.
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(sim_.Step(kDt));
  }
  const auto camera = sim_.Sensor<Camera>("overhead_camera");
  const auto frame = camera.Read().value;
  ASSERT_EQ(frame.width, 640U);
  ASSERT_EQ(frame.height, 480U);
  const auto& depth = camera.State().images[frame.image].depth;

  // Where world (x, y) on the floor lands, from overhead_camera.arow's
  // height and field of view, with square pixels.
  constexpr float kHeight = 10.0F;
  const float half_width = kHeight * std::tan(0.6F);
  const float half_height = half_width * 480.0F / 640.0F;
  const auto depth_at = [&](float x, float y) {
    const auto col = static_cast<std::size_t>(
        (0.5F + (x / (2.0F * half_width))) * static_cast<float>(frame.width)
    );
    const auto row = static_cast<std::size_t>(
        (0.5F - (y / (2.0F * half_height))) * static_cast<float>(frame.height)
    );
    return depth[(row * frame.width) + col];
  };

  // Cell centers are always floor: the walls run along the cells' edges.
  EXPECT_NEAR(depth_at(0.75F, 0.75F), kHeight, 0.02F);
  EXPECT_NEAR(depth_at(3.75F, -3.75F), kHeight, 0.02F);  // mirrored left-right
  EXPECT_NEAR(depth_at(-3.75F, 3.75F), kHeight, 0.02F);  // mirrored top-bottom
  EXPECT_LT(depth_at(-3.75F, -3.75F), kHeight - 0.05F);  // the robot
}
