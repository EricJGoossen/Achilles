// What each display draws and asks for, read straight from its Frame -- no
// screen, no window: the frame is the whole contract.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <type_traits>

#include "achilles/build_config.hpp"
#include "display/contract/frame.hpp"
#include "display/contract/input.hpp"
#include "display/views/camera/camera_view.hpp"
#include "display/views/camera/depth_view.hpp"
#include "display/views/effort_control.hpp"
#include "display/views/lidar_view.hpp"
#include "display/views/reading_view.hpp"
#include "interface/simulation.hpp"
#include "support/display_handles.hpp"

using achilles::display::CameraAxesView;
using achilles::display::CameraView;
using achilles::display::DepthView;
using achilles::display::EffortChannel;
using achilles::display::EncoderChannel;
using achilles::display::Frame;
using achilles::display::HasChannels;
using achilles::display::Input;
using achilles::display::LidarView;
using achilles::display::PoseChannel;
using achilles::display::ReadingView;
using achilles::display::TorqueControl;
using achilles::domain::sensing::Camera;
using achilles::domain::sensing::Encoder;
using achilles::domain::sensing::Lidar;
using achilles::interface::Simulation;
using achilles::test_support::HandlesNamed;
using Torque = achilles::interface::truth::Torque;

// A reading view takes its own reading's channels: an encoder's, a motor's
// -- and there's none for a camera's image or a lidar's scan.
static_assert(HasChannels<Encoder> && HasChannels<Torque>);
static_assert(!HasChannels<Camera> && !HasChannels<Lidar>);
static_assert(std::is_same_v<ReadingView<Encoder>::Channel, EncoderChannel>);
static_assert(std::is_same_v<ReadingView<Torque>::Channel, EffortChannel>);

namespace {

constexpr float kDt = 0.002F;

// `scene`/`scene`.arow and its sim_config.yaml, from examples/.
void Load(Simulation& sim, const std::string& scene) {
  const std::string dir =
      std::string(ACHILLES_SOURCE_DIR) + "/examples/" + scene + "/";
  ASSERT_TRUE(sim.LoadArowFile(dir + scene + ".arow"));
  ASSERT_TRUE(sim.LoadConfigFile(dir + "sim_config.yaml"));
  ASSERT_TRUE(sim.Init(1));
}

}  // namespace

TEST(ClipToImage, KeepsWhatsInsideAndDropsWhatsNot) {
  const auto inside = CameraAxesView::ClipToImage({0.2F, 0.2F}, {0.8F, 0.6F});
  ASSERT_TRUE(inside.has_value());
  const std::array<float, 4> kept = inside.value_or(std::array<float, 4>{});
  EXPECT_FLOAT_EQ(kept[0], 0.2F);
  EXPECT_FLOAT_EQ(kept[3], 0.6F);

  // From the center out past the right edge: cut at x = 1.
  const auto crossing = CameraAxesView::ClipToImage({0.5F, 0.5F}, {1.5F, 0.5F});
  ASSERT_TRUE(crossing.has_value());
  const std::array<float, 4> cut = crossing.value_or(std::array<float, 4>{});
  EXPECT_FLOAT_EQ(cut[2], 1.0F);
  EXPECT_FLOAT_EQ(cut[3], 0.5F);

  EXPECT_FALSE(
      CameraAxesView::ClipToImage({1.2F, 0.1F}, {1.5F, 0.9F}).has_value()
  );
  EXPECT_FALSE(
      CameraAxesView::ClipToImage({-0.5F, 2.0F}, {2.0F, 3.0F}).has_value()
  );
}

TEST(EffortControl, OneSpringingSliderAndItsEffort) {
  Simulation sim;
  Load(sim, "maze");
  const TorqueControl control{.max_effort = 7.0F};
  const auto handles = HandlesNamed<TorqueControl>(sim, "left_front_motor");

  Frame frame;
  control.Draw(handles, &frame);
  ASSERT_EQ(frame.sliders.size(), 1U);
  EXPECT_FLOAT_EQ(frame.sliders[0].min, -7.0F);
  EXPECT_FLOAT_EQ(frame.sliders[0].max, 7.0F);
  EXPECT_TRUE(frame.sliders[0].spring);
  EXPECT_EQ(frame.aspect, 0.0F);  // controls only: nothing to draw

  EXPECT_EQ(control.Efforts(Input{{3.0F}}), (std::array<float, 1>{3.0F}));
  // A slider the window hasn't drawn yet asks for nothing.
  EXPECT_EQ(control.Efforts(Input{}), (std::array<float, 1>{0.0F}));
}

TEST(LidarView, APointPerBeamThatHitInsideTheRings) {
  Simulation sim;
  Load(sim, "maze");
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(sim.Step(kDt));
  }
  const auto handles = HandlesNamed<LidarView>(sim, "lidar");
  const auto scan = std::get<0>(handles).Read().value;
  std::size_t hits = 0;
  for (std::size_t i = 0; i < scan.beams; ++i) {
    hits += std::isfinite(scan.ranges[i]) ? 1 : 0;
  }
  ASSERT_GT(hits, 0U);

  Frame frame;
  LidarView{}.Draw(handles, &frame);
  EXPECT_EQ(frame.aspect, 1.0F);
  ASSERT_EQ(frame.points.size(), 1U);
  EXPECT_EQ(frame.points[0].xy.size(), 2 * hits);
  // Every hit is inside the plot, which spans the max range.
  for (float coordinate : frame.points[0].xy) {
    EXPECT_GE(coordinate, 0.0F);
    EXPECT_LE(coordinate, 1.0F);
  }
}

// Only the channels asked for, in the order the reading lists them: a
// robot's chassis on the ground -- where it is across it, and its heading.
TEST(ReadingView, OnlyTheChannelsAskedFor) {
  Simulation sim;
  Load(sim, "waypoint_robot");
  ASSERT_TRUE(sim.Step(kDt));
  using View = ReadingView<achilles::interface::truth::WorldTransform>;
  const auto handles = HandlesNamed<View>(sim, "chassis_pose");

  Frame frame;
  View{
      .channels = {PoseChannel::kX, PoseChannel::kZ, PoseChannel::kTurnY}
  }.Draw(handles, &frame);
  ASSERT_EQ(frame.traces.size(), 3U);
  EXPECT_EQ(frame.traces[0].label, "x (m)");
  EXPECT_EQ(frame.traces[1].label, "z (m)");
  EXPECT_EQ(frame.traces[2].label, "turn y (rad)");
  for (const auto& trace : frame.traces) {
    EXPECT_TRUE(trace.plot.empty()) << trace.label << ": a plot of its own";
  }
}

// Several attachments of one kind: each channel one plot, named for it, a
// line per attachment named by `names` -- here what four wheel motors
// applied, shown without being pushed on.
TEST(ReadingView, SeveralAttachmentsShareAPlotPerChannel) {
  Simulation sim;
  Load(sim, "waypoint_robot");
  ASSERT_TRUE(sim.Step(kDt));
  using View = ReadingView<achilles::interface::truth::Torque, 4>;
  const auto handles = HandlesNamed<View>(
      sim,
      "left_front_motor",
      "left_rear_motor",
      "right_front_motor",
      "right_rear_motor"
  );

  Frame frame;
  View{
      .channels = {EffortChannel::kEffort}, .names = {"lf", "lr", "rf", "rr"}
  }.Draw(handles, &frame);
  ASSERT_EQ(frame.traces.size(), 4U);
  const std::array<std::string, 4> names = {"lf", "lr", "rf", "rr"};
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(frame.traces[i].label, names[i]);
    EXPECT_EQ(frame.traces[i].plot, "effort");
  }
  EXPECT_EQ(frame.traces[0].value, std::get<0>(handles).Read().value.effort);
}

// A camera's two views of one image: its color, with the world's axes over
// it for CameraAxesView, and its depth -- each filling its own drawing area.
TEST(CameraView, ColorAndDepthEachInTheirOwnView) {
  if constexpr (!achilles::kWithGpu) {
    GTEST_SKIP() << "cameras render only on the GPU";
  }
  Simulation sim;
  Load(sim, "pan_tilt");
  const auto handles = HandlesNamed<CameraView>(sim, "rig_camera");
  // Pipelined: the first image arrives one sample after the first.
  for (int i = 0; i < 40; ++i) {
    ASSERT_TRUE(sim.Step(kDt));
  }
  Frame color;
  CameraAxesView{}.Draw(handles, &color);
  ASSERT_EQ(color.images.size(), 1U);
  EXPECT_FLOAT_EQ(color.aspect, 640.0F / 480.0F);
  EXPECT_EQ(color.images[0].rgb.size(), 3U * 640U * 480U);
  EXPECT_TRUE(color.images[0].values.empty());
  EXPECT_EQ(color.images[0].where.width, 1.0F);
  // Looking at the world's origin from 5 m out, the axes are in view.
  EXPECT_FALSE(color.lines.empty());

  Frame plain;
  CameraView{}.Draw(handles, &plain);
  ASSERT_EQ(plain.images.size(), 1U);
  EXPECT_TRUE(plain.lines.empty());

  Frame depth;
  DepthView{}.Draw(handles, &depth);
  ASSERT_EQ(depth.images.size(), 1U);
  EXPECT_FLOAT_EQ(depth.aspect, 640.0F / 480.0F);
  EXPECT_TRUE(depth.images[0].rgb.empty());
  EXPECT_EQ(depth.images[0].values.size(), 640U * 480U);
  EXPECT_EQ(depth.images[0].where.width, 1.0F);
}

// Before a camera's first image, either view says so and draws nothing.
TEST(CameraView, NothingUntilTheFirstImage) {
  if constexpr (!achilles::kWithGpu) {
    GTEST_SKIP() << "cameras render only on the GPU";
  }
  Simulation sim;
  Load(sim, "pan_tilt");
  const auto handles = HandlesNamed<DepthView>(sim, "rig_camera");
  Frame frame;
  DepthView{}.Draw(handles, &frame);
  EXPECT_TRUE(frame.images.empty());
  EXPECT_EQ(frame.aspect, 0.0F);
  ASSERT_EQ(frame.text.size(), 1U);
}

// An encoder's two channels, each its latest value at its sample time, and no
// drawing area: just its plots.
TEST(ReadingView, AnEncodersPositionAndVelocityAtTheirSampleTime) {
  Simulation sim;
  Load(sim, "maze");
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(sim.Step(kDt));
  }
  using View = ReadingView<Encoder>;
  const auto handles = HandlesNamed<View>(sim, "left_front_encoder");
  const auto reading = std::get<0>(handles).Read();
  ASSERT_GT(reading.sequence, 0U);

  Frame frame;
  View{}.Draw(handles, &frame);
  EXPECT_EQ(frame.aspect, 0.0F);
  ASSERT_EQ(frame.traces.size(), 2U);
  EXPECT_EQ(frame.traces[0].label, "position");
  EXPECT_EQ(frame.traces[0].value, reading.value.position);
  EXPECT_EQ(frame.traces[1].label, "velocity");
  EXPECT_EQ(frame.traces[1].value, reading.value.velocity);
  for (const auto& trace : frame.traces) {
    EXPECT_EQ(trace.time, reading.sample_time);
  }
}

// A pose's position and rotation vector: the maze's robot, in its start cell
// (maze.arow), facing the world's +y.
TEST(ReadingView, APoseWhereItIsAndHowItsTurned) {
  Simulation sim;
  Load(sim, "maze");
  ASSERT_TRUE(sim.Step(kDt));
  using View = ReadingView<achilles::interface::truth::WorldTransform>;
  const auto handles = HandlesNamed<View>(sim, "chassis_pose");

  Frame frame;
  View{}.Draw(handles, &frame);
  ASSERT_EQ(frame.traces.size(), 6U);
  EXPECT_NEAR(frame.traces[0].value, -3.75F, 0.01F);  // x (m)
  EXPECT_NEAR(frame.traces[1].value, -3.75F, 0.01F);  // y (m)
  const auto turn = std::get<0>(handles).Read().value.Rotation().Log();
  EXPECT_EQ(frame.traces[3].value, turn.X());
  EXPECT_EQ(frame.traces[4].value, turn.Y());
  EXPECT_EQ(frame.traces[5].value, turn.Z());
}
