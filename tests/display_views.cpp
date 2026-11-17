// What each display draws, read straight from its Frame -- no screen, no
// window: the frame is the whole contract.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "achilles/build_config.hpp"
#include "display/contract/frame.hpp"
#include "display/views/camera/camera_view.hpp"
#include "display/views/camera/depth_view.hpp"
#include "display/views/lidar_view.hpp"
#include "display/views/reading_view.hpp"
#include "domain/actuation/dc_motor.hpp"
#include "domain/actuation/truth_actuator.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sensing/camera.hpp"
#include "domain/sensing/encoder.hpp"
#include "domain/sensing/gyroscope.hpp"
#include "domain/sensing/lidar.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/transform.hpp"
#include "interface/attachment_handle.hpp"
#include "interface/simulation.hpp"

using achilles::display::CameraAxesView;
using achilles::display::CameraView;
using achilles::display::DepthView;
using achilles::display::EffortChannel;
using achilles::display::EncoderChannel;
using achilles::display::Frame;
using achilles::display::HasChannels;
using achilles::display::LidarView;
using achilles::display::MotionChannel;
using achilles::display::MotorChannel;
using achilles::display::PoseChannel;
using achilles::display::ReadingChannels;
using achilles::display::ReadingView;
using achilles::display::VectorChannel;
using achilles::domain::sensing::Camera;
using achilles::domain::sensing::Encoder;
using achilles::domain::sensing::Gyroscope;
using achilles::domain::sensing::Lidar;
using achilles::interface::Simulation;
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

TEST(LidarView, APointPerBeamThatHitInsideTheRings) {
  Simulation sim;
  Load(sim, "maze");
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(sim.Step(kDt));
  }
  const auto lidar = sim.Sensor<Lidar>("lidar");
  const auto scan = lidar.Read().value;
  std::size_t hits = 0;
  for (std::size_t i = 0; i < scan.beams; ++i) {
    hits += std::isfinite(scan.ranges[i]) ? 1 : 0;
  }
  ASSERT_GT(hits, 0U);

  Frame frame;
  LidarView(lidar).Draw(&frame);
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
  const View view(
      sim.Sensor<achilles::interface::truth::WorldTransform>("chassis_pose"),
      {.channels = {PoseChannel::kX, PoseChannel::kZ, PoseChannel::kTurnY}}
  );

  Frame frame;
  view.Draw(&frame);
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
  using View = ReadingView<Torque, 4>;
  const View view(
      {sim.Actuator<Torque>("left_front_motor"),
       sim.Actuator<Torque>("left_rear_motor"),
       sim.Actuator<Torque>("right_front_motor"),
       sim.Actuator<Torque>("right_rear_motor")},
      {.channels = {EffortChannel::kEffort}, .names = {"lf", "lr", "rf", "rr"}}
  );

  Frame frame;
  view.Draw(&frame);
  ASSERT_EQ(frame.traces.size(), 4U);
  const std::array<std::string, 4> names = {"lf", "lr", "rf", "rr"};
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(frame.traces[i].label, names[i]);
    EXPECT_EQ(frame.traces[i].plot, "effort");
  }
  EXPECT_EQ(
      frame.traces[0].value,
      sim.Actuator<Torque>("left_front_motor").Read().value.effort
  );
}

// A camera's two views of one image: its color, with the world's axes over
// it for CameraAxesView, and its depth -- each filling its own drawing area.
TEST(CameraView, ColorAndDepthEachInTheirOwnView) {
  if constexpr (!achilles::kWithGpu) {
    GTEST_SKIP() << "cameras render only on the GPU";
  }
  Simulation sim;
  Load(sim, "pan_tilt");
  const auto camera = sim.Sensor<Camera>("rig_camera");
  // Pipelined: the first image arrives one sample after the first.
  for (int i = 0; i < 40; ++i) {
    ASSERT_TRUE(sim.Step(kDt));
  }
  Frame color;
  CameraAxesView(camera).Draw(&color);
  ASSERT_EQ(color.images.size(), 1U);
  EXPECT_FLOAT_EQ(color.aspect, 640.0F / 480.0F);
  EXPECT_EQ(color.images[0].rgb.size(), 3U * 640U * 480U);
  EXPECT_TRUE(color.images[0].values.empty());
  EXPECT_EQ(color.images[0].where.width, 1.0F);
  // Looking at the world's origin from 5 m out, the axes are in view.
  EXPECT_FALSE(color.lines.empty());

  Frame plain;
  CameraView(camera).Draw(&plain);
  ASSERT_EQ(plain.images.size(), 1U);
  EXPECT_TRUE(plain.lines.empty());

  Frame depth;
  DepthView(camera).Draw(&depth);
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
  Frame frame;
  DepthView(sim.Sensor<Camera>("rig_camera")).Draw(&frame);
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
  const auto encoder = sim.Sensor<Encoder>("left_front_encoder");
  const auto reading = encoder.Read();
  ASSERT_GT(reading.sequence, 0U);

  Frame frame;
  ReadingView<Encoder>(encoder).Draw(&frame);
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
  using Pose = achilles::interface::truth::WorldTransform;
  const auto pose = sim.Sensor<Pose>("chassis_pose");

  Frame frame;
  ReadingView<Pose>(pose).Draw(&frame);
  ASSERT_EQ(frame.traces.size(), 6U);
  EXPECT_NEAR(frame.traces[0].value, -3.75F, 0.01F);  // x (m)
  EXPECT_NEAR(frame.traces[1].value, -3.75F, 0.01F);  // y (m)
  const auto turn = pose.Read().value.Rotation().Log();
  EXPECT_EQ(frame.traces[3].value, turn.X());
  EXPECT_EQ(frame.traces[4].value, turn.Y());
  EXPECT_EQ(frame.traces[5].value, turn.Z());
}

namespace {

// What a reading's channels emit, in order: (channel, label, value).
template <typename Value>
struct Emitted {
  using Channel = typename ReadingChannels<Value>::Channel;
  std::vector<Channel> channels;
  std::vector<std::string> labels;
  std::vector<float> values;
};
template <typename Value>
Emitted<Value> EmitOf(const Value& value) {
  Emitted<Value> out;
  ReadingChannels<Value>::Emit(
      value,
      [&](typename Emitted<Value>::Channel channel,
          std::string_view label,
          float emitted) {
        out.channels.push_back(channel);
        out.labels.emplace_back(label);
        out.values.push_back(emitted);
      }
  );
  return out;
}

// Every channel of a type is emitted once, in kAll's order.
template <typename Value>
void ExpectEmitsEveryChannelInOrder(const Emitted<Value>& emitted) {
  const auto& all = ReadingChannels<Value>::kAll;
  ASSERT_EQ(emitted.channels.size(), all.size());
  for (std::size_t i = 0; i < all.size(); ++i) {
    EXPECT_EQ(emitted.channels[i], all[i]) << "channel " << i;
  }
}

}  // namespace

// Each reading type's channels, by value: every one, in kAll's order, with
// its label and the value it reads.
TEST(ReadingChannels, AVectorsThreeAxes) {
  const auto emitted = EmitOf(achilles::domain::math::Vector3<float>(1, 2, 3));
  ExpectEmitsEveryChannelInOrder(emitted);
  EXPECT_EQ(emitted.labels, (std::vector<std::string>{"x", "y", "z"}));
  EXPECT_EQ(emitted.values, (std::vector<float>{1, 2, 3}));
}

TEST(ReadingChannels, AnEncodersPositionThenVelocity) {
  const auto emitted = EmitOf(
      achilles::domain::sensing::EncoderReading{.position = 4, .velocity = -5}
  );
  ExpectEmitsEveryChannelInOrder(emitted);
  EXPECT_EQ(emitted.labels, (std::vector<std::string>{"position", "velocity"}));
  EXPECT_EQ(emitted.values, (std::vector<float>{4, -5}));
}

TEST(ReadingChannels, APosesPlaceThenItsTurnAboutEachAxis) {
  // At (1, 2, 3), turned 0.5 rad about y.
  const achilles::domain::spatial::Transform<float> pose(
      1, 2, 3, std::cos(0.25F), 0, std::sin(0.25F), 0
  );
  const auto emitted = EmitOf(pose);
  ExpectEmitsEveryChannelInOrder(emitted);
  EXPECT_EQ(emitted.labels[0], "x (m)");
  EXPECT_EQ(emitted.labels[5], "turn z (rad)");
  EXPECT_FLOAT_EQ(emitted.values[0], 1.0F);
  EXPECT_FLOAT_EQ(emitted.values[2], 3.0F);
  EXPECT_NEAR(emitted.values[3], 0.0F, 1e-6F);
  EXPECT_NEAR(emitted.values[4], 0.5F, 1e-5F);
  EXPECT_NEAR(emitted.values[5], 0.0F, 1e-6F);
}

TEST(ReadingChannels, AMotionsAngularPartThenItsLinear) {
  // Dual's own order: roll, pitch, yaw, then x, y, z.
  const achilles::domain::spatial::SpatialVelocity<float> velocity(
      1, 2, 3, 4, 5, 6
  );
  const auto emitted = EmitOf(velocity);
  ExpectEmitsEveryChannelInOrder(emitted);
  EXPECT_EQ(emitted.labels[0], "angular x");
  EXPECT_EQ(emitted.labels[3], "linear x");
  EXPECT_EQ(emitted.values, (std::vector<float>{1, 2, 3, 4, 5, 6}));
  static_assert(std::is_same_v<
                ReadingChannels<
                    achilles::domain::spatial::SpatialVelocity<float>>::Channel,
                MotionChannel>);
  static_assert(std::is_same_v<
                ReadingChannels<achilles::domain::spatial::SpatialAcceleration<
                    float>>::Channel,
                MotionChannel>);
}

TEST(ReadingChannels, AnActuatorsEffortAndAMotorsCurrent) {
  const auto effort =
      EmitOf(achilles::domain::actuation::EffortOutput{.effort = 1.5F});
  ExpectEmitsEveryChannelInOrder(effort);
  EXPECT_EQ(effort.labels, (std::vector<std::string>{"effort"}));
  EXPECT_EQ(effort.values, (std::vector<float>{1.5F}));

  const auto motor = EmitOf(
      achilles::domain::actuation::DcMotorOutput{.effort = 2, .current = 3}
  );
  ExpectEmitsEveryChannelInOrder(motor);
  EXPECT_EQ(motor.channels.front(), MotorChannel::kEffort);
  EXPECT_EQ(
      motor.labels, (std::vector<std::string>{"effort (N*m)", "current (A)"})
  );
  EXPECT_EQ(motor.values, (std::vector<float>{2, 3}));
}

// A view plots every channel unless told which, and none if told none.
TEST(ReadingView, EveryChannelByDefaultAndNoneWhenAskedForNone) {
  Simulation sim;
  Load(sim, "maze");
  ASSERT_TRUE(sim.Step(kDt));
  const auto gyroscope = sim.Sensor<Gyroscope>("gyroscope");

  Frame all;
  ReadingView<Gyroscope>(gyroscope).Draw(&all);
  ASSERT_EQ(all.traces.size(), 3U);
  EXPECT_EQ(all.traces[0].label, "x");
  EXPECT_EQ(all.traces[2].label, "z");

  Frame yaw;
  ReadingView<Gyroscope>(
      gyroscope, {.channels = {VectorChannel::kY}}
  ).Draw(&yaw);
  ASSERT_EQ(yaw.traces.size(), 1U);
  EXPECT_EQ(yaw.traces[0].label, "y");
  EXPECT_FLOAT_EQ(yaw.traces[0].value, gyroscope.Read().value.Y());

  Frame none;
  ReadingView<Gyroscope>(gyroscope, {.channels = {}}).Draw(&none);
  EXPECT_TRUE(none.traces.empty());
  // Still says what it's showing.
  EXPECT_FALSE(none.text.empty());
}

// Order in `channels` doesn't change what's plotted: always the reading's.
TEST(ReadingView, PlotsInTheReadingsOrderWhateverOrderItsAskedIn) {
  Simulation sim;
  Load(sim, "maze");
  ASSERT_TRUE(sim.Step(kDt));
  Frame frame;
  ReadingView<Encoder>(
      sim.Sensor<Encoder>("left_front_encoder"),
      {.channels = {EncoderChannel::kVelocity, EncoderChannel::kPosition}}
  ).Draw(&frame);
  ASSERT_EQ(frame.traces.size(), 2U);
  EXPECT_EQ(frame.traces[0].label, "position");
  EXPECT_EQ(frame.traces[1].label, "velocity");
}

// Before its first sample a view says so; unnamed attachments are numbered.
TEST(ReadingView, NoSampleYetAndNumberedWhenUnnamed) {
  Simulation sim;
  Load(sim, "maze");
  using View = ReadingView<Encoder, 2>;
  const std::array<View::Handle, 2> encoders = {
      sim.Sensor<Encoder>("left_front_encoder"),
      sim.Sensor<Encoder>("left_rear_encoder")
  };

  Frame frame;
  View(encoders, {.channels = {EncoderChannel::kPosition}}).Draw(&frame);
  ASSERT_FALSE(frame.text.empty());
  EXPECT_NE(frame.text[0].find("no sample yet"), std::string::npos)
      << frame.text[0];
  ASSERT_EQ(frame.traces.size(), 2U);
  EXPECT_EQ(frame.traces[0].label, "#1");
  EXPECT_EQ(frame.traces[1].label, "#2");
  EXPECT_EQ(frame.traces[0].plot, "position");

  // Named in part: the rest keep their number.
  Frame named;
  View(
      encoders,
      {.channels = {EncoderChannel::kPosition}, .names = {"front", ""}}
  ).Draw(&named);
  ASSERT_EQ(named.traces.size(), 2U);
  EXPECT_EQ(named.traces[0].label, "front");
  EXPECT_EQ(named.traces[1].label, "#2");
}
