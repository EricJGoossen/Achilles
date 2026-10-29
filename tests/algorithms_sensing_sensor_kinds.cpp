// Every library sensor kind, read through a real Simulation against motion
// whose answer is known in closed form: a body spinning freely about the
// vertical (the encoders, the IMU parts, the truth sensors), a box falling
// onto the ground and coming to rest (the accelerometer's constrained
// acceleration), and lidars over the ground (ranges, misses, noise).
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <string>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sensing/accelerometer.hpp"
#include "algorithms/sensing/encoder.hpp"
#include "algorithms/sensing/gyroscope.hpp"
#include "algorithms/sensing/lidar.hpp"
#include "algorithms/sensing/magnetometer.hpp"
#include "algorithms/sensing/truth_sensor.hpp"
#include "engine/topology/orderings.hpp"
#include "interface/simulation.hpp"
#include "support/sensing_scene.hpp"
#include "support/temp_dir.hpp"

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::algorithms::sensing::Accelerometer;
using achilles::algorithms::sensing::Encoder;
using achilles::algorithms::sensing::Gyroscope;
using achilles::algorithms::sensing::Lidar;
using achilles::algorithms::sensing::Magnetometer;
using achilles::algorithms::sensing::TruthOf;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::Simulation;
using achilles::test_support::Archetype;
using achilles::test_support::kZUpConfig;
using achilles::test_support::SceneJoint;
using achilles::test_support::SensorFile;
using achilles::test_support::TempDir;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kDt = 0.002F;
constexpr float kSpin = 2.0F;   // rad/s about +z
constexpr float kLever = 0.5F;  // the accelerometer's offset from the axis
constexpr float kGravity = 9.81F;

void Steps(Simulation& sim, int count) {
  for (int i = 0; i < count; ++i) {
    ASSERT_TRUE(sim.Step(kDt));
  }
}

// A body spinning at kSpin about the world's vertical axis, nothing
// slowing it, and sensors on it.
class Spinner : public ::testing::Test {
 protected:
  void SetUp() override {
    SceneJoint world{.name = "world"};
    SceneJoint spinner{
        .name = "spinner",
        .parent = "world",
        .dof = 2,  // yaw, about +z
        .velocity = {kSpin, 0, 0, 0, 0, 0}
    };
    dir_.Write(
        "scene.arow",
        Archetype(
            "scene",
            {world, spinner},
            {"multi.arow",
             "single.arow",
             "gyro.arow",
             "gyro_tilted.arow",
             "accel.arow",
             "mag.arow",
             "truth.arow"}
        )
    );
    dir_.Write(
        "multi.arow",
        SensorFile("multi", "encoder", "scene", "spinner", {"axis: [0, 0, 1]"})
    );
    dir_.Write(
        "single.arow",
        SensorFile(
            "single",
            "encoder",
            "scene",
            "spinner",
            {"axis: [0, 0, 1]", "multi_turn: false"}
        )
    );
    dir_.Write(
        "gyro.arow", SensorFile("gyro", "gyroscope", "scene", "spinner")
    );
    // A quarter turn about x: the sensor's +y is the body's +z.
    dir_.Write(
        "gyro_tilted.arow",
        SensorFile(
            "gyro_tilted",
            "gyroscope",
            "scene",
            "spinner",
            {"mount: {translation: [0, 0, 0], rotation: [0.70710678, "
             "0.70710678, 0, 0]}"}
        )
    );
    dir_.Write(
        "accel.arow",
        SensorFile(
            "accel",
            "accelerometer",
            "scene",
            "spinner",
            {"mount: {translation: [" + std::to_string(kLever) +
             ", 0, 0], rotation: [1, 0, 0, 0]}"}
        )
    );
    dir_.Write(
        "mag.arow", SensorFile("mag", "magnetometer", "scene", "spinner")
    );
    dir_.Write(
        "truth.arow", SensorFile("truth", "world_transform", "scene", "spinner")
    );
    dir_.Write("config.yaml", kZUpConfig);
    ASSERT_TRUE(sim_.LoadArowFile((dir_.Path() / "scene.arow").string()));
    ASSERT_TRUE(sim_.LoadConfigFile((dir_.Path() / "config.yaml").string()));
    ASSERT_TRUE(sim_.Init(1));
  }

  TempDir dir_;
  Simulation sim_;
};

}  // namespace

TEST_F(Spinner, AMultiTurnEncoderUnwrapsPastHalfATurn) {
  Steps(sim_, 1000);  // 2 s: 4 rad, past pi
  const auto reading = sim_.Read(sim_.FindSensor<Encoder>({"multi", 0}));
  EXPECT_NEAR(reading.value.position, kSpin * reading.sample_time, 1e-3F);
  EXPECT_GT(reading.value.position, kPi);
  EXPECT_NEAR(reading.value.velocity, kSpin, 1e-4F);
}

TEST_F(Spinner, ASingleTurnEncoderWrapsIntoOneTurn) {
  Steps(sim_, 1000);
  const auto multi = sim_.Read(sim_.FindSensor<Encoder>({"multi", 0}));
  const auto single = sim_.Read(sim_.FindSensor<Encoder>({"single", 0}));
  EXPECT_LE(single.value.position, kPi);
  EXPECT_GT(single.value.position, -kPi);
  EXPECT_NEAR(
      single.value.position, multi.value.position - (2.0F * kPi), 1e-3F
  );
}

TEST_F(Spinner, AGyroscopeReadsTheSpinInItsOwnAxes) {
  Steps(sim_, 100);
  const auto straight = sim_.Read(sim_.FindSensor<Gyroscope>({"gyro", 0}));
  EXPECT_NEAR(straight.value.Z(), kSpin, 1e-4F);
  EXPECT_NEAR(straight.value.X(), 0.0F, 1e-4F);
  const auto tilted = sim_.Read(sim_.FindSensor<Gyroscope>({"gyro_tilted", 0}));
  EXPECT_NEAR(tilted.value.Y(), kSpin, 1e-4F);
  EXPECT_NEAR(tilted.value.Z(), 0.0F, 1e-4F);
}

// Off the axis, specific force is the centripetal pull toward the axis
// plus the support against gravity: (-w^2 r, 0, g) in the body's axes.
TEST_F(Spinner, AnAccelerometerOffTheAxisReadsCentripetalPlusSupport) {
  Steps(sim_, 100);
  const auto reading = sim_.Read(sim_.FindSensor<Accelerometer>({"accel", 0}));
  EXPECT_NEAR(reading.value.X(), -kSpin * kSpin * kLever, 1e-3F);
  EXPECT_NEAR(reading.value.Y(), 0.0F, 1e-3F);
  EXPECT_NEAR(reading.value.Z(), kGravity, 1e-3F);
}

TEST_F(Spinner, AMagnetometerReadsTheWorldFieldTurnedIntoTheBody) {
  Steps(sim_, 300);
  const auto reading = sim_.Read(sim_.FindSensor<Magnetometer>({"mag", 0}));
  const float angle = kSpin * reading.sample_time;
  // The default field, (0.2, 0, -0.45), seen from a body turned `angle`.
  EXPECT_NEAR(reading.value.X(), 0.2F * std::cos(angle), 1e-3F);
  EXPECT_NEAR(reading.value.Y(), -0.2F * std::sin(angle), 1e-3F);
  EXPECT_NEAR(reading.value.Z(), -0.45F, 1e-4F);
}

TEST_F(Spinner, ATruthSensorReadsTheFieldExactly) {
  Steps(sim_, 50);
  using Truth = TruthOf<ABAAlgorithm, ABAField::kWorldTransform>;
  const auto reading = sim_.Read(sim_.FindSensor<Truth>({"truth", 0}));
  const std::size_t row =
      sim_.RowFor<TopologicalOrdering>({"scene", "spinner"});
  const auto pose = sim_.ViewFor<ABAAlgorithm>()
                        .Load<ABAField::kWorldTransform, ScalarOperationT>(row);
  EXPECT_EQ(reading.value.Rotation().W(), pose.Rotation().W());
  EXPECT_EQ(reading.value.Rotation().Z(), pose.Rotation().Z());
}

TEST_F(Spinner, EverySampleAdvancesTheSequence) {
  Steps(sim_, 10);
  const auto reading = sim_.Read(sim_.FindSensor<Gyroscope>({"gyro", 0}));
  EXPECT_EQ(reading.sequence, 10U);
}

// ---------------------------------------------------------------------------
// Accelerometer under contact
// ---------------------------------------------------------------------------

namespace {

// A box dropped onto the ground: free-fall reads nothing, resting reads
// the ground's support -- the contact solver's constrained acceleration.
class DroppedBox : public ::testing::Test {
 protected:
  void SetUp() override {
    SceneJoint world{.name = "world"};
    SceneJoint ground{
        .name = "ground",
        .parent = "world",
        .translation = {0, 0, -0.1F},
        .extents = {2, 2, 0.1F},
        .group = 1,
        .mask = 2
    };
    SceneJoint box{
        .name = "box",
        .parent = "world",
        .dof = 6,
        .translation = {0, 0, 0.6F},
        .extents = {0.1F, 0.1F, 0.1F},
        .group = 2,
        .mask = 1
    };
    dir_.Write(
        "scene.arow", Archetype("scene", {world, ground, box}, {"accel.arow"})
    );
    dir_.Write(
        "accel.arow", SensorFile("accel", "accelerometer", "scene", "box")
    );
    dir_.Write("config.yaml", kZUpConfig);
    ASSERT_TRUE(sim_.LoadArowFile((dir_.Path() / "scene.arow").string()));
    ASSERT_TRUE(sim_.LoadConfigFile((dir_.Path() / "config.yaml").string()));
    ASSERT_TRUE(sim_.Init(1));
    sim_.SetIntegrator(Simulation::Integrator::kSemiImplicitEuler);
  }

  TempDir dir_;
  Simulation sim_;
};

}  // namespace

TEST_F(DroppedBox, AnAccelerometerReadsNothingInFreeFallAndGravityAtRest) {
  Steps(sim_, 50);  // 0.1 s: still falling
  const auto handle = sim_.FindSensor<Accelerometer>({"accel", 0});
  EXPECT_NEAR(sim_.Read(handle).value.Norm(), 0.0F, 1e-3F);

  Steps(sim_, 1500);  // 3 s: landed and settled
  const auto resting = sim_.Read(handle).value;
  EXPECT_NEAR(resting.Z(), kGravity, 0.05F);
  EXPECT_NEAR(resting.X(), 0.0F, 0.05F);
  EXPECT_NEAR(resting.Y(), 0.0F, 0.05F);
}

// ---------------------------------------------------------------------------
// Lidar
// ---------------------------------------------------------------------------

namespace {

// A single-beam lidar 1 m over the ground looking down, one looking up at
// nothing, and a noisy one looking down.
class LidarOverGround : public ::testing::Test {
 protected:
  void Build(Simulation& sim, std::uint64_t seed) {
    SceneJoint world{.name = "world"};
    SceneJoint ground{
        .name = "ground",
        .parent = "world",
        .translation = {0, 0, -0.1F},
        .extents = {2, 2, 0.1F}
    };
    SceneJoint post{
        .name = "post", .parent = "world", .translation = {0, 0, 1.0F}
    };
    dir_.Write(
        "scene.arow",
        Archetype(
            "scene",
            {world, ground, post},
            {"down.arow", "up.arow", "noisy.arow", "coarse.arow"}
        )
    );
    // Sensor +x onto the body's -z (down) or +z (up): a quarter turn about
    // y either way.
    const std::string down =
        "mount: {translation: [0, 0, 0], rotation: [0.70710678, 0, "
        "0.70710678, 0]}";
    const std::string up =
        "mount: {translation: [0, 0, 0], rotation: [0.70710678, 0, "
        "-0.70710678, 0]}";
    const std::string one_beam = "horizontal_samples: 1";
    const std::string narrow = "horizontal_fov: 0.01";
    dir_.Write(
        "down.arow",
        SensorFile("down", "lidar", "scene", "post", {down, one_beam, narrow})
    );
    dir_.Write(
        "up.arow",
        SensorFile("up", "lidar", "scene", "post", {up, one_beam, narrow})
    );
    dir_.Write(
        "noisy.arow",
        SensorFile(
            "noisy",
            "lidar",
            "scene",
            "post",
            {down, one_beam, narrow, "range_noise: 0.05"}
        )
    );
    dir_.Write(
        "coarse.arow",
        SensorFile(
            "coarse",
            "lidar",
            "scene",
            "post",
            {down, one_beam, narrow, "range_bias: 0.06", "resolution: 0.25"}
        )
    );
    dir_.Write("config.yaml", kZUpConfig);
    ASSERT_TRUE(sim.LoadArowFile((dir_.Path() / "scene.arow").string()));
    ASSERT_TRUE(sim.LoadConfigFile((dir_.Path() / "config.yaml").string()));
    ASSERT_TRUE(sim.Init(seed));
    Steps(sim, 2);
  }

  float Range(const Simulation& sim, const std::string& name) {
    const auto scan = sim.Read(sim.FindSensor<Lidar>({name, 0}));
    EXPECT_EQ(scan.value.beams, 1U);
    return scan.value.ranges[0];
  }

  TempDir dir_;
};

}  // namespace

TEST_F(LidarOverGround, ABeamReadsTheDistanceToWhatItHits) {
  Simulation sim;
  Build(sim, 1);
  EXPECT_NEAR(Range(sim, "down"), 1.0F, 1e-3F);
}

TEST_F(LidarOverGround, ABeamThatHitsNothingReadsInfinity) {
  Simulation sim;
  Build(sim, 1);
  EXPECT_TRUE(std::isinf(Range(sim, "up")));
}

TEST_F(LidarOverGround, BiasAndResolutionApplyToTheRange) {
  Simulation sim;
  Build(sim, 1);
  // 1 + 0.06, to the nearest 0.25.
  EXPECT_FLOAT_EQ(Range(sim, "coarse"), 1.0F);
}

TEST_F(LidarOverGround, NoiseIsTheSameForTheSameSeedAndDiffersForAnother) {
  Simulation first;
  Build(first, 7);
  Simulation again;
  Build(again, 7);
  Simulation other;
  Build(other, 8);
  const float noisy = Range(first, "noisy");
  EXPECT_NE(noisy, 1.0F);
  EXPECT_NEAR(noisy, 1.0F, 0.3F);
  EXPECT_EQ(noisy, Range(again, "noisy"));
  EXPECT_NE(noisy, Range(other, "noisy"));
}
