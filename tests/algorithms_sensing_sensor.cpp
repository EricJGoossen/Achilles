#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "algorithms/sensing/sensor.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sensing/effects.hpp"
#include "domain/spatial/transform.hpp"

using achilles::algorithms::sensing::SensorConfig;
using achilles::domain::math::Vector3;
using achilles::domain::sensing::Vec3;
using achilles::domain::spatial::Transform;

namespace {

using Entries = std::vector<std::pair<std::string, std::vector<double>>>;

}  // namespace

TEST(SensorConfig, ReadsEachSettingOrItsDefault) {
  const Entries entries = {
      {"gain", {2.5}},
      {"count", {7}},
      {"on", {1}},
      {"axis", {0, 1, 0}},
  };
  SensorConfig config("s", entries);
  EXPECT_FLOAT_EQ(config.Scalar("gain", 0.0F), 2.5F);
  EXPECT_FLOAT_EQ(config.Scalar("missing", 4.0F), 4.0F);
  EXPECT_EQ(config.Count("count", 0), 7U);
  EXPECT_TRUE(config.Flag("on", false));
  const Vector3<float> axis = config.Vector("axis", {});
  EXPECT_FLOAT_EQ(axis.Y(), 1.0F);
  EXPECT_NO_THROW(config.Finish());
}

TEST(SensorConfig, PeriodIsTheOneOptionEverySensorHas) {
  const Entries entries = {{"period", {0.1}}};
  SensorConfig config("s", entries);
  EXPECT_FLOAT_EQ(config.Options().period, 0.1F);
  SensorConfig none("s", {});
  EXPECT_FLOAT_EQ(none.Options().period, 0.0F);
}

TEST(SensorConfig, PoseIsTranslationThenNormalizedRotation) {
  const Entries entries = {{"mount", {1, 2, 3, 2, 0, 0, 0}}};
  SensorConfig config("s", entries);
  const Transform<float> pose = config.Pose("mount", {});
  EXPECT_FLOAT_EQ(pose.Translation().Z(), 3.0F);
  EXPECT_FLOAT_EQ(pose.Rotation().W(), 1.0F);
}

TEST(SensorConfig, ErrorModelReadsItsPrefixedSettings) {
  const Entries entries = {
      {"velocity_bias", {0.5}},
      {"velocity_resolution", {0.1}},
      {"velocity_delay_samples", {3}},
  };
  SensorConfig config("s", entries);
  const auto params = config.ErrorModel<float>("velocity_");
  EXPECT_FLOAT_EQ(params.bias.bias, 0.5F);
  EXPECT_FLOAT_EQ(params.resolution.resolution, 0.1F);
  EXPECT_EQ(params.latency.samples, 3U);
  EXPECT_FLOAT_EQ(params.noise.density, 0.0F);
  EXPECT_NO_THROW(config.Finish());
}

TEST(SensorConfig, AThreeAxisModelsBiasTakesThreeNumbers) {
  const Entries entries = {{"bias", {1, 2, 3}}};
  SensorConfig config("s", entries);
  const auto params = config.ErrorModel<Vec3>();
  EXPECT_FLOAT_EQ(params.bias.bias.Z(), 3.0F);
}

TEST(SensorConfigErrors, FinishRejectsASettingNothingRead) {
  const Entries entries = {{"perod", {0.1}}};
  SensorConfig config("s", entries);
  (void)config.Options();
  EXPECT_THROW(config.Finish(), std::invalid_argument);
}

TEST(SensorConfigErrors, TheWrongNumberOfValuesThrows) {
  const Entries entries = {{"axis", {1, 0}}};
  SensorConfig config("s", entries);
  EXPECT_THROW((void)config.Vector("axis", {}), std::invalid_argument);
}

TEST(SensorConfigErrors, ANegativeCountThrows) {
  const Entries entries = {{"count", {-1}}};
  SensorConfig config("s", entries);
  EXPECT_THROW((void)config.Count("count", 0), std::invalid_argument);
}

TEST(SensorConfigErrors, AZeroRotationThrows) {
  const Entries entries = {{"mount", {0, 0, 0, 0, 0, 0, 0}}};
  SensorConfig config("s", entries);
  EXPECT_THROW((void)config.Pose("mount", {}), std::invalid_argument);
}

TEST(SensorConfigErrors, ADelayPastTheCapacityThrows) {
  const Entries entries = {{"delay_samples", {1000}}};
  SensorConfig config("s", entries);
  EXPECT_THROW((void)config.ErrorModel<float>(), std::invalid_argument);
}
