#include <gtest/gtest.h>

#include <filesystem>

#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "interface/sim_config_loader.hpp"
#include "support/temp_dir.hpp"
#include "util/simd_ops.hpp"

using achilles::algorithms::Acceleration;
using achilles::algorithms::BatchOperationT;
using achilles::algorithms::Quaternion;
using achilles::algorithms::SimConfig;
using achilles::algorithms::Transform;
using achilles::algorithms::Velocity;
using achilles::algorithms::Vector3;
using achilles::interface::LoadSimConfig;
using achilles::test_support::TempDir;
using achilles::util::AllTrue;

namespace {

// SimConfig's own fields are batched (BatchOperationT lanes, all equal
// since nothing here ever loads per-lane-distinct data) -- IsApprox
// returns a lane mask, not a bool, so every comparison below reduces it
// with AllTrue the same way algorithms_aba_aba_step.cpp's BatchTrue does.
::testing::AssertionResult ConfigApprox(
    const SimConfig& actual, const SimConfig& expected
) {
  if (!AllTrue(actual.world_base_transform.IsApprox(expected.world_base_transform)
      )) {
    return ::testing::AssertionFailure() << "world_base_transform mismatch";
  }
  if (!AllTrue(actual.base_velocity.IsApprox(expected.base_velocity))) {
    return ::testing::AssertionFailure() << "base_velocity mismatch";
  }
  if (!AllTrue(actual.base_acceleration.IsApprox(expected.base_acceleration))) {
    return ::testing::AssertionFailure() << "base_acceleration mismatch";
  }
  return ::testing::AssertionSuccess();
}

}  // namespace

// No file at all is the most common "no proper config" case (a fresh
// project with nothing authored yet) -- must come back as
// SimConfig::Default(), not throw.
TEST(SimConfigLoader, MissingFileFallsBackToDefault) {
  SimConfig config = LoadSimConfig("/nonexistent/path/does_not_exist.yaml");

  EXPECT_TRUE(ConfigApprox(config, SimConfig::Default()));
}

// Garbage YAML (not even parseable) must fall back the same way a missing
// file does, not throw or propagate a yaml-cpp parse error.
TEST(SimConfigLoader, UnparsableYamlFallsBackToDefault) {
  TempDir dir;
  std::filesystem::path file = dir.Write("bad.yaml", "not: [valid: yaml: at: all");

  SimConfig config = LoadSimConfig(file);

  EXPECT_TRUE(ConfigApprox(config, SimConfig::Default()));
}

// A bare scalar document (valid YAML, but not the expected map shape) must
// also fall back rather than crash on the first `root["..."]` lookup.
TEST(SimConfigLoader, NonMapDocumentFallsBackToDefault) {
  TempDir dir;
  std::filesystem::path file = dir.Write("scalar.yaml", "42");

  SimConfig config = LoadSimConfig(file);

  EXPECT_TRUE(ConfigApprox(config, SimConfig::Default()));
}

// Every field given -- proves LoadSimConfig actually reads real values
// through, not just that it falls back safely.
TEST(SimConfigLoader, ParsesEveryFieldWhenFullySpecified) {
  TempDir dir;
  std::filesystem::path file = dir.Write("full.yaml", R"(
world_base_transform:
  translation: [1, 2, 3]
  rotation: [0, 1, 0, 0]
base_velocity:
  angular: [0.1, 0.2, 0.3]
  linear: [1, 0, 0]
base_acceleration:
  angular: [0, 0, 0]
  linear: [0, 0, -9.8]
)");

  SimConfig config = LoadSimConfig(file);

  SimConfig expected;
  expected.world_base_transform = Transform(
      Vector3(BatchOperationT(1.0F), BatchOperationT(2.0F), BatchOperationT(3.0F)),
      Quaternion(
          BatchOperationT(0.0F),
          BatchOperationT(1.0F),
          BatchOperationT(0.0F),
          BatchOperationT(0.0F)
      )
  );
  expected.base_velocity = Velocity(
      Vector3(BatchOperationT(0.1F), BatchOperationT(0.2F), BatchOperationT(0.3F)),
      Vector3(BatchOperationT(1.0F), BatchOperationT(0.0F), BatchOperationT(0.0F))
  );
  expected.base_acceleration = Acceleration(
      Vector3::Zero(),
      Vector3(BatchOperationT(0.0F), BatchOperationT(0.0F), BatchOperationT(-9.8F))
  );

  EXPECT_TRUE(ConfigApprox(config, expected));
}

// A field that's present but malformed (wrong element count) falls back to
// that field's own default, while an unrelated, well-formed field in the
// same document still parses -- proves fallback is per-field, not
// all-or-nothing across the whole document.
TEST(SimConfigLoader, MalformedFieldFallsBackWhileOthersStillParse) {
  TempDir dir;
  std::filesystem::path file = dir.Write("partial.yaml", R"(
world_base_transform:
  translation: [1, 2]
base_velocity:
  linear: [5, 0, 0]
)");

  SimConfig config = LoadSimConfig(file);

  SimConfig expected;
  expected.base_velocity = Velocity(
      Vector3::Zero(),
      Vector3(BatchOperationT(5.0F), BatchOperationT(0.0F), BatchOperationT(0.0F))
  );

  EXPECT_TRUE(ConfigApprox(config, expected));
}
