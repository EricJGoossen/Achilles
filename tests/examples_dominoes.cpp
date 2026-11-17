// Runs examples/dominoes/dominoes.arow with its own sim_config.yaml -- the
// scene scripts/run-dominoes.sh launches -- and checks the chain does what
// the scene says: the first domino, set spinning, knocks over the second,
// and so on down the row, each falling after the one before, until every one
// of the ten is down.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <numbers>
#include <string>
#include <vector>

#include "achilles/build_config.hpp"
#include "domain/attachment/sim_quantities.hpp"
#include "domain/math/vector3.hpp"
#include "interface/simulation.hpp"
#include "interface/state_handle.hpp"

using achilles::interface::Simulation;
using achilles::interface::StateHandle;
namespace input = achilles::interface::input;

namespace {

constexpr std::size_t kDominoes = 10;
constexpr float kDt = 0.004F;
constexpr float kSeconds = 8.0F;

const std::filesystem::path kExampleDir =
    std::filesystem::path(ACHILLES_SOURCE_DIR) / "examples" / "dominoes";

// How far a domino has tipped from upright, in degrees: the angle between
// its own +y (up its height) and the world's.
float Tilt(const StateHandle<const input::WorldTransform>& pose) {
  const auto up = pose.Read().Rotation().Rotate(
      achilles::domain::math::Vector3<float>(0.0F, 1.0F, 0.0F)
  );
  return std::acos(std::clamp(up.Y(), -1.0F, 1.0F)) * 180.0F /
         std::numbers::pi_v<float>;
}

class Dominoes : public ::testing::Test {
 protected:
  void SetUp() override {
    // dominoes.arow carries a camera, which renders only on the GPU.
    if constexpr (!achilles::kWithGpu) {
      GTEST_SKIP() << "the scene's camera renders only on the GPU";
    }
    ASSERT_TRUE(sim_.LoadArowFile((kExampleDir / "dominoes.arow").string()));
    ASSERT_TRUE(sim_.LoadConfigFile((kExampleDir / "sim_config.yaml").string())
    );
    ASSERT_TRUE(sim_.Init(1));
    // The integrator with collision, as achilles runs it by default.
    sim_.SetIntegrator(Simulation::Integrator::kSemiImplicitEuler);
    for (std::size_t i = 0; i < kDominoes; ++i) {
      poses_.push_back(sim_.State<const input::WorldTransform>(
          {"scene", "domino_" + std::to_string(i + 1)}
      ));
    }
  }

  Simulation sim_;
  std::vector<StateHandle<const input::WorldTransform>> poses_;
};

}  // namespace

TEST_F(Dominoes, EachKnocksOverTheNextUntilEveryOneIsDown) {
  // When each domino first tipped past 30 degrees: falling, not just nudged.
  constexpr float kFalling = 30.0F;
  std::array<float, kDominoes> fell_at{};
  fell_at.fill(-1.0F);
  const auto steps = static_cast<int>(kSeconds / kDt);
  for (int step = 1; step <= steps; ++step) {
    ASSERT_TRUE(sim_.Step(kDt));
    for (std::size_t i = 0; i < kDominoes; ++i) {
      if (fell_at[i] < 0.0F && Tilt(poses_[i]) > kFalling) {
        fell_at[i] = static_cast<float>(step) * kDt;
      }
    }
  }

  for (std::size_t i = 0; i < kDominoes; ++i) {
    ASSERT_GE(fell_at[i], 0.0F) << "domino " << i + 1 << " never fell";
    if (i > 0) {
      EXPECT_GT(fell_at[i], fell_at[i - 1])
          << "domino " << i + 1 << " fell before the one that knocks it over";
    }
    // Down: lying on the next one, or on the ground for the last.
    EXPECT_GT(Tilt(poses_[i]), 60.0F) << "domino " << i + 1;
  }
}
