// Validates examples/balancers/ -- the scene scripts/run-balancers.sh
// drives -- end to end: actuators on the wheels of several copies of one
// archetype, the wheels' contacts with the ground, and the free-floating
// bodies together. Each piece has its own tests; this is the one place that
// checks they compose into robots that fall when left alone, can be held
// up by the PD loop balancers.cpp runs, and answer only to their own
// actuators.
//
// Reads the real files, so an edit to the scene that breaks it fails here
// -- but with balancer.arow's 8 copies cut to 2: every test needs only two
// robots, and the collision passes' cost grows with the pairs of bodies,
// which in a debug build turns 8 copies into minutes.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numbers>
#include <string>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/actuator.hpp"
#include "interface/simulation.hpp"
#include "support/temp_dir.hpp"

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::AngularActuator;
using achilles::interface::JointAddress;
using achilles::interface::Simulation;
using achilles::test_support::TempDir;

namespace {

constexpr ScalarOperationT kDt = 0.004F;  // balancers.cpp's
constexpr std::size_t kRobotCount = 2;    // copies, cut down from 8
constexpr float kWheelRadius = 0.1F;
// Well on its way down: past this, an unactuated robot doesn't come back.
constexpr float kFalling = 30.0F * std::numbers::pi_v<float> / 180.0F;

std::filesystem::path ExampleDir() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "examples" / "balancers";
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// The row each joint lives at in ABA's Views, which TopologicalOrdering
// orders.
std::size_t Row(const Simulation& sim, const JointAddress& joint) {
  return sim.RowFor<TopologicalOrdering>(joint);
}

// One copy's rows and wheel actuators.
struct Robot {
  std::size_t body;
  std::size_t left_row;
  std::size_t right_row;
  AngularActuator left;
  AngularActuator right;
};

Robot AttachRobot(const Simulation& sim, std::size_t instance) {
  auto attach = [&](const char* wheel) {
    return sim.AttachActuator<AngularActuator>(
        {"balancer", wheel, instance}, {1, 0, 0}
    );
  };
  return {
      Row(sim, {"balancer", "body", instance}),
      Row(sim, {"balancer", "left_wheel", instance}),
      Row(sim, {"balancer", "right_wheel", instance}),
      attach("left_wheel"),
      attach("right_wheel")
  };
}

// Same convention as balancers.cpp's Balancer::Pitch: positive leans the
// body's top toward +z.
float Pitch(const Simulation& sim, const Robot& robot) {
  auto world =
      sim.ViewFor<ABAAlgorithm>()
          .Load<ABAField::kWorldTransform, ScalarOperationT>(robot.body);
  const auto& q = world.Rotation();
  float up_y = 1.0F - (2.0F * ((q.X() * q.X()) + (q.Z() * q.Z())));
  float up_z = 2.0F * ((q.Y() * q.Z()) + (q.W() * q.X()));
  return std::atan2(up_z, up_y);
}

float BodyZ(const Simulation& sim, const Robot& robot) {
  return sim.ViewFor<ABAAlgorithm>()
      .Load<ABAField::kWorldTransform, ScalarOperationT>(robot.body)
      .Translation()
      .Z();
}

float AngularX(const Simulation& sim, std::size_t row) {
  return sim.ViewFor<ABAAlgorithm>()
      .Load<ABAField::kJointVelocity, ScalarOperationT>(row)
      .Angular()
      .X();
}

// balancers.cpp's control law, with one of its well-tuned gain pairs.
void Balance(const Simulation& sim, const Robot& robot) {
  constexpr float kKp = 10.0F;
  constexpr float kKd = 1.2F;
  constexpr float kSpeedGain = 1.5F;
  constexpr float kPositionGain = 0.5F;
  constexpr float kLimit = 2.5F;
  float pitch_rate = AngularX(sim, robot.body);
  float wheel_spin =
      0.5F * (AngularX(sim, robot.left_row) + AngularX(sim, robot.right_row));
  float speed = (wheel_spin + pitch_rate) * kWheelRadius;
  float torque = (kKp * Pitch(sim, robot)) + (kKd * pitch_rate) +
                 (kSpeedGain * speed) + (kPositionGain * BodyZ(sim, robot));
  torque = std::clamp(torque, -kLimit, kLimit);
  robot.left.Apply(torque);
  robot.right.Apply(torque);
}

class BalancersScene : public ::testing::Test {
 protected:
  void SetUp() override {
    // The real scene, copied whole, with only balancer.arow's copies cut.
    std::string balancer = ReadFile(ExampleDir() / "balancer.arow");
    const std::string copies = "copies: 8";
    std::size_t at = balancer.find(copies);
    ASSERT_NE(at, std::string::npos) << "balancer.arow's copies changed";
    balancer.replace(at, copies.size(), "copies: 2");
    dir_.Write("balancer.arow", balancer);
    dir_.Write("box.arow", ReadFile(ExampleDir() / "box.arow"));
    std::filesystem::path arow =
        dir_.Write("balancers.arow", ReadFile(ExampleDir() / "balancers.arow"));

    ASSERT_TRUE(sim_.LoadArowFile(arow.string()));
    ASSERT_TRUE(sim_.LoadConfigFile((ExampleDir() / "sim_config.yaml").string())
    );
    ASSERT_TRUE(sim_.Init());
    // The integrator balancers.cpp runs with.
    sim_.SetIntegrator(Simulation::Integrator::kSemiImplicitEuler);
    for (std::size_t i = 0; i < kRobotCount; ++i) {
      robots_.push_back(AttachRobot(sim_, i));
    }
  }

  // Steps for `seconds`, calling `control` before every Step. Returns false
  // if any Step fails.
  template <typename ControlT>
  bool Run(float seconds, ControlT control) {
    for (float t = 0.0F; t < seconds; t += kDt) {
      control();
      if (!sim_.Step(kDt)) {
        return false;
      }
    }
    return true;
  }

  TempDir dir_;
  Simulation sim_;
  std::vector<Robot> robots_;
};

}  // namespace

// Positive torque on both wheels must roll the robot toward +z and, by
// the wheels' reaction on the body, pitch it back -- the signs
// balancers.cpp's control law relies on. Compared against an untouched
// copy in the same sim, which falls forward from its 5 degree start.
TEST_F(BalancersScene, PositiveWheelTorqueRollsForwardAndPitchesBack) {
  const Robot& driven = robots_[0];
  const Robot& idle = robots_[1];
  ASSERT_TRUE(Run(0.2F, [&] {
    driven.left.Apply(1.0F);
    driven.right.Apply(1.0F);
  }));

  EXPECT_LT(Pitch(sim_, driven), Pitch(sim_, idle) - 0.05F);
  EXPECT_GT(Pitch(sim_, idle), 0.1F);
  EXPECT_GT(BodyZ(sim_, driven), BodyZ(sim_, idle) + 0.02F);
}

// Only the copy whose wheels get a controller stays up; the other,
// identical but unactuated, falls. Checks both that the balancing comes
// from the actuators and that each actuator drives its own copy, not
// another lane of the same SIMD batch.
TEST_F(BalancersScene, OnlyTheControlledCopyStaysUp) {
  constexpr std::size_t kControlled = 1;
  ASSERT_TRUE(Run(0.6F, [&] { Balance(sim_, robots_[kControlled]); }));

  for (std::size_t i = 0; i < kRobotCount; ++i) {
    float pitch = std::abs(Pitch(sim_, robots_[i]));
    if (i == kControlled) {
      EXPECT_LT(pitch, 0.05F) << "robot " << i;
    } else {
      EXPECT_GT(pitch, kFalling) << "robot " << i;
    }
  }
}
