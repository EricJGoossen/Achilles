// Validates examples/ball_balance/ -- the scene scripts/run-ball-balance.sh
// drives -- end to end: actuators on two pivots of each of several copies
// of one archetype, a free ball rolling on a moving plate, and contact
// between them. Each piece has its own tests; this is the one place that
// checks they compose into plates that tip the way ball_balance.cpp
// assumes, that can catch and center a dropped ball, and that answer only
// to their own actuators.
//
// Reads the real files, so an edit to the scene that breaks it fails here
// -- but with plate_rig.arow's and ball.arow's one copy each raised to 2,
// so one plate can be checked against another.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "domain/archetype.hpp"
#include "engine/topology/orderings.hpp"
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

constexpr ScalarOperationT kDt = 0.004F;  // ball_balance.cpp's
constexpr std::size_t kPlateCount = 2;    // copies, raised from 1
constexpr float kPlateSpacing = 0.6F;
constexpr float kPlateTop = 0.52F;  // the pivot 0.5 up, plus the plate
constexpr float kBallRadius = 0.08F;

std::filesystem::path ExampleDir() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "examples" / "ball_balance";
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// `file`'s text with its "copies: 1" raised to "copies: 2". Fails the
// calling test if the scene no longer says 1.
std::string RaiseCopies(const std::string& file) {
  std::string text = ReadFile(ExampleDir() / file);
  const std::string copies = "copies: 1";
  std::size_t at = text.find(copies);
  EXPECT_NE(at, std::string::npos) << file << "'s copies changed";
  if (at != std::string::npos) {
    text.replace(at, copies.size(), "copies: 2");
  }
  return text;
}

// The row each joint lives at in ABA's Views, which TopologicalOrdering
// orders.
std::size_t Row(const Simulation& sim, const JointAddress& joint) {
  return sim.RowFor<TopologicalOrdering>(joint);
}

// Same as ball_balance.cpp's Place: moves a root joint to (x, y, z),
// keeping its rotation, and stops it.
void Place(const Simulation& sim, std::size_t row, float x, float y, float z) {
  auto view = sim.ViewFor<ABAAlgorithm>();
  auto pose = view.Load<ABAField::kJointPosition, ScalarOperationT>(row);
  const auto& q = pose.Rotation();
  view.Store<ABAField::kJointPosition, ScalarOperationT>(
      row, decltype(pose)(x, y, z, q.W(), q.X(), q.Y(), q.Z())
  );
  view.Store<ABAField::kJointVelocity, ScalarOperationT>(row, {});
}

// One copy's rows and pivot motors.
struct Rig {
  std::size_t post;
  std::size_t gimbal;
  std::size_t plate;
  std::size_t ball;
  AngularActuator tilt_x;
  AngularActuator tilt_z;
};

Rig AttachRig(const Simulation& sim, std::size_t instance) {
  return {
      Row(sim, {"plate_rig", "post", instance}),
      Row(sim, {"plate_rig", "gimbal", instance}),
      Row(sim, {"plate_rig", "plate", instance}),
      Row(sim, {"ball", "ball", instance}),
      sim.AttachActuator<AngularActuator>(
          {"plate_rig", "gimbal", instance}, {1, 0, 0}
      ),
      sim.AttachActuator<AngularActuator>(
          {"plate_rig", "plate", instance}, {0, 0, 1}
      ),
  };
}

// The ball's center relative to its plate's pivot, in the scene's axes -- the
// same reading ball_balance.cpp's Plate::BallOnPlate makes.
std::array<float, 3> BallOnPlate(const Simulation& sim, const Rig& rig) {
  auto view = sim.ViewFor<ABAAlgorithm>();
  auto ball = view.Load<ABAField::kWorldTransform, ScalarOperationT>(rig.ball);
  auto pivot = view.Load<ABAField::kWorldTransform, ScalarOperationT>(rig.plate)
                   .Translation();
  const auto& q = ball.Rotation();
  float up_x = 2.0F * ((q.X() * q.Y()) - (q.W() * q.Z()));
  float up_y = 1.0F - (2.0F * ((q.X() * q.X()) + (q.Z() * q.Z())));
  float up_z = 2.0F * ((q.Y() * q.Z()) + (q.W() * q.X()));
  // Then from world axes back to the scene's own (ball_balance.arow's
  // world joint turned its +y to the world's +z): world (x, y, z) is
  // scene (x, z, -y).
  float x = ball.Translation().X() + (kBallRadius * up_x) - pivot.X();
  float y = ball.Translation().Y() + (kBallRadius * up_y) - pivot.Y();
  float z = ball.Translation().Z() + (kBallRadius * up_z) - pivot.Z();
  return {x, z, -y};
}

// ball_balance.cpp's inner loop: drives one pivot to `target` radians.
// `use_z` picks the angle about Z (the plate's pivot) rather than X (the
// gimbal's); either way the rate is the joint's one generalized velocity.
void HoldPivot(
    const Simulation& sim,
    const AngularActuator& motor,
    std::size_t row,
    float target,
    bool use_z
) {
  auto view = sim.ViewFor<ABAAlgorithm>();
  const auto& q =
      view.Load<ABAField::kJointPosition, ScalarOperationT>(row).Rotation();
  float angle = 2.0F * std::atan2(use_z ? q.Z() : q.X(), q.W());
  float rate =
      view.Load<ABAField::kJointVelocity, ScalarOperationT>(row).Angular().X();
  float torque = (25.0F * (target - angle)) - (1.5F * rate);
  motor.Apply(std::clamp(torque, -8.0F, 8.0F));
}

class BallBalanceScene : public ::testing::Test {
 protected:
  void SetUp() override {
    // The real scene, copied whole, with only the copies raised.
    dir_.Write("plate_rig.arow", RaiseCopies("plate_rig.arow"));
    dir_.Write("ball.arow", RaiseCopies("ball.arow"));
    std::filesystem::path arow = dir_.Write(
        "ball_balance.arow", ReadFile(ExampleDir() / "ball_balance.arow")
    );

    ASSERT_TRUE(sim_.LoadArowFile(arow.string()));
    ASSERT_TRUE(sim_.LoadConfigFile((ExampleDir() / "sim_config.yaml").string())
    );
    ASSERT_TRUE(sim_.Init());
    // The integrator ball_balance.cpp runs with.
    sim_.SetIntegrator(Simulation::Integrator::kSemiImplicitEuler);
    for (std::size_t i = 0; i < kPlateCount; ++i) {
      rigs_.push_back(AttachRig(sim_, i));
      Place(sim_, rigs_.back().post, PlateX(i), 0.0F, 0.0F);
    }
  }

  static float PlateX(std::size_t index) {
    return static_cast<float>(index) * kPlateSpacing;
  }

  // Sets ball `index` down at (x, z) from its plate's center, `height`
  // above the plate's top face.
  void SetBall(std::size_t index, float x, float z, float height) {
    Place(sim_, rigs_[index].ball, PlateX(index) + x, kPlateTop + height, z);
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
  std::vector<Rig> rigs_;
};

}  // namespace

// Holding plate 0 tilted positively about X must roll its ball toward +z,
// and plate 1 tilted positively about Z must roll its ball toward -x --
// the signs ball_balance.cpp's outer loop relies on.
TEST_F(BallBalanceScene, PositiveTiltsRollTheBallTheWayTheControllerAssumes) {
  SetBall(0, 0.0F, 0.0F, 0.0F);
  SetBall(1, 0.0F, 0.0F, 0.0F);
  constexpr float kTilt = 0.1F;
  ASSERT_TRUE(Run(0.35F, [&] {
    HoldPivot(sim_, rigs_[0].tilt_x, rigs_[0].gimbal, kTilt, false);
    HoldPivot(sim_, rigs_[0].tilt_z, rigs_[0].plate, 0.0F, true);
    HoldPivot(sim_, rigs_[1].tilt_x, rigs_[1].gimbal, 0.0F, false);
    HoldPivot(sim_, rigs_[1].tilt_z, rigs_[1].plate, kTilt, true);
  }));

  auto ball0 = BallOnPlate(sim_, rigs_[0]);
  auto ball1 = BallOnPlate(sim_, rigs_[1]);
  EXPECT_GT(ball0[2], 0.01F);
  EXPECT_NEAR(ball0[0], 0.0F, 0.005F);
  EXPECT_LT(ball1[0], -0.01F);
  EXPECT_NEAR(ball1[2], 0.0F, 0.005F);
}

// A ball dropped off center onto plate 1, which runs ball_balance.cpp's two
// loops, ends up near the center within a second; the same drop onto plate 0,
// whose motors never act, ends with the ball off the plate. Checks both that
// the catch comes from the actuators and that each actuator drives its own
// copy.
TEST_F(BallBalanceScene, OnlyTheControlledPlateCatchesItsBall) {
  constexpr float kBallGain = 3.6F;
  constexpr float kBallRateGain = 1.1F;
  constexpr float kMaxTilt = 0.25F;
  SetBall(0, 0.1F, 0.1F, 0.1F);
  SetBall(1, 0.1F, 0.1F, 0.1F);
  std::array<float, 2> last = {0.1F, 0.1F};
  ASSERT_TRUE(Run(1.0F, [&] {
    // ball_balance.cpp's outer loop toward the center, with velocity by a
    // plain finite difference.
    auto ball = BallOnPlate(sim_, rigs_[1]);
    float velocity_x = (ball[0] - last[0]) / kDt;
    float velocity_z = (ball[2] - last[1]) / kDt;
    last = {ball[0], ball[2]};
    float tilt_x = std::clamp(
        -(kBallGain * ball[2]) - (kBallRateGain * velocity_z),
        -kMaxTilt,
        kMaxTilt
    );
    float tilt_z = std::clamp(
        (kBallGain * ball[0]) + (kBallRateGain * velocity_x),
        -kMaxTilt,
        kMaxTilt
    );
    HoldPivot(sim_, rigs_[1].tilt_x, rigs_[1].gimbal, tilt_x, false);
    HoldPivot(sim_, rigs_[1].tilt_z, rigs_[1].plate, tilt_z, true);
  }));

  auto caught = BallOnPlate(sim_, rigs_[1]);
  EXPECT_GT(caught[1], 0.0F);  // still on top of its plate
  EXPECT_LT(std::hypot(caught[0], caught[2]), 0.02F);
  auto dropped = BallOnPlate(sim_, rigs_[0]);
  EXPECT_LT(dropped[1], 0.0F);  // below its plate's pivot: fell off
}
