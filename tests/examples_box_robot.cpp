// Validates examples/box_robot/box_robot.arow + sim_config.yaml -- the scene
// scripts/run-box-robot.sh drives -- end to end: actuators on the wheel
// joints, the wheels' contacts with the ground, and the free-floating
// chassis together. Each piece has its own tests; this is the one place
// that checks they compose into a robot that sits still, drives, and turns
// the way box_robot.cpp's controller assumes.
//
// Reads the real files rather than a copy, so an edit to the scene that
// breaks it fails here.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "engine/topology/orderings.hpp"
#include "interface/actuator.hpp"
#include "interface/simulation.hpp"

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::AngularActuator;
using achilles::interface::Simulation;

namespace {

constexpr ScalarOperationT kDt = 0.004F;

// Simulated time per test. Short, because a Debug-build tick of this scene
// is slow; long enough that free fall, drive, and turning are each far
// outside the tolerances below.
constexpr float kSeconds = 0.25F;

std::filesystem::path ExampleDir() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "examples" / "box_robot";
}

std::string ArowPath() { return (ExampleDir() / "box_robot.arow").string(); }

struct Pose {
  float x;
  float y;
  float z;
  float heading;  // yaw about +y; 0 faces +z
};

// Same convention as box_robot.cpp's ChassisPose, plus height.
Pose ChassisPose(const Simulation& sim, std::size_t row) {
  auto world = sim.ViewFor<ABAAlgorithm>()
                   .Load<ABAField::kWorldTransform, ScalarOperationT>(row);
  const auto& q = world.Rotation();
  float forward_x = 2.0F * ((q.X() * q.Z()) + (q.W() * q.Y()));
  float forward_z = 1.0F - (2.0F * ((q.X() * q.X()) + (q.Y() * q.Y())));
  return {
      world.Translation().X(),
      world.Translation().Y(),
      world.Translation().Z(),
      std::atan2(forward_x, forward_z)
  };
}

// One actuator per wheel, each about its own axle (local X), as the
// example attaches them.
struct Wheels {
  std::vector<AngularActuator> left;
  std::vector<AngularActuator> right;
};

Wheels AttachWheels(const Simulation& sim) {
  auto attach = [&](const char* name) {
    return sim.AttachActuator<AngularActuator>({"box_robot", name}, {1, 0, 0});
  };
  return {
      {attach("left_front"), attach("left_rear")},
      {attach("right_front"), attach("right_rear")}
  };
}

// Applies `left`/`right` N*m to each side's wheels every tick for `seconds`.
// Returns false if any Step fails.
bool Drive(
    Simulation& sim,
    const Wheels& wheels,
    float left,
    float right,
    float seconds
) {
  for (float t = 0.0F; t < seconds; t += kDt) {
    for (const AngularActuator& wheel : wheels.left) {
      wheel.Apply(left);
    }
    for (const AngularActuator& wheel : wheels.right) {
      wheel.Apply(right);
    }
    if (!sim.Step(kDt)) {
      return false;
    }
  }
  return true;
}

class BoxRobotScene : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(sim_.LoadArowFile(ArowPath()));
    ASSERT_TRUE(sim_.LoadConfigFile((ExampleDir() / "sim_config.yaml").string())
    );
    ASSERT_TRUE(sim_.Init());
    // The integrator box_robot.cpp runs with.
    sim_.SetIntegrator(Simulation::Integrator::kSemiImplicitEuler);
    chassis_ = sim_.RowFor<TopologicalOrdering>({"box_robot", "chassis"});
  }

  Simulation sim_;
  std::size_t chassis_ = 0;
};

}  // namespace

// With no effort the robot must come to rest on its wheels: neither sink
// through the ground nor creep across it. Anything else and the driving
// tests below are measuring drift, not drive.
TEST_F(BoxRobotScene, UndrivenRobotRestsInPlace) {
  Wheels wheels = AttachWheels(sim_);
  Pose start = ChassisPose(sim_, chassis_);
  ASSERT_TRUE(Drive(sim_, wheels, 0.0F, 0.0F, kSeconds));
  Pose end = ChassisPose(sim_, chassis_);

  EXPECT_NEAR(end.y, start.y, 0.02F);
  EXPECT_NEAR(end.x, start.x, 0.01F);
  EXPECT_NEAR(end.z, start.z, 0.01F);
  EXPECT_NEAR(end.heading, start.heading, 0.01F);
}

// Equal positive torque on every wheel drives the robot along its local
// +z, straight -- the convention box_robot.cpp's DriveWheels relies on.
TEST_F(BoxRobotScene, EqualWheelTorqueDrivesStraightForward) {
  Wheels wheels = AttachWheels(sim_);
  Pose start = ChassisPose(sim_, chassis_);
  ASSERT_TRUE(Drive(sim_, wheels, 1.0F, 1.0F, kSeconds));
  Pose end = ChassisPose(sim_, chassis_);

  EXPECT_GT(end.z - start.z, 0.05F);
  EXPECT_NEAR(end.x, start.x, 0.05F);
  EXPECT_NEAR(end.heading, start.heading, 0.05F);
  EXPECT_NEAR(end.y, start.y, 0.02F);
}

// Left wheels forward, right wheels back turns the robot in place toward
// +heading -- the sign box_robot.cpp's skid steering assumes (a positive
// yaw-rate command speeds the left side up).
TEST_F(BoxRobotScene, LeftForwardRightBackTurnsTowardPositiveHeading) {
  Wheels wheels = AttachWheels(sim_);
  Pose start = ChassisPose(sim_, chassis_);
  ASSERT_TRUE(Drive(sim_, wheels, 1.0F, -1.0F, kSeconds));
  Pose end = ChassisPose(sim_, chassis_);

  EXPECT_GT(end.heading - start.heading, 0.05F);
  EXPECT_NEAR(end.x, start.x, 0.1F);
  EXPECT_NEAR(end.z, start.z, 0.1F);
}
