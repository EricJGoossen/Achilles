// Runs examples/robot_on_plane/robot_on_plane.arow with its own
// sim_config.yaml -- the scene scripts/run-robot-demo.sh launches -- loaded
// from the real files, through semi-implicit Euler (ABA, the collision
// pipeline, the contact solver, VI, PI) at the demo's dt, and checks the
// thing the demo is for: the passive robot topples, nothing passes through
// the ground, and the torso comes to rest on it.
//
// Not "every joint at rest": the scene has no joint damping and no
// self-collision, so once the robot lies down an arm whose swing plane is
// level with the ground can keep turning about its shoulder indefinitely,
// clear of the ground, with nothing to stop it.
//
// The horizon is kept short (the torso settles by ~1.5 s) because a Debug
// build runs this scene at roughly 0.1 s per tick.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <filesystem>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/integrators.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/archetype_loader.hpp"
#include "interface/sim_config_loader.hpp"
#include "support/registered_sim.hpp"

using namespace achilles::algorithms;
using achilles::engine::topology::TopologicalOrdering;
using achilles::test_support::RegisteredSim;

namespace {

using F = ScalarOperationT;

const std::filesystem::path kExampleDir =
    std::filesystem::path(ACHILLES_SOURCE_DIR) / "examples" / "robot_on_plane";

// scripts/run-robot-demo.sh's --dt.
constexpr F kDt = 0.004F;
constexpr std::size_t kTicks = 450;  // 1.8 s

// robot_on_plane.arow's joint order: world and ground (welded), then the
// robot -- torso (free joint) first.
constexpr std::size_t kTorso = 2;
constexpr std::size_t kJoints = 9;

struct RobotOnPlane {
  RegisteredSim sim;
  SimConfig config;

  RobotOnPlane()
      : sim(achilles::interface::LoadArchetypes(
            (kExampleDir / "robot_on_plane.arow").string()
        )),
        config(
            achilles::interface::LoadSimConfig(kExampleDir / "sim_config.yaml")
        ) {
    sim.SimContext().Step<achilles::algorithms::Setup>(0.0F, config);
  }

  std::size_t Row(std::size_t joint) const {
    return sim.LayoutFor<TopologicalOrdering>().ToSorted(0, joint);
  }
  F Height(std::size_t joint) const {
    return sim.ViewFor<aba::ABAAlgorithm>()
        .Load<aba::ABAField::kWorldTransform, F>(Row(joint))
        .Translation()
        .Y();
  }
  void Tick() const { sim.SimContext().Step<SemiImplicitEuler>(kDt, config); }
};

}  // namespace

// Every robot body's frame sits inside its own collision shape, so one
// below the ground's top face (y = 0) means the contact solver let a body
// through the plane -- checked at every tick, not just at the end, along
// with every height staying finite.
TEST(RobotOnPlane, TopplesWithoutAnyBodyPassingThroughTheGround) {
  RobotOnPlane robot;
  F start = robot.Height(kTorso);
  for (std::size_t tick = 0; tick < kTicks; ++tick) {
    robot.Tick();
    for (std::size_t joint = kTorso; joint < kJoints; ++joint) {
      F y = robot.Height(joint);
      ASSERT_TRUE(std::isfinite(y)) << "joint " << joint << ", tick " << tick;
      ASSERT_GT(y, 0.0F) << "joint " << joint << ", tick " << tick;
    }
  }
  // It fell over: the hips started 0.8 up and end lying on the ground.
  EXPECT_LT(robot.Height(kTorso), 0.3F * start);
}

// Once down, the torso rests on the ground: its free joint's velocity has
// died out and its height holds still, rather than jittering or sinking
// through the soft contact.
TEST(RobotOnPlane, TorsoComesToRestOnTheGround) {
  RobotOnPlane robot;
  for (std::size_t tick = 0; tick < kTicks - 50; ++tick) {
    robot.Tick();
  }
  F settled = robot.Height(kTorso);
  for (std::size_t tick = 0; tick < 50; ++tick) {
    robot.Tick();
  }
  EXPECT_NEAR(robot.Height(kTorso), settled, 1e-3F);

  auto qd = robot.sim.ViewFor<aba::ABAAlgorithm>()
                .Load<aba::ABAField::kJointVelocity, F>(robot.Row(kTorso));
  for (std::size_t k = 0; k < 6; ++k) {
    EXPECT_NEAR(qd[k], 0.0F, 0.05F) << "torso qd[" << k << "]";
  }
}
