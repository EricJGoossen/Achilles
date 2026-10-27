#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/joint_limit/joint_limit_data.hpp"
#include "algorithms/joint_limit/joint_limit_step.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "support/registered_sim.hpp"
#include "support/temp_dir.hpp"
#include "util/tmp.hpp"

// JointLimitStep wires DetectJointLimitOp (algorithms_joint_limit_
// joint_limit_ops.cpp covers its math) over a real JointLimitView: one
// linear pass over every row of the TopologicalOrdering topology, reading
// whatever pose ABA's (shared) kJointPosition holds now. What's under test
// here is that wiring -- every joint's rows written, at its own row, from
// this tick's pose. That it runs as part of the collision pipeline is
// algorithms_integrators.cpp's to check.

using namespace achilles::algorithms;
using achilles::engine::topology::TopologicalOrdering;
using achilles::test_support::Dof0LimitFields;
using achilles::test_support::kPendulumInertia;
using achilles::test_support::MakeRegisteredSim;
using achilles::test_support::RegisteredSim;
using achilles::test_support::RevoluteZJointArow;
using achilles::test_support::TempDir;
using achilles::util::TypeList;
using joint_limit::JointLimitAlgorithm;
using joint_limit::JointLimitField;
using joint_limit::JointLimitStep;

namespace {

using B = MathematicalT;

void RunDetector(const RegisteredSim& sim) {
  sim.SimContext().Step<TypeList<JointLimitStep>>(0.0F, SimConfig{});
}

struct LimitRow {
  float distance;
  float side;
  bool active;
};

// DOF 0's row for joint `joint` of the one archetype.
LimitRow Dof0Row(const RegisteredSim& sim, std::size_t joint) {
  std::size_t row = sim.LayoutFor<TopologicalOrdering>().ToSorted(0, joint);
  auto view = sim.ViewFor<JointLimitAlgorithm>();
  return {
      view.Load<JointLimitField::kLimitDistance, ScalarOperationT>(row)[0],
      view.Load<JointLimitField::kLimitSide, ScalarOperationT>(row)[0],
      (view.Load<JointLimitField::kLimitActive, std::int32_t>(row)
           .AsStorage() &
       1) != 0
  };
}

}  // namespace

// One hinge 0.2 past its upper end: the Step leaves that joint's row
// negative, at the upper side, active.
TEST(JointLimitStep, DetectsAJointPastItsLimit) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      RevoluteZJointArow(
          "hinge", "", kPendulumInertia, 1.2F, 0.0F, Dof0LimitFields(-1.0F, 1.0F)
      )
  );
  RunDetector(sim);

  LimitRow row = Dof0Row(sim, 0);
  EXPECT_NEAR(row.distance, -0.2F, 1e-5F);
  EXPECT_FLOAT_EQ(row.side, -1.0F);
  EXPECT_TRUE(row.active);
}

// Two joints of one chain land in different rows (a parent and its child
// never share a lane group); each gets its own range's row, not its
// neighbour's.
TEST(JointLimitStep, EachJointGetsItsOwnRow) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      RevoluteZJointArow(
          "upper", "", kPendulumInertia, 0.9F, 0.0F, Dof0LimitFields(-1.0F, 0.5F)
      ) +
          RevoluteZJointArow(
              "lower", "upper", kPendulumInertia, -0.1F, 0.0F,
              Dof0LimitFields(0.0F, 2.0F)
          )
  );
  RunDetector(sim);

  LimitRow upper = Dof0Row(sim, 0);
  EXPECT_NEAR(upper.distance, -0.4F, 1e-5F);
  EXPECT_FLOAT_EQ(upper.side, -1.0F);
  EXPECT_TRUE(upper.active);

  LimitRow lower = Dof0Row(sim, 1);
  EXPECT_NEAR(lower.distance, -0.1F, 1e-5F);
  EXPECT_FLOAT_EQ(lower.side, 1.0F);
  EXPECT_TRUE(lower.active);
}

// The Step reads the pose ABA holds when it runs, not the one it was built
// with: moving q back inside the range and stepping again clears the row.
TEST(JointLimitStep, ReadsThePoseAtTheTimeItRuns) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      RevoluteZJointArow(
          "hinge", "", kPendulumInertia, 1.2F, 0.0F, Dof0LimitFields(-1.0F, 1.0F)
      )
  );
  RunDetector(sim);
  ASSERT_TRUE(Dof0Row(sim, 0).active);

  std::size_t row = sim.LayoutFor<TopologicalOrdering>().ToSorted(0, 0);
  sim.ViewFor<aba::ABAAlgorithm>()
      .Store<aba::ABAField::kJointPosition, ScalarOperationT>(
          row, spatial::Transform<ScalarOperationT>::Identity()
      );
  RunDetector(sim);

  LimitRow moved = Dof0Row(sim, 0);
  EXPECT_NEAR(moved.distance, 1.0F, 1e-5F);
  EXPECT_FALSE(moved.active);
}

// A joint with no limit fields comes out of the Step with no active rows,
// however far it has turned.
TEST(JointLimitStep, UnlimitedJointNeverActivates) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir, RevoluteZJointArow("hinge", "", kPendulumInertia, 3.0F, 0.0F)
  );
  RunDetector(sim);
  EXPECT_FALSE(Dof0Row(sim, 0).active);
}
