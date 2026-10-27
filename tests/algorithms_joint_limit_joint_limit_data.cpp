#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/contact_solver/contact_solver_data.hpp"
#include "algorithms/contact_solver/contact_solver_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/joint_limit/joint_limit_data.hpp"
#include "algorithms/joint_limit/joint_limit_step.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "support/registered_sim.hpp"
#include "support/temp_dir.hpp"
#include "util/simd_ops.hpp"

// joint_limit_data.hpp's memory contract, through every hosted Algorithm
// carved together (as interface::Simulation does):
//   1. The four limit fields load from .arow data by name.
//   2. A joint that gives none reads the zero-fill: nothing limited.
//   3. kJointSubspace/kJointPosition are ABA's own blocks, and the
//      detector's outputs and margin are the contact solver's -- the
//      SharedAs slots that let ABA's pose reach the detector and its rows
//      reach the solver without a copy.

using namespace achilles::algorithms;
using achilles::engine::topology::TopologicalOrdering;
using achilles::test_support::Dof0LimitFields;
using achilles::test_support::kPendulumInertia;
using achilles::test_support::MakeRegisteredSim;
using achilles::test_support::RegisteredSim;
using achilles::test_support::RevoluteZJointArow;
using achilles::test_support::TempDir;
using joint_limit::JointLimitAlgorithm;
using joint_limit::JointLimitField;

namespace {

using B = MathematicalT;
using MaskBatch = achilles::util::MaskStorageFor<B>;

// The one joint's row, scalar-indexed.
std::size_t JointRow(const RegisteredSim& sim) {
  return sim.LayoutFor<TopologicalOrdering>().ToSorted({0, 0});
}

}  // namespace

TEST(JointLimitData, LimitFieldsLoadFromArowByName) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      RevoluteZJointArow(
          "hinge", "", kPendulumInertia, 0.0F, 0.0F,
          Dof0LimitFields(-0.5F, 0.75F, 0.05F)
      )
  );
  auto view = sim.ViewFor<JointLimitAlgorithm>();
  std::size_t row = JointRow(sim);

  EXPECT_EQ(
      (view.Load<JointLimitField::kJointLimited, std::int32_t>(row)
           .AsStorage()),
      1
  );
  EXPECT_FLOAT_EQ(
      (view.Load<JointLimitField::kJointRangeLower, ScalarOperationT>(row)[0]),
      -0.5F
  );
  EXPECT_FLOAT_EQ(
      (view.Load<JointLimitField::kJointRangeUpper, ScalarOperationT>(row)[0]),
      0.75F
  );
  EXPECT_FLOAT_EQ(
      (view.Load<JointLimitField::kJointLimitMargin, ScalarOperationT>(row)
           .Value()),
      0.05F
  );
}

// No limit fields at all -- every existing model -- must read as "no DOF
// limited", so adding the detector changes nothing for them.
TEST(JointLimitData, JointWithoutLimitFieldsIsUnlimited) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir, RevoluteZJointArow("hinge", "", kPendulumInertia, 0.0F, 0.0F)
  );
  auto view = sim.ViewFor<JointLimitAlgorithm>();

  EXPECT_EQ(
      (view.Load<JointLimitField::kJointLimited, std::int32_t>(
               JointRow(sim)
           )
           .AsStorage()),
      0
  );
}

// The pose the detector reads is ABA's own: a q stored through ABA's view
// is the q the detector's view loads, never a private copy.
TEST(JointLimitData, SubspaceAndPositionAliasAbas) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir, RevoluteZJointArow("hinge", "", kPendulumInertia, 0.0F, 0.0F)
  );
  auto aba_view = sim.ViewFor<aba::ABAAlgorithm>();
  auto limit_view = sim.ViewFor<JointLimitAlgorithm>();
  std::size_t row = JointRow(sim);

  math::Quaternion<ScalarOperationT> rotation =
      math::Quaternion<ScalarOperationT>::Exp(
          math::Vector3<ScalarOperationT>(0.0F, 0.0F, 0.4F)
      );
  aba_view.Store<aba::ABAField::kJointPosition, ScalarOperationT>(
      row,
      spatial::Transform<ScalarOperationT>(
          math::Vector3<ScalarOperationT>::Zero(), rotation
      )
  );

  auto q = limit_view.Load<JointLimitField::kJointPosition, ScalarOperationT>(
      row
  );
  EXPECT_TRUE(q.Rotation().IsApprox(rotation));
  EXPECT_FLOAT_EQ(
      (limit_view.Load<JointLimitField::kJointSubspace, ScalarOperationT>(row)(
          2, 0
      )),
      1.0F
  );
}

// The detector's outputs and its margin are the contact solver's inputs:
// what the detector stores, the solver loads.
TEST(JointLimitData, OutputsAndMarginAliasTheContactSolvers) {
  using contact_solver::ContactSolverAlgorithm;
  using contact_solver::ContactSolverField;
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      RevoluteZJointArow(
          "hinge", "", kPendulumInertia, 0.0F, 0.0F,
          Dof0LimitFields(-1.0F, 1.0F, 0.25F)
      )
  );
  auto limit_view = sim.ViewFor<JointLimitAlgorithm>();
  auto solver_view = sim.ViewFor<ContactSolverAlgorithm>();
  std::size_t group = JointRow(sim) / B::size;

  joint_limit::JointCoordinates distance = joint_limit::JointCoordinates::Zero();
  distance[0] = B(-0.125F);
  joint_limit::JointCoordinates side = joint_limit::JointCoordinates::Zero();
  side[0] = B(-1.0F);
  limit_view.Store<JointLimitField::kLimitDistance, B>(group, distance);
  limit_view.Store<JointLimitField::kLimitSide, B>(group, side);
  limit_view.Store<JointLimitField::kLimitActive, MaskBatch>(
      group, Mat6Mask(MaskBatch(std::int32_t{1}))
  );

  EXPECT_FLOAT_EQ(
      (solver_view.Load<ContactSolverField::kLimitDistance, B>(group)[0].get(0)),
      -0.125F
  );
  EXPECT_FLOAT_EQ(
      (solver_view.Load<ContactSolverField::kLimitSide, B>(group)[0].get(0)),
      -1.0F
  );
  EXPECT_EQ(
      (solver_view.Load<ContactSolverField::kLimitActive, MaskBatch>(group)
           .AsStorage()
           .get(0)),
      1
  );
  EXPECT_FLOAT_EQ(
      (solver_view.Load<ContactSolverField::kJointLimitMargin, ScalarOperationT>(
           JointRow(sim)
       )
           .Value()),
      0.25F
  );
}
