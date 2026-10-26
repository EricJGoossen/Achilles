#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/contact_solver/contact_solver_data.hpp"
#include "algorithms/contact_solver/contact_solver_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/narrow_phase/narrow_phase_data.hpp"
#include "algorithms/narrow_phase/narrow_phase_step.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "support/registered_sim.hpp"
#include "support/temp_dir.hpp"
#include "util/simd_ops.hpp"

// contact_solver_data.hpp's memory contract, through every hosted Algorithm
// carved together (as interface::Simulation does):
//   1. The per-body contact parameters and per-joint limit parameters load
//      from .arow data by name, and read as zero when a body gives none
//      (which the Ops take as MuJoCo's defaults --
//      algorithms_contact_solver_contact_solver_ops.cpp).
//   2. Every "(shared, read-only here)" field is the other algorithm's own
//      block: ABA's factors and state (including the SharedAs slots ABA
//      gained for this, kRigidBodyInertia through kDInvTerm) and the narrow
//      phase's contacts. The joint-limit detector's rows are checked from
//      its side, in algorithms_joint_limit_joint_limit_data.cpp.

using namespace achilles::algorithms;
using achilles::engine::topology::TopologicalOrdering;
using achilles::test_support::kPendulumInertia;
using achilles::test_support::MakeRegisteredSim;
using achilles::test_support::RegisteredSim;
using achilles::test_support::RevoluteZJointArow;
using achilles::test_support::TempDir;
using contact_solver::ContactSolverAlgorithm;
using contact_solver::ContactSolverField;

namespace {

using B = MathematicalT;
using F = ScalarOperationT;

std::size_t JointRow(const RegisteredSim& sim) {
  return sim.LayoutFor<TopologicalOrdering>().ToSorted({0, 0});
}

RegisteredSim MakeHinge(const TempDir& dir, const std::string& extra = "") {
  return MakeRegisteredSim(
      dir, RevoluteZJointArow("hinge", "", kPendulumInertia, 0.0F, 0.0F, extra)
  );
}

}  // namespace

TEST(ContactSolverData, ContactAndLimitParametersLoadFromArowByName) {
  TempDir dir;
  RegisteredSim sim = MakeHinge(
      dir,
      "      contact_friction: [0.7]\n"
      "      contact_solref: [0.05, 0.9]\n"
      "      contact_solimp: [0.8, 0.9, 0.01, 0.4, 3]\n"
      "      joint_limit_solref: [0.1, 1.1]\n"
      "      joint_limit_solimp: [0.7, 0.8, 0.02, 0.3, 4]\n"
  );
  auto view = sim.ViewFor<ContactSolverAlgorithm>();
  std::size_t row = JointRow(sim);

  EXPECT_FLOAT_EQ(
      (view.Load<ContactSolverField::kContactFriction, F>(row).Value()), 0.7F
  );
  auto contact_solref = view.Load<ContactSolverField::kContactSolref, F>(row);
  EXPECT_FLOAT_EQ(contact_solref(0, 0), 0.05F);
  EXPECT_FLOAT_EQ(contact_solref(1, 0), 0.9F);
  auto contact_solimp = view.Load<ContactSolverField::kContactSolimp, F>(row);
  EXPECT_FLOAT_EQ(contact_solimp(0, 0), 0.8F);
  EXPECT_FLOAT_EQ(contact_solimp(4, 0), 3.0F);
  auto limit_solref = view.Load<ContactSolverField::kJointLimitSolref, F>(row);
  EXPECT_FLOAT_EQ(limit_solref(0, 0), 0.1F);
  EXPECT_FLOAT_EQ(limit_solref(1, 0), 1.1F);
  auto limit_solimp = view.Load<ContactSolverField::kJointLimitSolimp, F>(row);
  EXPECT_FLOAT_EQ(limit_solimp(0, 0), 0.7F);
  EXPECT_FLOAT_EQ(limit_solimp(4, 0), 4.0F);
}

// Unset parameters read as zero -- the "use MuJoCo's default" marker every
// Op that reads them keys on.
TEST(ContactSolverData, UnsetParametersReadAsZero) {
  TempDir dir;
  RegisteredSim sim = MakeHinge(dir);
  auto view = sim.ViewFor<ContactSolverAlgorithm>();
  std::size_t row = JointRow(sim);

  EXPECT_FLOAT_EQ(
      (view.Load<ContactSolverField::kContactFriction, F>(row).Value()), 0.0F
  );
  EXPECT_TRUE((view.Load<ContactSolverField::kContactSolref, F>(row).IsZero()));
  EXPECT_TRUE((view.Load<ContactSolverField::kContactSolimp, F>(row).IsZero()));
  EXPECT_TRUE(
      (view.Load<ContactSolverField::kJointLimitSolref, F>(row).IsZero())
  );
  EXPECT_TRUE(
      (view.Load<ContactSolverField::kJointLimitSolimp, F>(row).IsZero())
  );
}

// Each of ABA's blocks the solver names is ABA's own memory: a value ABA's
// view stores is the value the solver's view loads, field by field.
TEST(ContactSolverData, AbaFactorsAndStateAliasAbas) {
  TempDir dir;
  RegisteredSim sim = MakeHinge(dir);
  auto aba_view = sim.ViewFor<aba::ABAAlgorithm>();
  auto solver_view = sim.ViewFor<ContactSolverAlgorithm>();
  std::size_t group = JointRow(sim) / B::size;
  using aba::ABAField;

  Matrix6x6 marker = Matrix6x6::Identity() * B(3.0F);
  Transform pose(Vector3(B(1.0F), B(2.0F), B(3.0F)), Quaternion::Identity());
  Velocity velocity(Vector3(B(0.5F), B(0.0F), B(0.0F)), Vector3::Zero());
  Acceleration acceleration(
      Vector3(B(0.0F), B(-4.0F), B(0.0F)), Vector3::Zero()
  );
  Inertia inertia(
      B(3.0F), Vector3::Zero(), B(1.0F), B(1.0F), B(1.0F), B(0.0F), B(0.0F),
      B(0.0F)
  );

  aba_view.Store<ABAField::kRigidBodyInertia, B>(group, inertia);
  aba_view.Store<ABAField::kParentToBodyTransform, B>(group, pose);
  aba_view.Store<ABAField::kWorldTransform, B>(group, pose);
  aba_view.Store<ABAField::kSpatialVelocity, B>(group, velocity);
  aba_view.Store<ABAField::kJointVelocity, B>(group, velocity);
  aba_view.Store<ABAField::kJointAcceleration, B>(group, acceleration);
  aba_view.Store<ABAField::kUTerm, B>(group, InertiaOperator<false>(marker));
  aba_view.Store<ABAField::kDInvTerm, B>(group, InertiaOperator<true>(marker));

  auto all = [](const auto& mask) { return achilles::util::AllTrue(mask); };
  EXPECT_TRUE(all(solver_view.Load<ContactSolverField::kRigidBodyInertia, B>(group)
                      .IsApprox(inertia)));
  EXPECT_TRUE(all(
      solver_view.Load<ContactSolverField::kParentToBodyTransform, B>(group)
          .Translation()
          .IsApprox(pose.Translation())
  ));
  EXPECT_TRUE(all(solver_view.Load<ContactSolverField::kWorldTransform, B>(group)
                      .Translation()
                      .IsApprox(pose.Translation())));
  EXPECT_TRUE(all(solver_view.Load<ContactSolverField::kSpatialVelocity, B>(group)
                      .IsApprox(velocity)));
  EXPECT_TRUE(all(solver_view.Load<ContactSolverField::kJointVelocity, B>(group)
                      .IsApprox(velocity)));
  EXPECT_TRUE(all(
      solver_view.Load<ContactSolverField::kJointAcceleration, B>(group)
          .IsApprox(acceleration)
  ));
  EXPECT_TRUE(all(solver_view.Load<ContactSolverField::kUTerm, B>(group)
                      .AsMatrix()
                      .IsApprox(marker)));
  EXPECT_TRUE(all(solver_view.Load<ContactSolverField::kDInvTerm, B>(group)
                      .AsMatrix()
                      .IsApprox(marker)));
  EXPECT_FLOAT_EQ(
      (solver_view.Load<ContactSolverField::kJointSubspace, F>(JointRow(sim))(
          2, 0
      )),
      1.0F
  );
}

// The narrow phase's contacts and margin are the solver's inputs: what the
// narrow phase stores at a pair element, the solver loads there.
TEST(ContactSolverData, NarrowPhaseContactsAliasTheNarrowPhases) {
  using narrow_phase::NarrowPhaseAlgorithm;
  using narrow_phase::NarrowPhaseField;
  TempDir dir;
  RegisteredSim sim =
      MakeHinge(dir, "      collision_margin: [0.125]\n");
  auto narrow_view = sim.ViewFor<NarrowPhaseAlgorithm>();
  auto solver_view = sim.ViewFor<ContactSolverAlgorithm>();
  std::size_t pair = 0;

  narrow_phase::ContactDistances distance = narrow_phase::ContactDistances::Zero();
  distance(2, 0) = B(-0.25F);
  narrow_phase::ContactPoints point = narrow_phase::ContactPoints::Zero();
  point(1, 3) = B(4.0F);
  Vector3 normal(B(0.0F), B(1.0F), B(0.0F));
  narrow_view.Store<NarrowPhaseField::kContactDistance, B>(pair, distance);
  narrow_view.Store<NarrowPhaseField::kContactPoint, B>(pair, point);
  narrow_view.Store<NarrowPhaseField::kContactNormal, B>(pair, normal);
  narrow_view.Store<NarrowPhaseField::kContactActive, std::int32_t>(
      pair, narrow_phase::ContactActiveMask(std::int32_t{0b101})
  );

  EXPECT_FLOAT_EQ(
      (solver_view.Load<ContactSolverField::kContactDistance, B>(pair)(2, 0)
           .get(0)),
      -0.25F
  );
  EXPECT_FLOAT_EQ(
      (solver_view.Load<ContactSolverField::kContactPoint, B>(pair)(1, 3).get(0)),
      4.0F
  );
  EXPECT_FLOAT_EQ(
      (solver_view.Load<ContactSolverField::kContactNormal, B>(pair).Y().get(0)),
      1.0F
  );
  EXPECT_EQ(
      (solver_view.Load<ContactSolverField::kContactActive, std::int32_t>(pair)
           .AsStorage()),
      0b101
  );
  EXPECT_FLOAT_EQ(
      (solver_view.Load<ContactSolverField::kCollisionMargin, F>(JointRow(sim))
           .Value()),
      0.125F
  );
}
