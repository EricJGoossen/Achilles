#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/contact_solver/contact_solver_data.hpp"
#include "algorithms/contact_solver/contact_solver_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/integrators.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/math/scalar.hpp"
#include "domain/spatial/dual.hpp"
#include "engine/topology/orderings.hpp"
#include "support/registered_sim.hpp"
#include "support/temp_dir.hpp"
#include "util/tmp.hpp"

// The contact solver's Steps composed (contact_solver_step.hpp): each Op's
// own math is algorithms_contact_solver_contact_solver_ops.cpp's; what's
// under test here is that the passes are wired together right -- the
// right fields between them, the tree passes walking real chains, the
// reductions adding up across the contact and limit halves -- checked
// against answers known in closed form:
//
//   - Two coaxial hinges, I1 = 4 and I2 = 1 about the shared axis, have
//     M = [[I1 + I2, I2], [I2, I2]] exactly, so DirectionStep's M search,
//     UpdateStep's M^-1 grad and InvWeightStep's dof_invweight0 =
//     diag(M^-1) are all known.
//   - With one DOF and every active row sharing J = 1 (or J = side), the
//     solve minimizes 1/2 M (a - a0)^2 + sum 1/2 D (J a - aref)^2, whose
//     minimum is a* = (M a0 + sum D J aref) / (M + sum D) while every row
//     stays active there. A pendulum past its limit and a ball resting on
//     the ground are each that, with D and aref re-derived from MuJoCo's
//     formulas below.

using namespace achilles::algorithms;
using achilles::engine::topology::TopologicalOrdering;
using achilles::test_support::Dof0LimitFields;
using achilles::test_support::kPendulumInertia;
using achilles::test_support::kPendulumIzz;
using achilles::test_support::kPendulumK;
using achilles::test_support::MakeRegisteredSim;
using achilles::test_support::PendulumConfig;
using achilles::test_support::RegisteredSim;
using achilles::test_support::RevoluteZJointArow;
using achilles::test_support::TempDir;
using achilles::util::TypeList;
using contact_solver::ContactSolverAlgorithm;
using contact_solver::ContactSolverField;

namespace {

using F = ScalarOperationT;

constexpr F kDt = 0.002F;

// MuJoCo's default solref/solimp, as a contact or limit with none reads
// them: deeper than the 0.001 width the impedance is dwidth = 0.95, and
// K = 1 / (dw tc dr)^2 with tc = 0.02 (2 dt = 0.004 doesn't raise it).
constexpr double kImpedance = 0.95;
constexpr double kStiffness = 1.0 / (0.95 * 0.95 * 0.02 * 0.02);

const char* const kAxleInertia = "[2, 0,0,0, 2,3,4, 0,0,0]";   // Izz = 4
const char* const kWheelInertia = "[1, 0,0,0, 1,1,1, 0,0,0]";  // Izz = 1

std::size_t Row(const RegisteredSim& sim, std::size_t joint) {
  return sim.LayoutFor<TopologicalOrdering>().ToSorted({0, joint});
}

// Two hinges on one axis: the wheel's frame sits on the axle's.
RegisteredSim MakeCoaxialHinges(const TempDir& dir) {
  return MakeRegisteredSim(
      dir,
      RevoluteZJointArow("axle", "", kAxleInertia, 0.0F, 0.0F) +
          RevoluteZJointArow("wheel", "axle", kWheelInertia, 0.0F, 0.0F)
  );
}

// algorithms::Setup (named apart from gtest's own Test::Setup).
using SetupList = achilles::algorithms::Setup;

template <typename List>
void RunSteps(const RegisteredSim& sim, const SimConfig& config, F dt = kDt) {
  sim.SimContext().Step<List>(dt, config);
}

// mj_forward up to the constrained qacc: ABA, the collision pipeline, the
// whole solve -- SemiImplicitEuler's list, without VI/PI's integration.
using Forward = achilles::util::ConcatT<
    TypeList<aba::ABAStep>,
    Collision,
    contact_solver::Solve>;

F Qdd(const RegisteredSim& sim, std::size_t joint) {
  return sim.ViewFor<aba::ABAAlgorithm>()
      .Load<aba::ABAField::kJointAcceleration, F>(Row(sim, joint))[0];
}

}  // namespace

// The Setup list's InvWeightStep, over ABA's factors at the initial pose:
// dof_invweight0 = diag(M^-1) = (1 / I1, 1 / I1 + 1 / I2) -- the wheel's
// includes the axle's response to its reaction, through the real tree.
TEST(InvWeightStep, DofInvweightIsTheMassMatrixInversesDiagonal) {
  TempDir dir;
  RegisteredSim sim = MakeCoaxialHinges(dir);
  RunSteps<SetupList>(sim, SimConfig{}, 0.0F);

  auto view = sim.ViewFor<ContactSolverAlgorithm>();
  EXPECT_NEAR(
      (view.Load<ContactSolverField::kDofInvWeight, F>(Row(sim, 0))[0]), 0.25F,
      1e-5F
  );
  EXPECT_NEAR(
      (view.Load<ContactSolverField::kDofInvWeight, F>(Row(sim, 1))[0]), 1.25F,
      1e-5F
  );
}

// DirectionStep's M search, through the real forward/backward tree passes:
// M (0.7, -0.3) = (5 * 0.7 - 0.3, 0.7 - 0.3) = (3.2, 0.4), and the Gauss
// term search . M search it reduces = 0.7 * 3.2 - 0.3 * 0.4 = 2.12.
TEST(DirectionStep, MSearchIsTheMassMatrixTimesSearch) {
  TempDir dir;
  RegisteredSim sim = MakeCoaxialHinges(dir);
  RunSteps<TypeList<aba::ABAStep>>(sim, SimConfig{});
  auto view = sim.ViewFor<ContactSolverAlgorithm>();
  view.Store<ContactSolverField::kSearch, F>(
      Row(sim, 0), spatial::SpatialAcceleration<F>(0.7F, 0, 0, 0, 0, 0)
  );
  view.Store<ContactSolverField::kSearch, F>(
      Row(sim, 1), spatial::SpatialAcceleration<F>(-0.3F, 0, 0, 0, 0, 0)
  );

  RunSteps<TypeList<contact_solver::DirectionStep>>(sim, SimConfig{});

  EXPECT_NEAR(
      (view.Load<ContactSolverField::kMSearch, F>(Row(sim, 0))[0]), 3.2F, 1e-5F
  );
  EXPECT_NEAR(
      (view.Load<ContactSolverField::kMSearch, F>(Row(sim, 1))[0]), 0.4F, 1e-5F
  );
  EXPECT_NEAR(
      (view.Load<ContactSolverField::kSearchMSearch, F>(0).Value()), 2.12F,
      1e-5F
  );
}

// UpdateStep's M^-1 grad, through the real inverse tree passes: with
// alpha = 0 and no rows, grad = Ma, and M^-1 (3.2, 0.4) = (0.7, -0.3) --
// M^-1 = [[1, -1], [-1, 5]] / 4.
TEST(UpdateStep, MgradIsTheMassMatrixInverseTimesGrad) {
  TempDir dir;
  RegisteredSim sim = MakeCoaxialHinges(dir);
  RunSteps<TypeList<aba::ABAStep>>(sim, SimConfig{});
  auto view = sim.ViewFor<ContactSolverAlgorithm>();
  view.Store<ContactSolverField::kStep, F>(0, math::Scalar<F>(0.0F));
  view.Store<ContactSolverField::kMa, F>(
      Row(sim, 0), spatial::SpatialForce<F>(3.2F, 0, 0, 0, 0, 0)
  );
  view.Store<ContactSolverField::kMa, F>(
      Row(sim, 1), spatial::SpatialForce<F>(0.4F, 0, 0, 0, 0, 0)
  );

  RunSteps<TypeList<contact_solver::UpdateStep>>(sim, SimConfig{});

  EXPECT_NEAR(
      (view.Load<ContactSolverField::kMgrad, F>(Row(sim, 0))[0]), 0.7F, 1e-5F
  );
  EXPECT_NEAR(
      (view.Load<ContactSolverField::kMgrad, F>(Row(sim, 1))[0]), -0.3F, 1e-5F
  );
}

// ---------------------------------------------------------------------------
// A planar double pendulum: two revolute-z links, the second hinged 1 along
// the first's x, each kPendulumInertia (m = 1, center 0.5 along its own x,
// Izz = 0.26 about its joint). Unlike the coaxial hinges, x_up carries a
// translation, so M is configuration-dependent and every pass's force
// transform's t^ R term matters:
//
//   M11 = I1 + I2 + m2 (l^2 + 2 l c2 cos q2),  M12 = I2 + m2 l c2 cos q2,
//   M22 = I2.
// ---------------------------------------------------------------------------

namespace {

constexpr F kUpperAngle = 0.3F;
constexpr F kLowerAngle = 0.7F;

RegisteredSim MakeOffsetArm(const TempDir& dir) {
  std::string lower = RevoluteZJointArow(
      "lower", "upper", kPendulumInertia, kLowerAngle, 0.0F
  );
  const std::string at_origin = "fixed_joint_transform: [0,0,0,";
  lower.replace(
      lower.find(at_origin), at_origin.size(), "fixed_joint_transform: [1,0,0,"
  );
  return MakeRegisteredSim(
      dir,
      RevoluteZJointArow("upper", "", kPendulumInertia, kUpperAngle, 0.0F) +
          lower
  );
}

struct Mat2 {
  double a11, a12, a22;  // symmetric
};

Mat2 OffsetArmMassMatrix() {
  double i = kPendulumIzz;
  double coupling = 0.5 * std::cos(static_cast<double>(kLowerAngle));
  return {i + i + 1.0 + (2.0 * coupling), i + coupling, i};
}

Mat2 Inverse(const Mat2& m) {
  double det = (m.a11 * m.a22) - (m.a12 * m.a12);
  return {m.a22 / det, -m.a12 / det, m.a11 / det};
}

}  // namespace

// DirectionStep's M search, column by column (search = e_j), against the
// closed-form M -- including both off-diagonal entries, so M comes out
// symmetric through the real forward/backward tree passes.
TEST(DirectionStep, MSearchMatchesAnOffsetArmsClosedFormMassMatrix) {
  TempDir dir;
  RegisteredSim sim = MakeOffsetArm(dir);
  RunSteps<TypeList<aba::ABAStep>>(sim, SimConfig{});
  auto view = sim.ViewFor<ContactSolverAlgorithm>();
  Mat2 m = OffsetArmMassMatrix();
  double expected[2][2] = {{m.a11, m.a12}, {m.a12, m.a22}};

  for (std::size_t col = 0; col < 2; ++col) {
    for (std::size_t joint = 0; joint < 2; ++joint) {
      view.Store<ContactSolverField::kSearch, F>(
          Row(sim, joint),
          spatial::SpatialAcceleration<F>(
              joint == col ? 1.0F : 0.0F, 0, 0, 0, 0, 0
          )
      );
    }
    RunSteps<TypeList<contact_solver::DirectionStep>>(sim, SimConfig{});
    for (std::size_t row = 0; row < 2; ++row) {
      EXPECT_NEAR(
          (view.Load<ContactSolverField::kMSearch, F>(Row(sim, row))[0]),
          expected[row][col], 1e-4
      ) << "M(" << row << ", " << col << ")";
    }
  }
}

// UpdateStep's M^-1 grad, column by column (grad = Ma = e_j, alpha = 0, no
// rows), against the closed-form inverse of the same M.
TEST(UpdateStep, MgradMatchesAnOffsetArmsClosedFormInverse) {
  TempDir dir;
  RegisteredSim sim = MakeOffsetArm(dir);
  RunSteps<TypeList<aba::ABAStep>>(sim, SimConfig{});
  auto view = sim.ViewFor<ContactSolverAlgorithm>();
  view.Store<ContactSolverField::kStep, F>(0, math::Scalar<F>(0.0F));
  Mat2 inv = Inverse(OffsetArmMassMatrix());
  double expected[2][2] = {{inv.a11, inv.a12}, {inv.a12, inv.a22}};

  for (std::size_t col = 0; col < 2; ++col) {
    for (std::size_t joint = 0; joint < 2; ++joint) {
      view.Store<ContactSolverField::kMa, F>(
          Row(sim, joint),
          spatial::SpatialForce<F>(joint == col ? 1.0F : 0.0F, 0, 0, 0, 0, 0)
      );
    }
    RunSteps<TypeList<contact_solver::UpdateStep>>(sim, SimConfig{});
    for (std::size_t row = 0; row < 2; ++row) {
      EXPECT_NEAR(
          (view.Load<ContactSolverField::kMgrad, F>(Row(sim, row))[0]) /
              expected[row][col],
          1.0, 1e-3
      ) << "M^-1(" << row << ", " << col << ")";
    }
  }
}

// InvWeightStep's dof_invweight0 is diag(M^-1) for the offset arm too --
// the Omega recursion carried through a translated x_up.
TEST(InvWeightStep, DofInvweightMatchesAnOffsetArmsInverseDiagonal) {
  TempDir dir;
  RegisteredSim sim = MakeOffsetArm(dir);
  RunSteps<SetupList>(sim, SimConfig{}, 0.0F);
  auto view = sim.ViewFor<ContactSolverAlgorithm>();
  Mat2 inv = Inverse(OffsetArmMassMatrix());
  EXPECT_NEAR(
      (view.Load<ContactSolverField::kDofInvWeight, F>(Row(sim, 0))[0]) /
          inv.a11,
      1.0, 1e-3
  );
  EXPECT_NEAR(
      (view.Load<ContactSolverField::kDofInvWeight, F>(Row(sim, 1))[0]) /
          inv.a22,
      1.0, 1e-3
  );
}

// With no active rows the cost is Gauss's alone, minimized at
// qacc_smooth: the solve hands ABA's own qdd back unchanged.
TEST(Solve, WithNoActiveRowsQddIsUnchanged) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      RevoluteZJointArow(
          "hinge", "", kPendulumInertia, 0.3F, 0.0F, Dof0LimitFields(-1.0F, 1.0F)
      )
  );
  SimConfig config = PendulumConfig();
  RunSteps<SetupList>(sim, config, 0.0F);

  RunSteps<Forward>(sim, config);

  EXPECT_NEAR(Qdd(sim, 0), kPendulumK * std::cos(0.3), 1e-3);
}

// A pendulum 0.05 past its upper limit, at rest, pulled further past it by
// gravity: one limit row, J = side = -1, so a* = (M a0 - D aref) / (M + D)
// with M = Izz, D = 1 / ((1 - d) / d dof_invweight0), dof_invweight0 =
// 1 / Izz, and aref = -K d r. J^T f, left in kConstraintForce, is what
// holds it there: M (a* - a0).
TEST(Solve, LimitRowMatchesTheClosedFormMinimum) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      RevoluteZJointArow(
          "hinge", "", kPendulumInertia, 1.05F, 0.0F,
          Dof0LimitFields(-1.0F, 1.0F)
      )
  );
  SimConfig config = PendulumConfig();
  RunSteps<SetupList>(sim, config, 0.0F);

  RunSteps<Forward>(sim, config);

  double m = kPendulumIzz;
  double a0 = kPendulumK * std::cos(1.05);
  double r = -0.05;
  double d = 1.0 / ((1.0 - kImpedance) / kImpedance * (1.0 / m));
  double aref = -kStiffness * kImpedance * r;
  double expected = ((m * a0) - (d * aref)) / (m + d);
  ASSERT_LT(-expected - aref, 0.0);  // the row is still active there

  EXPECT_NEAR(Qdd(sim, 0) / expected, 1.0, 1e-3);
  F constraint_force = sim.ViewFor<ContactSolverAlgorithm>()
                           .Load<ContactSolverField::kConstraintForce, F>(
                               Row(sim, 0)
                           )[0];
  EXPECT_NEAR(constraint_force / (m * (expected - a0)), 1.0, 1e-3);
}

// A limited joint inside its range has no active row: unconstrained.
TEST(Solve, LimitedJointInsideItsRangeIsUnconstrained) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      RevoluteZJointArow(
          "hinge", "", kPendulumInertia, 0.9F, 0.0F, Dof0LimitFields(-1.0F, 1.0F)
      )
  );
  SimConfig config = PendulumConfig();
  RunSteps<SetupList>(sim, config, 0.0F);

  RunSteps<Forward>(sim, config);

  EXPECT_NEAR(Qdd(sim, 0), kPendulumK * std::cos(0.9), 1e-3);
}

// ---------------------------------------------------------------------------
// A ball on a slide joint (vertical), resting 0.01 into a welded ground slab
// -- the robot_on_plane.arow layout in miniature: a welded root carrying
// the ground and the ball as siblings, so their pair isn't skipped as
// parent/child, and collision groups pairing exactly those two.
// ---------------------------------------------------------------------------

namespace {

constexpr F kBallMass = 2.0F;

std::string WeldedJoint(
    const std::string& name, const std::string& parent, float y,
    const std::string& collision
) {
  std::string out = "  - name: " + name + "\n";
  if (!parent.empty()) {
    out += "    parent: " + parent + "\n";
  }
  out +=
      "    fields:\n"
      "      joint_subspace: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,"
      "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]\n"
      "      joint_activation_mask: [0]\n"
      "      fixed_joint_transform: [0," +
      std::to_string(y) +
      ",0, 1,0,0,0]\n"
      "      rigid_body_inertia: [1, 0,0,0, 0.1,0.1,0.1, 0,0,0]\n"
      "      joint_position: [0,0,0, 1,0,0,0]\n" +
      collision;
  return out;
}

// The collision fields every joint declares (LoadArchetypes takes an
// archetype's field list from its first joint, so all three must).
std::string CollisionFields(
    const std::string& extents, float radius, int group, int mask
) {
  return "      collision_pose: [0,0,0, 1,0,0,0]\n"
         "      collision_extents: " +
         extents + "\n      collision_radius: [" + std::to_string(radius) +
         "]\n      collision_group: [" + std::to_string(group) +
         "]\n      collision_mask: [" + std::to_string(mask) + "]\n";
}

// Slide along y (subspace entry (4, 0)), 2 kg, a sphere of radius 0.1 in
// collision group 2 (colliding with group 1, the ground). `x` places its
// slide sideways; `extra` is any further field lines.
std::string BallJoint(
    float y, const std::string& name = "ball", float x = 0.0F,
    float mass = kBallMass, int group = 2, int mask = 1,
    const std::string& extra = ""
) {
  return "  - name: " + name +
         "\n"
         "    parent: world\n"
         "    fields:\n"
         "      joint_subspace: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,"
         "0,0,1,0,0,0,0,0,0,0,0,0,0,0]\n"
         "      joint_activation_mask: [1]\n"
         "      fixed_joint_transform: [" +
         std::to_string(x) +
         ",0,0, 1,0,0,0]\n"
         "      rigid_body_inertia: [" +
         std::to_string(mass) +
         ", 0,0,0, 0.01,0.01,0.01, 0,0,0]\n"
         "      joint_position: [0," +
         std::to_string(y) + ",0, 1,0,0,0]\n" +
         CollisionFields("[0, 0, 0]", 0.1F, group, mask) + extra;
}

RegisteredSim MakeBallOnGround(const TempDir& dir, float ball_y) {
  // The slab's frame sits 0.1 down, its box 0.1 thick: top face at y = 0.
  // The root is in no collision group at all.
  return MakeRegisteredSim(
      dir,
      WeldedJoint("world", "", 0.0F, CollisionFields("[0, 0, 0]", 0.0F, 0, 0)) +
          WeldedJoint(
              "ground", "world", -0.1F,
              CollisionFields("[1, 0.1, 1]", 0.0F, 1, 2)
          ) +
          BallJoint(ball_y)
  );
}

// Gravity -9.8 along y.
SimConfig DownwardGravity() {
  using B = MathematicalT;
  SimConfig config;
  config.base_acceleration = Acceleration(
      Vector3::Zero(), Vector3(B(0.0F), B(9.8F), B(0.0F))
  );
  return config;
}

// How many contact rows ended up active (D > 0), over every pair element.
std::size_t ActiveContactRows(const RegisteredSim& sim) {
  auto view = sim.ViewFor<ContactSolverAlgorithm>();
  std::size_t rows = 0;
  for (std::size_t i = 0;
       i < view.NumBatches<ContactSolverField::kRowWeight, MathematicalT>();
       ++i) {
    auto weight = view.Load<ContactSolverField::kRowWeight, MathematicalT>(i);
    for (std::size_t r = 0; r < weight.Rows() * weight.Cols(); ++r) {
      for (std::size_t lane = 0; lane < MathematicalT::size; ++lane) {
        rows += weight[r].get(lane) > 0.0F ? 1 : 0;
      }
    }
  }
  return rows;
}

}  // namespace

// Every one of the contact's pyramid rows has J = n . y = 1 on the slide
// (its tangents are horizontal), the same D and the same aref, so with n of
// them active a* = (M a0 + n D aref) / (M + n D): M = 2, a0 = -9.8,
// D = 1 / (2 mu^2 (1 - d) / d (1 + mu^2) invweight), mu = 1, and the
// ball's body_invweight0 the mean over three axes of 1 / m along the one it
// can move in (the ground's is zero). Through the real broad phase, narrow
// phase and solve.
TEST(Solve, ContactRowsMatchTheClosedFormMinimum) {
  TempDir dir;
  RegisteredSim sim = MakeBallOnGround(dir, 0.09F);
  SimConfig config = DownwardGravity();
  RunSteps<SetupList>(sim, config, 0.0F);

  RunSteps<Forward>(sim, config);

  std::size_t n = ActiveContactRows(sim);
  ASSERT_GE(n, contact_solver::kPyramidEdges);
  ASSERT_EQ(n % contact_solver::kPyramidEdges, 0U);

  double m = kBallMass;
  double a0 = -9.8;
  double r = -0.01;
  double invweight = (1.0 / m) / 3.0;
  double d = 1.0 / (2.0 * (1.0 - kImpedance) / kImpedance * 2.0 * invweight);
  double aref = -kStiffness * kImpedance * r;
  double nd = static_cast<double>(n) * d;
  double expected = ((m * a0) + (nd * aref)) / (m + nd);
  ASSERT_LT(expected - aref, 0.0);  // every row still active there

  EXPECT_NEAR(Qdd(sim, 2) / expected, 1.0, 1e-3);
}

// A limit row and a contact's rows in the same cost: the ball on its slide
// is 0.01 into the ground and 0.01 past its slide's lower limit (0.1), so
// on top of the n contact rows there's one limit row, J = side = +1, with
// its own D (R = (1 - d) / d dof_invweight0, dof_invweight0 = 1 / m) and
// the same aref -- a* = (M a0 + (n D_c + D_l) aref) / (M + n D_c + D_l).
// What's under test is that the limit half of every sum (the warmstart
// cost, the line search, J^T f) adds onto the contact half rather than
// replacing it: either half alone lands somewhere else.
TEST(Solve, ContactAndLimitRowsShareOneCost) {
  // LoadArchetypes takes the field list from the first joint, so the
  // welded ones declare every limit field too (unlimited) -- or the ball's
  // range would silently read as zero.
  std::string unlimited = Dof0LimitFields(0.0F, 0.0F);
  unlimited.replace(unlimited.find("[1]"), 3, "[0]");
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      WeldedJoint(
          "world", "", 0.0F,
          CollisionFields("[0, 0, 0]", 0.0F, 0, 0) + unlimited
      ) +
          WeldedJoint(
              "ground", "world", -0.1F,
              CollisionFields("[1, 0.1, 1]", 0.0F, 1, 2) + unlimited
          ) +
          BallJoint(
              0.09F, "ball", 0.0F, kBallMass, 2, 1,
              Dof0LimitFields(0.1F, 1.0F)
          )
  );
  SimConfig config = DownwardGravity();
  RunSteps<SetupList>(sim, config, 0.0F);

  RunSteps<Forward>(sim, config);

  std::size_t n = ActiveContactRows(sim);
  ASSERT_GE(n, contact_solver::kPyramidEdges);

  double m = kBallMass;
  double a0 = -9.8;
  double r = -0.01;
  double contact_d =
      1.0 / (2.0 * (1.0 - kImpedance) / kImpedance * 2.0 * ((1.0 / m) / 3.0));
  double limit_d = 1.0 / ((1.0 - kImpedance) / kImpedance * (1.0 / m));
  double aref = -kStiffness * kImpedance * r;
  double d_sum = (static_cast<double>(n) * contact_d) + limit_d;
  double expected = ((m * a0) + (d_sum * aref)) / (m + d_sum);
  ASSERT_LT(expected - aref, 0.0);  // every row still active there

  EXPECT_NEAR(Qdd(sim, 2) / expected, 1.0, 1e-3);
}

// A lower limit's row has side = +1 (an upper one's, above, -1): a
// pendulum 0.05 below its lower limit gets a* = (M a0 + D aref) / (M + D),
// pushed back up past where gravity alone would take it.
TEST(Solve, LowerLimitRowPushesTheJointBackInside) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      RevoluteZJointArow(
          "hinge", "", kPendulumInertia, -1.05F, 0.0F,
          Dof0LimitFields(-1.0F, 1.0F)
      )
  );
  SimConfig config = PendulumConfig();
  RunSteps<SetupList>(sim, config, 0.0F);

  RunSteps<Forward>(sim, config);

  double m = kPendulumIzz;
  double a0 = kPendulumK * std::cos(-1.05);
  double r = -0.05;
  double d = 1.0 / ((1.0 - kImpedance) / kImpedance * (1.0 / m));
  double aref = -kStiffness * kImpedance * r;
  double expected = ((m * a0) + (d * aref)) / (m + d);
  ASSERT_LT(expected - aref, 0.0);  // the row is still active there
  ASSERT_GT(expected, a0);

  EXPECT_NEAR(Qdd(sim, 0) / expected, 1.0, 1e-3);
}

// Two free balls pressed 0.01 into each other, no gravity, neither one
// welded: whichever is the pair's target and whichever its source, the
// contact pushes them apart with equal and opposite forces, so
// m_low qdd_low + m_high qdd_high = 0. The source's wrench is the pair
// pass's lane reduction into its broadcast row, the target's a plain
// store -- a sign or a dropped lane on either side breaks the balance.
TEST(Solve, ContactBetweenTwoMovingBodiesConservesMomentum) {
  constexpr F kLowMass = 2.0F;
  constexpr F kHighMass = 1.0F;
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      WeldedJoint("world", "", 0.0F, CollisionFields("[0, 0, 0]", 0.0F, 0, 0)) +
          BallJoint(0.0F, "low", 0.0F, kLowMass, 1, 2) +
          BallJoint(0.19F, "high", 0.0F, kHighMass, 2, 1)
  );
  SimConfig config;
  RunSteps<SetupList>(sim, config, 0.0F);

  RunSteps<Forward>(sim, config);

  ASSERT_GE(ActiveContactRows(sim), contact_solver::kPyramidEdges);
  F low = Qdd(sim, 1);
  F high = Qdd(sim, 2);
  EXPECT_LT(low, -1.0F);
  EXPECT_GT(high, 1.0F);
  EXPECT_NEAR((kLowMass * low) + (kHighMass * high), 0.0F, 1e-3F * high);
}

// More balls than one lane group holds, every one resting 0.01 into the
// same ground slab: the slab is the source row of pairs spread across
// several target groups, and every pass (setup, line search, forces)
// reduces over all of them. Identical, independent balls must each land
// on exactly what one ball alone does.
TEST(Solve, EveryBallAcrossSeveralLaneGroupsIsHeldTheSame) {
  constexpr std::size_t kBalls = (2 * kLanes) + 1;
  TempDir lone_dir;
  RegisteredSim lone = MakeBallOnGround(lone_dir, 0.09F);
  SimConfig config = DownwardGravity();
  RunSteps<SetupList>(lone, config, 0.0F);
  RunSteps<Forward>(lone, config);
  F expected = Qdd(lone, 2);
  ASSERT_GT(expected, -9.0F);  // premise: the ground holds the lone ball

  std::string joints =
      WeldedJoint("world", "", 0.0F, CollisionFields("[0, 0, 0]", 0.0F, 0, 0)) +
      WeldedJoint(
          "ground", "world", -0.1F,
          CollisionFields("[10, 0.1, 1]", 0.0F, 1, 2)
      );
  for (std::size_t i = 0; i < kBalls; ++i) {
    // 0.3 apart: clear of each other (and they don't collide anyway)
    float x = (static_cast<float>(i) * 0.3F) - 4.0F;
    joints += BallJoint(0.09F, "ball" + std::to_string(i), x);
  }
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(dir, joints);
  RunSteps<SetupList>(sim, config, 0.0F);

  RunSteps<Forward>(sim, config);

  for (std::size_t i = 0; i < kBalls; ++i) {
    EXPECT_NEAR(Qdd(sim, 2 + i) / expected, 1.0F, 1e-3F) << "ball " << i;
  }
}

// A ball clear of the ground has no contact rows: it just falls.
TEST(Solve, BallClearOfTheGroundFallsFreely) {
  TempDir dir;
  RegisteredSim sim = MakeBallOnGround(dir, 0.5F);
  SimConfig config = DownwardGravity();
  RunSteps<SetupList>(sim, config, 0.0F);

  RunSteps<Forward>(sim, config);

  EXPECT_EQ(ActiveContactRows(sim), 0U);
  EXPECT_NEAR(Qdd(sim, 2), -9.8F, 1e-4F);
}

// The warmstart: solving the same state twice, the second solve starts
// from the first's qacc (kQacc persists across ticks) and must land on the
// same answer -- a warmstart that leaked the previous solve's state into
// this one's cost would move it.
TEST(Solve, WarmstartedResolveLandsOnTheSameAnswer) {
  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      RevoluteZJointArow(
          "hinge", "", kPendulumInertia, 1.05F, 0.0F,
          Dof0LimitFields(-1.0F, 1.0F)
      )
  );
  SimConfig config = PendulumConfig();
  RunSteps<SetupList>(sim, config, 0.0F);

  RunSteps<Forward>(sim, config);
  F first = Qdd(sim, 0);
  RunSteps<Forward>(sim, config);

  EXPECT_NEAR(Qdd(sim, 0) / first, 1.0F, 1e-4F);
}
