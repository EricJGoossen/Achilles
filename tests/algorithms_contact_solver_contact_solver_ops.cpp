#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <xsimd/xsimd.hpp>

#include "algorithms/contact_solver/contact_solver_data.hpp"
#include "algorithms/contact_solver/contact_solver_ops.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/narrow_phase/narrow_phase_data.hpp"
#include "util/simd_ops.hpp"

// Every contact solver Op, directly (contact_solver_ops.hpp). Expected
// values are re-derived here, in double, from the MuJoCo formulas each Op's
// own comment names -- solimp's sigmoid, mj_makeImpedance's R,
// mj_referenceConstraint's aref, mju_makeFrame's tangents, Hager-Zhang's
// beta -- rather than by calling back into the implementation. Inputs are
// uniform across lanes unless a test is about a lane's own mask.
//
// How these Ops compose into a solve (the passes, their order, the
// reductions between them) is algorithms_contact_solver_contact_solver_
// step.cpp's to check.

using namespace achilles::algorithms;
using namespace achilles::algorithms::contact_solver;
using narrow_phase::ContactActiveMask;
using narrow_phase::kSweptBoxContactSlots;

namespace {

using B = MathematicalT;
using ContactDistances = narrow_phase::ContactDistances;
using ContactPoints = narrow_phase::ContactPoints;

constexpr double kTolerance = 1e-4;

// MuJoCo's defaults, which an unset (zero) solref/solimp reads as.
constexpr double kTimeconst = 0.02;
constexpr double kDampratio = 1.0;
constexpr double kD0 = 0.9;
constexpr double kDwidth = 0.95;
constexpr double kWidth = 0.001;
constexpr double kMidpoint = 0.5;
constexpr double kPower = 2.0;

// A dt small enough that refsafe's timeconst >= 2 dt leaves 0.02 alone.
constexpr float kDt = 0.001F;

// --- MuJoCo's formulas, written out independently ---

// getimpedance: the solimp sigmoid at x = |r| / width.
double Impedance(double r) {
  double x = std::min(std::abs(r) / kWidth, 1.0);
  double y = 0.0;
  if (x <= kMidpoint) {
    double a = 1.0 / std::pow(kMidpoint, kPower - 1.0);
    y = a * std::pow(x, kPower);
  } else {
    double b = 1.0 / std::pow(1.0 - kMidpoint, kPower - 1.0);
    y = 1.0 - (b * std::pow(1.0 - x, kPower));
  }
  return kD0 + (y * (kDwidth - kD0));
}

// K = 1 / (dw^2 tc^2 dr^2), B = 2 / (dw tc)
double Stiffness(double timeconst = kTimeconst) {
  return 1.0 /
         (kDwidth * kDwidth * timeconst * timeconst * kDampratio * kDampratio);
}
double Damping(double timeconst = kTimeconst) {
  return 2.0 / (kDwidth * timeconst);
}

// D = 1 / R for a pyramidal contact row, R = 2 mu^2 (1 - d) / d A_ii,
// A_ii = (1 + mu^2) (invweight_t + invweight_s).
double ContactWeight(double imp, double mu, double invweight_sum) {
  double diag_a = (1.0 + (mu * mu)) * invweight_sum;
  return 1.0 / (2.0 * mu * mu * (1.0 - imp) / imp * diag_a);
}

// D = 1 / R for a limit row, R = (1 - d) / d dof_invweight0.
double LimitWeight(double imp, double dof_invweight) {
  return 1.0 / ((1.0 - imp) / imp * dof_invweight);
}

// --- Inputs ---

Vector3 V(float x, float y, float z) { return {B(x), B(y), B(z)}; }

// Every lane's slot `slot` active (see ContactActiveMask's bit layout).
ContactActiveMask SlotActiveInEveryLane(std::size_t slot) {
  std::uint32_t lanes = (std::uint32_t{1} << kLanes) - 1;
  return ContactActiveMask(static_cast<std::int32_t>(lanes << (slot * kLanes))
  );
}

// Slot `slot` active in lane 0 alone.
ContactActiveMask SlotActiveInLaneZero(std::size_t slot) {
  return ContactActiveMask(
      static_cast<std::int32_t>(std::uint32_t{1} << (slot * kLanes))
  );
}

// One pair's SetupRowsOp inputs: a contact along +y, slot 0 only, both
// bodies at rest at the world origin, every parameter unset (MuJoCo's
// defaults) -- each test changes what it's about.
struct PairInputs {
  Vector3 normal = V(0.0F, 1.0F, 0.0F);
  ContactDistances distance = ContactDistances::Zero();
  ContactPoints point = ContactPoints::Zero();
  ContactActiveMask active = SlotActiveInEveryLane(0);
  float margin_target = 0.0F;
  float margin_source = 0.0F;
  float friction_target = 0.0F;
  float friction_source = 0.0F;
  Solref solref = Solref::Zero();
  Solimp solimp = Solimp::Zero();
  InvWeight invweight_target{B(0.5F), B(1.0F)};
  InvWeight invweight_source{B(0.25F), B(1.0F)};
  Transform x_target = Transform::Identity();
  Transform x_source = Transform::Identity();
  Velocity v_target = Velocity::Zero();
  Velocity v_source = Velocity::Zero();
  Acceleration a_target = Acceleration::Zero();
  Acceleration a_source = Acceleration::Zero();
};

struct PairRows {
  EdgeDirections edges;
  ContactPoints point;
  ContactRows reference;
  ContactRows weight;
  ContactRows residual;
};

PairRows SetupRows(const PairInputs& in) {
  PairRows out;
  SetupRowsOp op(kDt);
  op(
      in.normal,
      in.distance,
      in.point,
      in.active,
      Scalar(B(in.margin_target)),
      Scalar(B(in.margin_source)),
      Scalar(B(in.friction_target)),
      Scalar(B(in.friction_source)),
      in.solref,
      in.solref,
      in.solimp,
      in.solimp,
      in.invweight_target,
      in.invweight_source,
      in.x_target,
      in.x_source,
      in.v_target,
      in.v_source,
      in.a_target,
      in.a_source,
      &out.edges,
      &out.point,
      &out.reference,
      &out.weight,
      &out.residual
  );
  return out;
}

Vector3 Edge(const EdgeDirections& edges, std::size_t e) {
  return Vector3(edges.Column(e));
}

// One joint's SetupLimitRowsOp inputs: DOF 0 active, the rest inactive.
struct LimitInputs {
  joint_limit::JointCoordinates distance = joint_limit::JointCoordinates::Zero();
  joint_limit::JointCoordinates side = joint_limit::JointCoordinates::Zero();
  Mat6Mask active = Mat6Mask(achilles::util::MaskStorageFor<B>(1));
  float margin = 0.0F;
  Solref solref = Solref::Zero();
  Solimp solimp = Solimp::Zero();
  joint_limit::JointCoordinates dof_invweight =
      joint_limit::JointCoordinates::Zero();
  Velocity qd = Velocity::Zero();
  Acceleration qacc_smooth = Acceleration::Zero();
};

struct LimitRows {
  joint_limit::JointCoordinates weight;
  joint_limit::JointCoordinates residual;
};

LimitRows SetupLimitRows(const LimitInputs& in) {
  LimitRows out;
  SetupLimitRowsOp op(kDt);
  op(
      in.distance,
      in.side,
      in.active,
      Scalar(B(in.margin)),
      in.solref,
      in.solimp,
      in.dof_invweight,
      in.qd,
      in.qacc_smooth,
      &out.weight,
      &out.residual
  );
  return out;
}

// A revolute-Z subspace (DOF 0 = angular z) and a body with no mass offset,
// Izz = 4 about the joint.
MotionSubspace RevoluteZ() {
  MotionSubspace s = MotionSubspace::Zero();
  s(2, 0) = B(1.0F);
  return s;
}
Inertia CenteredBody(float mass, float ixx, float iyy, float izz) {
  return {B(mass), Vector3::Zero(), B(ixx), B(iyy), B(izz), B(0.0F), B(0.0F),
          B(0.0F)};
}

// ABA's U = I_A S and the masked D^-1 = (S^T U)^-1 for a leaf body (I_A =
// I) on a one-DOF joint -- built by hand, as ABA would leave them.
struct LeafFactors {
  InertiaOperator<false> u;
  InertiaOperator<true> d_inv;
};
LeafFactors OneDofLeaf(const MotionSubspace& s, const Inertia& inertia) {
  Matrix6x6 u = inertia.AsMatrix() * s;
  Matrix6x6 d_inv = Matrix6x6::Zero();
  d_inv(0, 0) = B(1.0F) / (s.Transpose() * u)(0, 0);
  return {InertiaOperator<false>(u), InertiaOperator<true>(d_inv)};
}

}  // namespace

// ===========================================================================
// ComputeInverseInertiaOp (mj_setConst's setInertia0)
// ===========================================================================

namespace {

struct InverseInertia {
  Matrix6x6 omega;
  InvWeight invweight;
  joint_limit::JointCoordinates dof_invweight;
};

InverseInertia ComputeInverseInertia(
    const MotionSubspace& s,
    const InertiaOperator<false>& u,
    const InertiaOperator<true>& d_inv,
    const Inertia& inertia,
    const Matrix6x6& omega_parent,
    const Transform& x_up = Transform::Identity()
) {
  InverseInertia out;
  ComputeInverseInertiaOp{}(
      s,
      u,
      d_inv,
      x_up,
      inertia,
      omega_parent,
      &out.omega,
      &out.invweight,
      &out.dof_invweight
  );
  return out;
}

}  // namespace

// The base row is every root's parent: a fixed base never moves, so its
// Omega is zero.
TEST(ComputeInverseInertiaOp, BaseRowIsZero) {
  Matrix6x6 omega_base = Matrix6x6::Identity();
  ComputeInverseInertiaOp{}.Initialize(&omega_base);
  EXPECT_TRUE(achilles::util::AllTrue(omega_base.IsZero()));
}

// A hinge on a fixed base: Omega = S D^-1 S^T = 1 / Izz about z alone,
// body_invweight0's rotational mean that over three axes, its translational
// zero (the body's center sits on the axis), and dof_invweight0 = 1 / Izz.
TEST(ComputeInverseInertiaOp, HingeOnAFixedBase) {
  MotionSubspace s = RevoluteZ();
  Inertia body = CenteredBody(2.0F, 2.0F, 3.0F, 4.0F);
  LeafFactors f = OneDofLeaf(s, body);

  InverseInertia out =
      ComputeInverseInertia(s, f.u, f.d_inv, body, Matrix6x6::Zero());

  EXPECT_NEAR(out.omega(2, 2).get(0), 0.25, kTolerance);
  EXPECT_NEAR(out.omega(0, 0).get(0), 0.0, kTolerance);
  EXPECT_NEAR(out.omega(4, 4).get(0), 0.0, kTolerance);
  EXPECT_NEAR(out.invweight(0, 0).get(0), 0.0, kTolerance);
  EXPECT_NEAR(out.invweight(1, 0).get(0), 0.25 / 3.0, kTolerance);
  EXPECT_NEAR(out.dof_invweight[0].get(0), 0.25, kTolerance);
  for (std::size_t k = 1; k < 6; ++k) {
    EXPECT_NEAR(out.dof_invweight[k].get(0), 0.0, kTolerance);
  }
}

// A free body whose center of mass is off its frame's origin: whatever the
// offset, a unit force at the center of mass accelerates it by 1 / m along
// every axis, and a unit torque by I_com^-1 -- so body_invweight0 is
// (1 / m, mean of I_com^-1's diagonal), measured at the center of mass, not
// the frame origin.
TEST(ComputeInverseInertiaOp, FreeBodyInvweightIsTakenAtTheCenterOfMass) {
  MotionSubspace s = MotionSubspace::Identity();
  // m = 2, c = (0.1, 0, 0), I about the origin diag(2, 3, 4), so
  // I_com = I - m (c^2 1 - c c^T) = diag(2, 2.98, 3.98).
  Inertia body(
      B(2.0F), V(0.2F, 0.0F, 0.0F), B(2.0F), B(3.0F), B(4.0F), B(0.0F),
      B(0.0F), B(0.0F)
  );
  InertiaOperator<false> u(body.AsMatrix());
  InertiaOperator<true> d_inv(body.AsMatrix().Inverse());

  InverseInertia out =
      ComputeInverseInertia(s, u, d_inv, body, Matrix6x6::Zero());

  EXPECT_NEAR(out.invweight(0, 0).get(0), 0.5, kTolerance);
  EXPECT_NEAR(
      out.invweight(1, 0).get(0),
      (1.0 / 2.0 + 1.0 / 2.98 + 1.0 / 3.98) / 3.0,
      kTolerance
  );
}

// dof_invweight0 is M^-1's diagonal, including what the parent's motion
// adds: two coaxial hinges (the child's frame on the parent's axis), with
// M = [[I1 + I2, I2], [I2, I2]] about z, so diag(M^-1) = (1 / I1,
// 1 / I1 + 1 / I2). The parent's D is I1 alone: a hinge child takes its
// own axis's inertia with it (ABA's I_A - U D^-1 U^T).
TEST(ComputeInverseInertiaOp, DofInvweightIncludesTheParentsResponse) {
  MotionSubspace s = RevoluteZ();
  Inertia parent_body = CenteredBody(2.0F, 2.0F, 3.0F, 4.0F);
  Inertia child_body = CenteredBody(1.0F, 1.0F, 1.0F, 1.0F);

  LeafFactors parent = OneDofLeaf(s, parent_body);
  InverseInertia parent_out = ComputeInverseInertia(
      s, parent.u, parent.d_inv, parent_body, Matrix6x6::Zero()
  );
  LeafFactors child = OneDofLeaf(s, child_body);
  InverseInertia child_out = ComputeInverseInertia(
      s, child.u, child.d_inv, child_body, parent_out.omega
  );

  EXPECT_NEAR(parent_out.dof_invweight[0].get(0), 1.0 / 4.0, kTolerance);
  EXPECT_NEAR(
      child_out.dof_invweight[0].get(0), (1.0 / 4.0) + (1.0 / 1.0), kTolerance
  );
}

// ===========================================================================
// Mass-matrix operators (M x and M^-1 y, at zero velocity)
// ===========================================================================

// M x, forward then backward, for a hinge on a fixed base: a = S x, the
// body's I a, and (M x)_0 = Izz x_0.
TEST(ApplyInertiaOps, HingeOnAFixedBaseGivesIzzTimesX) {
  MotionSubspace s = RevoluteZ();
  Inertia body = CenteredBody(2.0F, 2.0F, 3.0F, 4.0F);
  Acceleration x(V(2.0F, 0.0F, 0.0F), Vector3::Zero());

  Acceleration a;
  Force f;
  ApplyInertiaForwardOp<ContactSolverField::kSearch>{}(
      s, body, Transform::Identity(), x, Acceleration::Zero(), &a, &f
  );
  EXPECT_NEAR(a.Angular().Z().get(0), 2.0, kTolerance);
  EXPECT_NEAR(f.Angular().Z().get(0), 8.0, kTolerance);

  Force m_x;
  Force f_parent = Force::Zero();
  ApplyInertiaBackwardOp{}(s, Transform::Identity(), f, &m_x, &f_parent);
  EXPECT_NEAR(m_x[0].get(0), 8.0, kTolerance);
}

// The forward half carries the parent's acceleration into the child's
// frame (x_up^-1) and adds the joint's own on top.
TEST(ApplyInertiaOps, ForwardAddsTheParentsAccelerationInTheChildFrame) {
  MotionSubspace s = RevoluteZ();
  Inertia body = CenteredBody(2.0F, 2.0F, 3.0F, 4.0F);
  Transform x_up(V(1.0F, 0.0F, 0.0F), Quaternion::Identity());
  Acceleration a_parent(V(0.0F, 0.0F, 3.0F), Vector3::Zero());
  Acceleration x(V(0.5F, 0.0F, 0.0F), Vector3::Zero());

  Acceleration a;
  Force f;
  ApplyInertiaForwardOp<ContactSolverField::kSearch>{}(
      s, body, x_up, x, a_parent, &a, &f
  );

  Acceleration expected = x_up.Inverse().Apply(a_parent) + (s * x);
  EXPECT_TRUE(achilles::util::AllTrue(a.IsApprox(expected)));
}

// The backward half adds into the parent's force -- several children
// share one parent row -- and never overwrites it (TESTING.md 9.2).
TEST(ApplyInertiaOps, BackwardAccumulatesIntoTheParentsForce) {
  Force f(V(0.0F, 0.0F, 1.0F), V(2.0F, 0.0F, 0.0F));
  Force f_parent(V(0.0F, 0.0F, 10.0F), Vector3::Zero());
  Force m_x;
  ApplyInertiaBackwardOp{}(
      RevoluteZ(), Transform::Identity(), f, &m_x, &f_parent
  );

  EXPECT_NEAR(f_parent.Angular().Z().get(0), 11.0, kTolerance);
  EXPECT_NEAR(f_parent.Linear().X().get(0), 2.0, kTolerance);
}

// M^-1 undoes M: M x through the two Apply passes, then M^-1 of that
// through the two Inverse passes, gives x back -- the same factors ABA
// leaves (U, D^-1) used both ways.
TEST(InverseInertiaOps, InvertApplyInertiaForAHingeOnAFixedBase) {
  MotionSubspace s = RevoluteZ();
  Inertia body = CenteredBody(2.0F, 2.0F, 3.0F, 4.0F);
  LeafFactors f = OneDofLeaf(s, body);
  Acceleration x(V(-1.5F, 0.0F, 0.0F), Vector3::Zero());

  Acceleration a;
  Force body_force;
  ApplyInertiaForwardOp<ContactSolverField::kSearch>{}(
      s, body, Transform::Identity(), x, Acceleration::Zero(), &a, &body_force
  );
  Force m_x;
  Force ignored = Force::Zero();
  ApplyInertiaBackwardOp{}(s, Transform::Identity(), body_force, &m_x, &ignored);

  Force u;
  Force p_parent = Force::Zero();
  InverseInertiaBackwardOp{}(
      s, Transform::Identity(), f.u, f.d_inv, m_x, Force::Zero(), &u, &p_parent
  );
  Acceleration m_inv_m_x;
  Acceleration a_out;
  InverseInertiaForwardOp{}(
      s, Transform::Identity(), f.u, f.d_inv, u, Acceleration::Zero(),
      &m_inv_m_x, &a_out
  );

  EXPECT_NEAR(m_inv_m_x[0].get(0), -1.5, kTolerance);
}

// The backward half's bias: u = grad - S^T p (what the children passed
// up), and the parent's p += x_up (p + U D^-1 u), accumulated.
TEST(InverseInertiaOps, BackwardSubtractsChildBiasAndAccumulatesIntoParent) {
  MotionSubspace s = RevoluteZ();
  LeafFactors f = OneDofLeaf(s, CenteredBody(2.0F, 2.0F, 3.0F, 4.0F));
  Force grad(V(5.0F, 0.0F, 0.0F), Vector3::Zero());
  Force p(V(0.0F, 0.0F, 2.0F), Vector3::Zero());
  Force p_parent(V(0.0F, 0.0F, 100.0F), Vector3::Zero());

  Force u;
  InverseInertiaBackwardOp{}(
      s, Transform::Identity(), f.u, f.d_inv, grad, p, &u, &p_parent
  );

  // u_0 = 5 - p_z = 3; U D^-1 u = (Izz / Izz) * 3 about z, so the parent
  // gets 100 + (2 + 3).
  EXPECT_NEAR(u[0].get(0), 3.0, kTolerance);
  EXPECT_NEAR(p_parent.Angular().Z().get(0), 105.0, kTolerance);
}

// ===========================================================================
// ConstraintForceOp (J^T f, backward)
// ===========================================================================

// A body's two world wrenches (as a target, as a source) plus what its
// children passed up, in its own frame: J^T f is S^T of that, and the
// parent's force accumulates x_up of it.
TEST(ConstraintForceOp, SumsWrenchesInTheBodyFrameAndPassesThemUp) {
  // Body frame turned a quarter turn about z: a world torque about z is the
  // same torque in the body frame, a world force along x is -y in it.
  Transform x_world(
      Vector3::Zero(), Quaternion::Exp(V(0.0F, 0.0F, 1.5707963F))
  );
  Force wrench_target(V(0.0F, 0.0F, 2.0F), V(1.0F, 0.0F, 0.0F));
  Force wrench_source(V(0.0F, 0.0F, -0.5F), Vector3::Zero());
  Force f_children(V(0.0F, 0.0F, 1.0F), Vector3::Zero());
  Force f_parent(V(0.0F, 0.0F, 10.0F), Vector3::Zero());

  Force j_t_f;
  ConstraintForceOp{}(
      RevoluteZ(), Transform::Identity(), x_world, wrench_target,
      wrench_source, f_children, &j_t_f, &f_parent
  );

  EXPECT_NEAR(j_t_f[0].get(0), 2.0 - 0.5 + 1.0, kTolerance);
  EXPECT_NEAR(f_parent.Angular().Z().get(0), 10.0 + 2.5, kTolerance);
  EXPECT_NEAR(f_parent.Linear().Y().get(0), -1.0, kTolerance);
}

// ===========================================================================
// SetupRowsOp (mj_instantiateContact, mj_makeImpedance,
// mj_referenceConstraint)
// ===========================================================================

// mju_makeFrame for n = +y: t1 from +z (n is near +y), t2 = n x t1 = +x,
// and the pyramid edges n + t1, n - t1, n + t2, n - t2 at mu = 1.
TEST(SetupRowsOp, PyramidEdgesFollowMakeFramesTangents) {
  PairInputs in;
  in.distance(0, 0) = B(-0.01F);
  PairRows out = SetupRows(in);

  EXPECT_TRUE(achilles::util::AllTrue(
      Edge(out.edges, 0).IsApprox(V(0.0F, 1.0F, 1.0F))
  ));
  EXPECT_TRUE(achilles::util::AllTrue(
      Edge(out.edges, 1).IsApprox(V(0.0F, 1.0F, -1.0F))
  ));
  EXPECT_TRUE(achilles::util::AllTrue(
      Edge(out.edges, 2).IsApprox(V(1.0F, 1.0F, 0.0F))
  ));
  EXPECT_TRUE(achilles::util::AllTrue(
      Edge(out.edges, 3).IsApprox(V(-1.0F, 1.0F, 0.0F))
  ));
}

// The pair's friction is the larger of the two bodies' (mj_contactParam),
// so it scales every tangent: mu = max(0.5, 2) = 2.
TEST(SetupRowsOp, FrictionIsTheLargerOfTheTwo) {
  PairInputs in;
  in.distance(0, 0) = B(-0.01F);
  in.friction_target = 0.5F;
  in.friction_source = 2.0F;
  PairRows out = SetupRows(in);

  EXPECT_TRUE(achilles::util::AllTrue(
      Edge(out.edges, 0).IsApprox(V(0.0F, 1.0F, 2.0F))
  ));
}

// Deeper than solimp's width the impedance is dwidth, and every one of the
// slot's four rows gets D = 1 / R, R = 2 mu^2 (1 - d) / d A_ii, and
// aref = -K d r at rest -- with the residual J qacc_smooth - aref, J read
// off the target's acceleration (all four edges have n . (0, -2, 0) = -2).
TEST(SetupRowsOp, DeepContactRowAtRest) {
  PairInputs in;
  in.distance(0, 0) = B(-0.01F);
  in.a_target = Acceleration(Vector3::Zero(), V(0.0F, -2.0F, 0.0F));
  PairRows out = SetupRows(in);

  double r = -0.01;
  double imp = Impedance(r);
  ASSERT_NEAR(imp, kDwidth, 1e-12);
  double weight = ContactWeight(imp, 1.0, 0.5 + 0.25);
  double reference = -Stiffness() * imp * r;
  for (std::size_t e = 0; e < kPyramidEdges; ++e) {
    EXPECT_NEAR(out.weight(e, 0).get(0) / weight, 1.0, kTolerance);
    EXPECT_NEAR(out.reference(e, 0).get(0) / reference, 1.0, kTolerance);
    EXPECT_NEAR(
        out.residual(e, 0).get(0) / (-2.0 - reference), 1.0, kTolerance
    );
  }
}

// aref's damping term reads the relative velocity of the two bodies at the
// contact, along each edge: target falling at 0.5 against a source at
// rest gives e . dv = -0.5 on every edge, aref = 0.5 B - K d r.
TEST(SetupRowsOp, DampingReadsTheRelativeVelocityAlongEachEdge) {
  PairInputs in;
  in.distance(0, 0) = B(-0.01F);
  in.v_target = Velocity(Vector3::Zero(), V(0.0F, -0.5F, 0.0F));
  PairRows out = SetupRows(in);

  double r = -0.01;
  double reference = (0.5 * Damping()) - (Stiffness() * Impedance(r) * r);
  EXPECT_NEAR(out.reference(0, 0).get(0) / reference, 1.0, kTolerance);
  EXPECT_NEAR(out.reference(3, 0).get(0) / reference, 1.0, kTolerance);
}

// Inside solimp's width the impedance follows its sigmoid: a x^p below the
// midpoint, 1 - b (1 - x)^p above it -- each half picked once.
TEST(SetupRowsOp, ShallowContactFollowsTheSolimpSigmoid) {
  for (double r : {-0.00025, -0.00075}) {
    PairInputs in;
    in.distance(0, 0) = B(static_cast<float>(r));
    PairRows out = SetupRows(in);

    double weight = ContactWeight(Impedance(r), 1.0, 0.75);
    EXPECT_NEAR(out.weight(0, 0).get(0) / weight, 1.0, kTolerance) << r;
  }
}

// r = distance - (m_t + m_s): a contact within the summed margin is
// active even though it doesn't yet touch, one beyond it isn't.
TEST(SetupRowsOp, MarginIsTheSumOfBothBodies) {
  PairInputs within;
  within.distance(0, 0) = B(0.004F);
  within.margin_target = 0.003F;
  within.margin_source = 0.002F;
  EXPECT_GT(SetupRows(within).weight(0, 0).get(0), 0.0F);

  PairInputs beyond = within;
  beyond.margin_target = 0.001F;
  EXPECT_FLOAT_EQ(SetupRows(beyond).weight(0, 0).get(0), 0.0F);
}

// Every row of an inactive slot -- by the narrow phase's mask, or because
// r >= 0 -- has zero weight, aref and residual, and an inactive slot a
// zero point, so a non-contact lane's leftovers never reach a sum.
TEST(SetupRowsOp, InactiveSlotsAndLanesAreZeroed) {
  PairInputs in;
  in.active = SlotActiveInLaneZero(0);
  in.distance(0, 0) = B(-0.01F);
  in.distance(1, 0) = B(-0.01F);  // slot 1: deep, but not active
  in.point(0, 0) = B(0.1F);
  in.point(0, 1) = B(0.7F);
  in.a_target = Acceleration(Vector3::Zero(), V(0.0F, -2.0F, 0.0F));
  PairRows out = SetupRows(in);

  EXPECT_GT(out.weight(0, 0).get(0), 0.0F);
  EXPECT_FLOAT_EQ(out.point(0, 0).get(0), 0.1F);
  for (std::size_t e = 0; e < kPyramidEdges; ++e) {
    EXPECT_FLOAT_EQ(out.weight(e, 1).get(0), 0.0F);
    EXPECT_FLOAT_EQ(out.reference(e, 1).get(0), 0.0F);
    EXPECT_FLOAT_EQ(out.residual(e, 1).get(0), 0.0F);
  }
  EXPECT_FLOAT_EQ(out.point(0, 1).get(0), 0.0F);
  if (kLanes > 1) {
    // lane 1: slot 0 not active there
    EXPECT_FLOAT_EQ(out.weight(0, 0).get(1), 0.0F);
    EXPECT_FLOAT_EQ(out.residual(0, 0).get(1), 0.0F);
    EXPECT_FLOAT_EQ(out.point(0, 0).get(1), 0.0F);
  }
}

// ===========================================================================
// ProjectSearchOp (J search)
// ===========================================================================

// J x is the relative acceleration of the two bodies' points at the slot,
// along each edge. A target spinning up about z at its origin moves the
// world point (1, 0, 0) along +y.
TEST(ProjectSearchOp, ReadsTheRelativePointAccelerationAlongEachEdge) {
  EdgeDirections edges = EdgeDirections::Zero();
  edges(1, 0) = B(1.0F);  // edge 0 = +y
  edges(2, 1) = B(1.0F);  // edge 1 = +z
  ContactPoints point = ContactPoints::Zero();
  point(0, 0) = B(1.0F);
  Acceleration a_target(V(0.0F, 0.0F, 3.0F), Vector3::Zero());

  ContactRows search;
  ProjectSearchOp{}(
      edges, point, Transform::Identity(), Transform::Identity(), a_target,
      Acceleration::Zero(), &search
  );
  EXPECT_NEAR(search(0, 0).get(0), 3.0, kTolerance);
  EXPECT_NEAR(search(1, 0).get(0), 0.0, kTolerance);

  // The same motion on both bodies is no relative motion at all.
  ProjectSearchOp{}(
      edges, point, Transform::Identity(), Transform::Identity(), a_target,
      a_target, &search
  );
  EXPECT_NEAR(search(0, 0).get(0), 0.0, kTolerance);
}

// Body motion is in the body's own frame, so it's carried to the world by
// x_world first: spinning about its own origin, a body sitting at (1, 0, 0)
// doesn't move the world point at that origin at all.
TEST(ProjectSearchOp, CarriesBodyMotionToTheWorldFirst) {
  EdgeDirections edges = EdgeDirections::Zero();
  edges(1, 0) = B(1.0F);
  ContactPoints point = ContactPoints::Zero();
  point(0, 0) = B(1.0F);
  Transform x_target(V(1.0F, 0.0F, 0.0F), Quaternion::Identity());

  ContactRows search;
  ProjectSearchOp{}(
      edges, point, x_target, Transform::Identity(),
      Acceleration(V(0.0F, 0.0F, 3.0F), Vector3::Zero()), Acceleration::Zero(),
      &search
  );
  EXPECT_NEAR(search(0, 0).get(0), 0.0, kTolerance);
}

// ===========================================================================
// ContactForceOp
// ===========================================================================

// Each row's residual steps to alpha, its force is -D x while x < 0 (and
// nothing otherwise), and the pair's world wrench -- sum of f e at the
// slot's point -- is added onto the target's and taken off the source's.
TEST(ContactForceOp, PushesWhileTheResidualIsNegativeAndAccumulatesWrenches) {
  ContactRows residual = ContactRows::Zero();
  residual(0, 0) = B(-1.0F);
  residual(1, 0) = B(1.0F);
  ContactRows search = ContactRows::Zero();
  search(0, 0) = B(0.4F);
  ContactRows weight = ContactRows::Zero();
  weight(0, 0) = B(2.0F);
  weight(1, 0) = B(2.0F);
  EdgeDirections edges = EdgeDirections::Zero();
  edges(1, 0) = B(1.0F);  // +y
  edges(0, 1) = B(1.0F);  // +x
  ContactPoints point = ContactPoints::Zero();
  point(0, 0) = B(1.0F);
  Force wrench_target(V(0.0F, 0.0F, 5.0F), Vector3::Zero());
  Force wrench_source(V(0.0F, 0.0F, 5.0F), Vector3::Zero());

  ContactForceOp{}(
      Scalar(B(0.5F)), search, weight, edges, point, &residual, &wrench_target,
      &wrench_source
  );

  // x = -1 + 0.5 * 0.4 = -0.8, f = 1.6 along +y at (1, 0, 0): torque
  // (1, 0, 0) x (0, 1.6, 0) = 1.6 about z. The positive row adds nothing.
  EXPECT_NEAR(residual(0, 0).get(0), -0.8, kTolerance);
  EXPECT_NEAR(residual(1, 0).get(0), 1.0, kTolerance);
  EXPECT_NEAR(wrench_target.Linear().Y().get(0), 1.6, kTolerance);
  EXPECT_NEAR(wrench_target.Linear().X().get(0), 0.0, kTolerance);
  EXPECT_NEAR(wrench_target.Angular().Z().get(0), 5.0 + 1.6, kTolerance);
  EXPECT_NEAR(wrench_source.Linear().Y().get(0), -1.6, kTolerance);
  EXPECT_NEAR(wrench_source.Angular().Z().get(0), 5.0 - 1.6, kTolerance);
}

// ===========================================================================
// Row costs (WarmstartCostOp, LineSearchRowsOp)
// ===========================================================================

// 1/2 D x^2 over rows with x < 0, at the residual (qacc_smooth) and at
// residual + search (the warmstart), added onto whatever the sums hold.
TEST(WarmstartCostOp, SumsOnlyTheNegativeRowsAtBothPoints) {
  ContactRows residual = ContactRows::Zero();
  ContactRows search = ContactRows::Zero();
  ContactRows weight = ContactRows::Zero();
  residual(0, 0) = B(-2.0F);  // negative at both
  search(0, 0) = B(1.0F);
  weight(0, 0) = B(3.0F);
  residual(2, 1) = B(1.0F);  // negative only at the warmstart
  search(2, 1) = B(-3.0F);
  weight(2, 1) = B(1.0F);
  Scalar smooth(B(10.0F));
  Scalar warm(B(20.0F));

  WarmstartCostOp{}(residual, search, weight, &smooth, &warm);

  EXPECT_NEAR(smooth.Value().get(0), 10.0 + (0.5 * 3.0 * 4.0), kTolerance);
  EXPECT_NEAR(
      warm.Value().get(0), 20.0 + (0.5 * 3.0 * 1.0) + (0.5 * 1.0 * 4.0),
      kTolerance
  );
}

// PrimalEval's slope and curvature along search at alpha: D x s and D s^2
// over rows with x = residual + alpha s < 0, added onto the sums.
TEST(LineSearchRowsOp, SumsSlopeAndCurvatureOfTheActiveRows) {
  ContactRows residual = ContactRows::Zero();
  ContactRows search = ContactRows::Zero();
  ContactRows weight = ContactRows::Zero();
  residual(0, 0) = B(-2.0F);
  search(0, 0) = B(1.0F);
  weight(0, 0) = B(3.0F);
  residual(1, 0) = B(0.5F);  // x = 0.5 + 2 * 1 > 0: inactive at alpha
  search(1, 0) = B(1.0F);
  weight(1, 0) = B(7.0F);
  Scalar slope(B(1.0F));
  Scalar curvature(B(1.0F));

  LineSearchRowsOp{}(
      Scalar(B(2.0F)), residual, search, weight, &slope, &curvature
  );

  // x = -2 + 2 = 0: not < 0, so row 0 is inactive too at alpha = 2...
  EXPECT_NEAR(slope.Value().get(0), 1.0, kTolerance);
  EXPECT_NEAR(curvature.Value().get(0), 1.0, kTolerance);

  // ...and active at alpha = 1: x = -1.
  LineSearchRowsOp{}(
      Scalar(B(1.0F)), residual, search, weight, &slope, &curvature
  );
  EXPECT_NEAR(slope.Value().get(0), 1.0 + (3.0 * -1.0 * 1.0), kTolerance);
  EXPECT_NEAR(curvature.Value().get(0), 1.0 + 3.0, kTolerance);
}

// ===========================================================================
// Joint-limit rows
// ===========================================================================

// A limit row past its end: R = (1 - d) / d dof_invweight0, aref =
// -B (side qd) - K d r, residual = side qacc_smooth - aref. The margin
// shifts r: distance 0.01 with margin 0.03 is r = -0.02.
TEST(SetupLimitRowsOp, ActiveRowFollowsMakeImpedanceAndReferenceConstraint) {
  LimitInputs in;
  in.distance[0] = B(0.01F);
  in.margin = 0.03F;
  in.side[0] = B(-1.0F);
  in.dof_invweight[0] = B(4.0F);
  in.qd = Velocity(V(0.3F, 0.0F, 0.0F), Vector3::Zero());
  in.qacc_smooth = Acceleration(V(5.0F, 0.0F, 0.0F), Vector3::Zero());
  LimitRows out = SetupLimitRows(in);

  double r = -0.02;
  double imp = Impedance(r);
  double reference = (-Damping() * (-1.0 * 0.3)) - (Stiffness() * imp * r);
  EXPECT_NEAR(out.weight[0].get(0) / LimitWeight(imp, 4.0), 1.0, kTolerance);
  EXPECT_NEAR(
      out.residual[0].get(0) / ((-1.0 * 5.0) - reference), 1.0, kTolerance
  );
}

// A joint's own solref replaces the default: timeconst 0.1 changes K and B
// (and so aref) but not R.
TEST(SetupLimitRowsOp, JointSolrefReplacesTheDefault) {
  LimitInputs in;
  in.distance[0] = B(-0.02F);
  in.side[0] = B(1.0F);
  in.dof_invweight[0] = B(1.0F);
  in.qd = Velocity(V(0.3F, 0.0F, 0.0F), Vector3::Zero());
  in.solref = Solref{B(0.1F), B(1.0F)};
  LimitRows out = SetupLimitRows(in);

  double r = -0.02;
  double imp = Impedance(r);
  double reference = (-Damping(0.1) * 0.3) - (Stiffness(0.1) * imp * r);
  EXPECT_NEAR(out.residual[0].get(0) / -reference, 1.0, kTolerance);
  EXPECT_NEAR(out.weight[0].get(0) / LimitWeight(imp, 1.0), 1.0, kTolerance);
}

// Every DOF the detector didn't mark active is zeroed, whatever its
// distance -- so a padding or unlimited DOF never reaches a sum.
TEST(SetupLimitRowsOp, InactiveDofsAreZeroed) {
  LimitInputs in;
  for (std::size_t k = 0; k < 6; ++k) {
    in.distance[k] = B(-0.5F);
    in.side[k] = B(1.0F);
    in.dof_invweight[k] = B(1.0F);
  }
  in.qacc_smooth = Acceleration(V(1.0F, 1.0F, 1.0F), V(1.0F, 1.0F, 1.0F));
  LimitRows out = SetupLimitRows(in);

  EXPECT_GT(out.weight[0].get(0), 0.0F);
  for (std::size_t k = 1; k < 6; ++k) {
    EXPECT_FLOAT_EQ(out.weight[k].get(0), 0.0F);
    EXPECT_FLOAT_EQ(out.residual[k].get(0), 0.0F);
  }
}

// J search for a limit row is side search_k, DOF by DOF.
TEST(ProjectLimitSearchOp, IsSideTimesTheDofsSearch) {
  joint_limit::JointCoordinates side = joint_limit::JointCoordinates::Zero();
  side[0] = B(-1.0F);
  side[3] = B(1.0F);
  Acceleration search(V(2.0F, 7.0F, 0.0F), V(-3.0F, 0.0F, 0.0F));

  joint_limit::JointCoordinates out;
  ProjectLimitSearchOp{}(side, search, &out);

  EXPECT_FLOAT_EQ(out[0].get(0), -2.0F);
  EXPECT_FLOAT_EQ(out[1].get(0), 0.0F);
  EXPECT_FLOAT_EQ(out[3].get(0), -3.0F);
}

// The residual steps to alpha, the row pushes (-D x) only while x < 0, and
// J^T f = side f lands on the joint's own DOF -- added onto the contacts'
// J^T f already there, never over it (it runs after ConstraintForceOp).
TEST(LimitForceOp, AddsSideTimesForceOntoTheContactsConstraintForce) {
  joint_limit::JointCoordinates residual = joint_limit::JointCoordinates::Zero();
  residual[0] = B(-1.0F);
  residual[1] = B(1.0F);
  joint_limit::JointCoordinates search = joint_limit::JointCoordinates::Zero();
  search[0] = B(0.4F);
  joint_limit::JointCoordinates weight = joint_limit::JointCoordinates::Zero();
  weight[0] = B(2.0F);
  weight[1] = B(2.0F);
  joint_limit::JointCoordinates side = joint_limit::JointCoordinates::Zero();
  side[0] = B(-1.0F);
  side[1] = B(1.0F);
  Force constraint_force(V(10.0F, 20.0F, 0.0F), Vector3::Zero());

  LimitForceOp{}(
      Scalar(B(0.5F)), search, weight, side, &residual, &constraint_force
  );

  // x = -0.8, f = 1.6, J^T f = -1.6 on DOF 0; DOF 1's x > 0 adds nothing.
  EXPECT_NEAR(residual[0].get(0), -0.8, kTolerance);
  EXPECT_NEAR(constraint_force[0].get(0), 10.0 - 1.6, kTolerance);
  EXPECT_NEAR(constraint_force[1].get(0), 20.0, kTolerance);
}

// The limit halves of the two row sums are the same sums over a joint's
// six rows, added onto what the contact halves left.
TEST(WarmstartLimitCostOp, AddsTheLimitRowsOntoTheSums) {
  joint_limit::JointCoordinates residual = joint_limit::JointCoordinates::Zero();
  joint_limit::JointCoordinates search = joint_limit::JointCoordinates::Zero();
  joint_limit::JointCoordinates weight = joint_limit::JointCoordinates::Zero();
  residual[2] = B(-2.0F);
  search[2] = B(1.0F);
  weight[2] = B(3.0F);
  Scalar smooth(B(10.0F));
  Scalar warm(B(20.0F));

  WarmstartLimitCostOp{}(residual, search, weight, &smooth, &warm);

  EXPECT_NEAR(smooth.Value().get(0), 10.0 + 6.0, kTolerance);
  EXPECT_NEAR(warm.Value().get(0), 20.0 + 1.5, kTolerance);
}

TEST(LineSearchLimitsOp, AddsTheLimitRowsOntoTheSums) {
  joint_limit::JointCoordinates residual = joint_limit::JointCoordinates::Zero();
  joint_limit::JointCoordinates search = joint_limit::JointCoordinates::Zero();
  joint_limit::JointCoordinates weight = joint_limit::JointCoordinates::Zero();
  residual[4] = B(-2.0F);
  search[4] = B(1.0F);
  weight[4] = B(3.0F);
  Scalar slope(B(1.0F));
  Scalar curvature(B(1.0F));

  LineSearchLimitsOp{}(
      Scalar(B(1.0F)), residual, search, weight, &slope, &curvature
  );

  EXPECT_NEAR(slope.Value().get(0), 1.0 - 3.0, kTolerance);
  EXPECT_NEAR(curvature.Value().get(0), 1.0 + 3.0, kTolerance);
}

// ===========================================================================
// Joint-space vector updates
// ===========================================================================

// qacc starts at qacc_smooth, with the previous tick's qacc kept as the
// warmstart direction search = qacc_warm - qacc_smooth, and Ma = 0.
TEST(BeginSolveOp, StartsAtQaccSmoothSearchingTowardTheWarmstart) {
  Acceleration qdd(V(1.0F, 2.0F, 0.0F), Vector3::Zero());
  Acceleration qacc_smooth;
  Acceleration qacc(V(4.0F, 0.0F, 0.0F), Vector3::Zero());
  Acceleration search;
  Force ma(V(9.0F, 9.0F, 9.0F), Vector3::Zero());

  BeginSolveOp{}(qdd, &qacc_smooth, &qacc, &search, &ma);

  EXPECT_TRUE(achilles::util::AllTrue(qacc_smooth.IsApprox(qdd)));
  EXPECT_TRUE(achilles::util::AllTrue(qacc.IsApprox(qdd)));
  EXPECT_TRUE(achilles::util::AllTrue(
      search.IsApprox(Acceleration(V(3.0F, -2.0F, 0.0F), Vector3::Zero()))
  ));
  EXPECT_TRUE(achilles::util::AllTrue(ma.IsZero()));
}

// qacc += alpha search and Ma += alpha M search, onto their values; every
// per-body accumulator the rest of the iteration adds into is cleared.
TEST(StepJointOp, StepsQaccAndMaAndClearsTheAccumulators) {
  Acceleration search(V(2.0F, 0.0F, 0.0F), Vector3::Zero());
  Force m_search(V(4.0F, 0.0F, 0.0F), Vector3::Zero());
  Acceleration qacc(V(1.0F, 0.0F, 0.0F), Vector3::Zero());
  Force ma(V(1.0F, 0.0F, 0.0F), Vector3::Zero());
  Force nonzero(V(1.0F, 1.0F, 1.0F), V(1.0F, 1.0F, 1.0F));
  Force wrench_target = nonzero;
  Force wrench_source = nonzero;
  Force f = nonzero;
  Force p = nonzero;

  StepJointOp{}(
      Scalar(B(0.5F)), search, m_search, &qacc, &ma, &wrench_target,
      &wrench_source, &f, &p
  );

  EXPECT_NEAR(qacc[0].get(0), 2.0, kTolerance);
  EXPECT_NEAR(ma[0].get(0), 3.0, kTolerance);
  EXPECT_TRUE(achilles::util::AllTrue(wrench_target.IsZero()));
  EXPECT_TRUE(achilles::util::AllTrue(wrench_source.IsZero()));
  EXPECT_TRUE(achilles::util::AllTrue(f.IsZero()));
  EXPECT_TRUE(achilles::util::AllTrue(p.IsZero()));
}

// grad = Ma - J^T f, keeping the previous grad and Mgrad for the
// conjugate update.
TEST(GradientOp, IsMaMinusConstraintForceAndKeepsThePrevious) {
  Force ma(V(5.0F, 0.0F, 0.0F), Vector3::Zero());
  Force constraint_force(V(2.0F, 0.0F, 0.0F), Vector3::Zero());
  Acceleration m_grad(V(7.0F, 0.0F, 0.0F), Vector3::Zero());
  Force grad(V(1.0F, 0.0F, 0.0F), Vector3::Zero());
  Force grad_prev;
  Acceleration m_grad_prev;

  GradientOp{}(ma, constraint_force, m_grad, &grad, &grad_prev, &m_grad_prev);

  EXPECT_NEAR(grad[0].get(0), 3.0, kTolerance);
  EXPECT_NEAR(grad_prev[0].get(0), 1.0, kTolerance);
  EXPECT_NEAR(m_grad_prev[0].get(0), 7.0, kTolerance);
}

// search . M search and search . Ma, added onto the sums.
TEST(GaussTermsOp, AddsBothDotProductsOntoTheSums) {
  Acceleration search(V(1.0F, 2.0F, 0.0F), Vector3::Zero());
  Force m_search(V(3.0F, 4.0F, 0.0F), Vector3::Zero());
  Force ma(V(-1.0F, 1.0F, 0.0F), Vector3::Zero());
  Scalar s_m_s(B(1.0F));
  Scalar s_ma(B(1.0F));

  GaussTermsOp{}(search, m_search, ma, &s_m_s, &s_ma);

  EXPECT_NEAR(s_m_s.Value().get(0), 1.0 + 11.0, kTolerance);
  EXPECT_NEAR(s_ma.Value().get(0), 1.0 + 1.0, kTolerance);
}

// Hager-Zhang's six dot products, with y = grad - grad_prev, My = Mgrad -
// Mgrad_prev and d the search direction, added onto the sums.
TEST(ConjugacyTermsOp, AddsTheHagerZhangDotProducts) {
  Acceleration d(V(1.0F, 2.0F, 0.0F), Vector3::Zero());
  Force grad(V(3.0F, 1.0F, 0.0F), Vector3::Zero());
  Force grad_prev(V(1.0F, 1.0F, 0.0F), Vector3::Zero());
  Acceleration m_grad(V(0.5F, 2.0F, 0.0F), Vector3::Zero());
  Acceleration m_grad_prev(V(0.5F, 1.0F, 0.0F), Vector3::Zero());
  Scalar d_y(B(0.0F));
  Scalar y_my(B(0.0F));
  Scalar y_mgrad(B(0.0F));
  Scalar d_grad(B(0.0F));
  Scalar d_d(B(0.0F));
  Scalar grad_grad(B(1.0F));

  ConjugacyTermsOp{}(
      d, grad, grad_prev, m_grad, m_grad_prev, &d_y, &y_my, &y_mgrad, &d_grad,
      &d_d, &grad_grad
  );

  // y = (2, 0), My = (0, 1)
  EXPECT_NEAR(d_y.Value().get(0), 2.0, kTolerance);
  EXPECT_NEAR(y_my.Value().get(0), 0.0, kTolerance);
  EXPECT_NEAR(y_mgrad.Value().get(0), 1.0, kTolerance);
  EXPECT_NEAR(d_grad.Value().get(0), 5.0, kTolerance);
  EXPECT_NEAR(d_d.Value().get(0), 5.0, kTolerance);
  EXPECT_NEAR(grad_grad.Value().get(0), 1.0 + 10.0, kTolerance);
}

// search = beta search - Mgrad.
TEST(UpdateSearchOp, IsBetaTimesSearchMinusMgrad) {
  Acceleration search(V(2.0F, 0.0F, 0.0F), Vector3::Zero());
  UpdateSearchOp{}(
      Acceleration(V(1.0F, 0.0F, 0.0F), Vector3::Zero()), Scalar(B(0.5F)),
      &search
  );
  EXPECT_NEAR(search[0].get(0), 0.0, kTolerance);
}

TEST(FinishSolveOp, HandsQaccBackToQdd) {
  Acceleration qacc(V(1.0F, -2.0F, 3.0F), V(0.5F, 0.0F, 0.0F));
  Acceleration qdd = Acceleration::Zero();
  FinishSolveOp{}(qacc, &qdd);
  EXPECT_TRUE(achilles::util::AllTrue(qdd.IsApprox(qacc)));
}

// ===========================================================================
// Whole-sim scalar updates
// ===========================================================================

namespace {

GlobalScalar G(float value) { return GlobalScalar(value); }

}  // namespace

// The warmstart is taken (alpha = 1) only when its total cost -- rows plus
// Gauss's 1/2 s M s + s Ma, Ma = 0 at qacc_smooth -- beats qacc_smooth's;
// either way the first direction is steepest descent.
TEST(ChooseWarmstartOp, TakesTheWarmstartOnlyWhenItCostsLess) {
  GlobalScalar alpha;
  GlobalScalar restart;

  // 1 + 0.5 * 2 + 0 = 2 < 3
  ChooseWarmstartOp{}(G(3.0F), G(1.0F), G(2.0F), G(0.0F), &alpha, &restart);
  EXPECT_FLOAT_EQ(alpha.Value(), 1.0F);
  EXPECT_FLOAT_EQ(restart.Value(), 1.0F);

  // 1 + 0.5 * 2 + 2 = 4 > 3
  ChooseWarmstartOp{}(G(3.0F), G(1.0F), G(2.0F), G(2.0F), &alpha, &restart);
  EXPECT_FLOAT_EQ(alpha.Value(), 0.0F);
  EXPECT_FLOAT_EQ(restart.Value(), 1.0F);
}

// beta_hz = (y.Mgrad - 2 (y.My / d.y) d.grad) / d.y, bounded below by
// eta_k = -1 / (|d| min(eta, |grad|)), and the restart flag cleared.
TEST(ConjugateOp, IsHagerZhangsBetaBoundedBelow) {
  GlobalScalar beta;
  GlobalScalar restart(0.0F);

  // d.y = 2, y.My = 1, y.Mgrad = 3, d.grad = 0.5: (3 - 2 * 0.5 * 0.5) / 2
  ConjugateOp{}(
      G(2.0F), G(1.0F), G(3.0F), G(0.5F), G(4.0F), G(1.0F), &beta, &restart
  );
  EXPECT_NEAR(beta.Value(), 1.25, kTolerance);
  EXPECT_FLOAT_EQ(restart.Value(), 0.0F);

  // A very negative beta_hz is clamped up to eta_k = -1 / (2 * 0.01).
  ConjugateOp{}(
      G(1.0F), G(0.0F), G(-1000.0F), G(0.0F), G(4.0F), G(1.0F), &beta,
      &restart
  );
  EXPECT_NEAR(beta.Value(), -50.0, kTolerance);
}

// beta = 0 (steepest descent) right after a restart, or when d.y is
// degenerate -- and the restart is consumed.
TEST(ConjugateOp, RestartsAndDegenerateCurvatureGiveSteepestDescent) {
  GlobalScalar beta;
  GlobalScalar restart(1.0F);
  ConjugateOp{}(
      G(2.0F), G(1.0F), G(3.0F), G(0.5F), G(4.0F), G(1.0F), &beta, &restart
  );
  EXPECT_FLOAT_EQ(beta.Value(), 0.0F);
  EXPECT_FLOAT_EQ(restart.Value(), 0.0F);

  ConjugateOp{}(
      G(0.0F), G(1.0F), G(3.0F), G(0.5F), G(4.0F), G(1.0F), &beta, &restart
  );
  EXPECT_FLOAT_EQ(beta.Value(), 0.0F);
}

TEST(BeginLineSearchOp, StartsAtZeroWithAnUnboundedBracket) {
  GlobalScalar alpha(5.0F);
  GlobalScalar low(5.0F);
  GlobalScalar high(5.0F);
  BeginLineSearchOp{}(&alpha, &low, &high);
  EXPECT_FLOAT_EQ(alpha.Value(), 0.0F);
  EXPECT_FLOAT_EQ(low.Value(), 0.0F);
  EXPECT_GT(high.Value(), 1e20F);
}

// On a quadratic the Newton step lands on the minimum in one step:
// slope = rows' + alpha sMs + sMa = 0 at alpha* = -(rows' + sMa) / (rows''
// + sMs) from alpha = 0. Here rows add slope -6 and curvature 1, sMs = 2,
// sMa = 0: alpha* = 2.
TEST(NewtonStepOp, LandsOnAQuadraticsMinimumInOneStep) {
  GlobalScalar alpha(0.0F);
  GlobalScalar low(0.0F);
  GlobalScalar high(1e30F);
  NewtonStepOp{}(G(-6.0F), G(1.0F), G(2.0F), G(0.0F), &alpha, &low, &high);

  EXPECT_NEAR(alpha.Value(), 2.0, kTolerance);
  // the slope was negative at 0, so the minimum lies above it
  EXPECT_FLOAT_EQ(low.Value(), 0.0F);
}

// A Newton step that would leave the bracket the slopes seen so far
// establish bisects it instead -- and the bracket tightens to alpha from
// whichever side its slope says.
TEST(NewtonStepOp, BisectsWhenTheNewtonStepLeavesTheBracket) {
  // At alpha = 1 the slope is +3 (rows' 1 + 1 * sMs 2 + 0), so high -> 1;
  // Newton would go to 1 - 3 / 2 = -0.5, below low = 0, so bisect [0, 1].
  GlobalScalar alpha(1.0F);
  GlobalScalar low(0.0F);
  GlobalScalar high(1e30F);
  NewtonStepOp{}(G(1.0F), G(0.0F), G(2.0F), G(0.0F), &alpha, &low, &high);

  EXPECT_FLOAT_EQ(high.Value(), 1.0F);
  EXPECT_FLOAT_EQ(low.Value(), 0.0F);
  EXPECT_FLOAT_EQ(alpha.Value(), 0.5F);
}
