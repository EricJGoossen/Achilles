#pragma once

#include <array>

#include "algorithms/contact_solver/contact_solver_data.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/narrow_phase/narrow_phase_data.hpp"
#include "engine/op_contract.hpp"

namespace achilles::algorithms::contact_solver {

// What every Op below shares: its field enum, and the ArgData/ArgIndex
// names its kInputs/kOutputs are spelled in.
struct SolverOp {
  using FieldEnum = ContactSolverField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;
};

// A whole-sim scalar as a GlobalTraversal Op reads/writes it: one plain
// value (see topology::GlobalIndexing). A batched Op reads the same field
// as a Scalar, splatted, and adds into it as a lane reduction.
using GlobalScalar = math::Scalar<ScalarOperationT>;

// ---------------------------------------------------------------------------
// Once per sim
// ---------------------------------------------------------------------------

// Omega_i, body i's acceleration per unit wrench applied at it (body frame),
// by the inverse articulated-inertia recursion over ABA's own factors: a
// wrench f at body i reaches its parent as X^T K f, K = 1 - U D^-1 S^T (the
// part its own joint doesn't absorb), and the parent's response comes back
// down through the same joint, so
//
//   Omega_i = S D^-1 S^T + K^T X^T Omega_parent X K,  Omega_base = 0,
//
// X being x_up's force transform (child -> parent). Then MuJoCo's
// body_invweight0 (setInertia0): the mean diagonal of J_com M^-1 J_com^T,
// the translational block at the body's center of mass and the rotational
// block, from Omega_i directly instead of one M^-1 solve per body.
struct ComputeInverseInertiaOp : SolverOp {
  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 1> kInitOutputs = {
      ArgData{FieldEnum::kInverseInertia, ArgIndex::kTarget},
  };
  void Initialize(Matrix6x6* omega_base_out) const;

  static constexpr std::array<ArgData, 6> kInputs = {
      ArgData{FieldEnum::kJointSubspace, ArgIndex::kTarget},
      ArgData{FieldEnum::kUTerm, ArgIndex::kTarget},
      ArgData{FieldEnum::kDInvTerm, ArgIndex::kTarget},
      ArgData{FieldEnum::kParentToBodyTransform, ArgIndex::kTarget},
      ArgData{FieldEnum::kRigidBodyInertia, ArgIndex::kTarget},
      ArgData{FieldEnum::kInverseInertia, ArgIndex::kParent},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kInverseInertia, ArgIndex::kTarget},
      ArgData{FieldEnum::kInvWeight, ArgIndex::kTarget},
  };
  void operator()(
      const MotionSubspace& S,
      const InertiaOperator<false>& U,
      const InertiaOperator<true>& D_inv,
      const Transform& x_up,
      const Inertia& I,
      const Matrix6x6& omega_parent,
      Matrix6x6* omega_out,
      InvWeight* invweight_out
  ) const;
};

// ---------------------------------------------------------------------------
// Mass-matrix operators (zero velocity, no gravity: the base doesn't move)
// ---------------------------------------------------------------------------

// M x, forward half (RNEA): body accelerations a = x_up^-1 a_parent + S x of
// the joint-space vector in field Joint, and each body's own I a, which
// ApplyInertiaBackwardOp then accumulates down the tree. The accelerations
// are also exactly the body motion J x reads (ProjectSearchOp).
template <ContactSolverField Joint>
struct ApplyInertiaForwardOp : SolverOp {
  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 1> kInitOutputs = {
      ArgData{FieldEnum::kBodyAcceleration, ArgIndex::kTarget},
  };
  void Initialize(Acceleration* a_base_out) const {
    *a_base_out = Acceleration::Zero();
  }

  static constexpr std::array<ArgData, 5> kInputs = {
      ArgData{FieldEnum::kJointSubspace, ArgIndex::kTarget},
      ArgData{FieldEnum::kRigidBodyInertia, ArgIndex::kTarget},
      ArgData{FieldEnum::kParentToBodyTransform, ArgIndex::kTarget},
      ArgData{Joint, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyAcceleration, ArgIndex::kParent},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kBodyAcceleration, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyForce, ArgIndex::kTarget},
  };
  void operator()(
      const MotionSubspace& S,
      const Inertia& I,
      const Transform& x_up,
      const Acceleration& x,
      const Acceleration& a_parent,
      Acceleration* a_out,
      Force* f_out
  ) const {
    Acceleration a = x_up.Inverse().Apply(a_parent) + S * x;
    *a_out = a;
    *f_out = I.AsArticulated().Apply(a);
  }
};

// M x, backward half: (M x)_i = S^T f_i, f_i being body i's own I a plus
// everything its children passed up, which it passes on to its parent in
// turn.
struct ApplyInertiaBackwardOp : SolverOp {
  // The base row is every root's parent: zeroed, so what the roots pass
  // up into it stays bounded (nothing reads it).
  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 1> kInitOutputs = {
      ArgData{FieldEnum::kBodyForce, ArgIndex::kTarget},
  };
  void Initialize(Force* base_out) const { *base_out = Force::Zero(); }

  static constexpr std::array<ArgData, 3> kInputs = {
      ArgData{FieldEnum::kJointSubspace, ArgIndex::kTarget},
      ArgData{FieldEnum::kParentToBodyTransform, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyForce, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kMSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyForce, ArgIndex::kParent},
  };
  void operator()(
      const MotionSubspace& S,
      const Transform& x_up,
      const Force& f,
      Force* m_search_out,
      Force* f_parent_out
  ) const {
    *m_search_out = S.Transpose() * f;
    *f_parent_out += x_up.Apply(f);
  }
};

// M^-1 grad, backward half (ABA's PropagateInertiaOp at zero velocity):
// u = grad - S^T p, and the parent's bias p += x_up (p + U D^-1 u), p
// starting from zero (StepJointOp clears it).
struct InverseInertiaBackwardOp : SolverOp {
  // The base row is every root's parent: zeroed, so what the roots pass
  // up into it stays bounded (nothing reads it).
  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 1> kInitOutputs = {
      ArgData{FieldEnum::kInverseBiasForce, ArgIndex::kTarget},
  };
  void Initialize(Force* base_out) const { *base_out = Force::Zero(); }

  static constexpr std::array<ArgData, 6> kInputs = {
      ArgData{FieldEnum::kJointSubspace, ArgIndex::kTarget},
      ArgData{FieldEnum::kParentToBodyTransform, ArgIndex::kTarget},
      ArgData{FieldEnum::kUTerm, ArgIndex::kTarget},
      ArgData{FieldEnum::kDInvTerm, ArgIndex::kTarget},
      ArgData{FieldEnum::kGrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kInverseBiasForce, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kInverseJointForce, ArgIndex::kTarget},
      ArgData{FieldEnum::kInverseBiasForce, ArgIndex::kParent},
  };
  void operator()(
      const MotionSubspace& S,
      const Transform& x_up,
      const InertiaOperator<false>& U,
      const InertiaOperator<true>& D_inv,
      const Force& grad,
      const Force& p,
      Force* u_out,
      Force* p_parent_out
  ) const {
    Force u = grad - S.Transpose() * p;
    *u_out = u;
    *p_parent_out += x_up.Apply(p + (U.AsMatrix() * D_inv.AsMatrix()) * u);
  }
};

// M^-1 grad, forward half (ABA's PropagateAccelerationOp at zero
// velocity): Mgrad = D^-1 (u - U^T a_pre), a = a_pre + S Mgrad.
struct InverseInertiaForwardOp : SolverOp {
  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 1> kInitOutputs = {
      ArgData{FieldEnum::kBodyAcceleration, ArgIndex::kTarget},
  };
  void Initialize(Acceleration* a_base_out) const {
    *a_base_out = Acceleration::Zero();
  }

  static constexpr std::array<ArgData, 6> kInputs = {
      ArgData{FieldEnum::kJointSubspace, ArgIndex::kTarget},
      ArgData{FieldEnum::kParentToBodyTransform, ArgIndex::kTarget},
      ArgData{FieldEnum::kUTerm, ArgIndex::kTarget},
      ArgData{FieldEnum::kDInvTerm, ArgIndex::kTarget},
      ArgData{FieldEnum::kInverseJointForce, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyAcceleration, ArgIndex::kParent},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kMgrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyAcceleration, ArgIndex::kTarget},
  };
  void operator()(
      const MotionSubspace& S,
      const Transform& x_up,
      const InertiaOperator<false>& U,
      const InertiaOperator<true>& D_inv,
      const Force& u,
      const Acceleration& a_parent,
      Acceleration* m_grad_out,
      Acceleration* a_out
  ) const {
    Acceleration a_pre = x_up.Inverse().Apply(a_parent);
    Acceleration m_grad = D_inv.Apply(u - U.Transpose().Apply(a_pre));
    *m_grad_out = m_grad;
    *a_out = a_pre + S * m_grad;
  }
};

// J^T f, backward: each body's own contact wrench (both sides of every
// pair it's in, summed in world by ContactForceOp) moved into its own
// frame, plus whatever its children passed up; (J^T f)_i = S^T of that,
// passed on to the parent. Body forces start from zero (StepJointOp).
struct ConstraintForceOp : SolverOp {
  // The base row is every root's parent: zeroed, so what the roots pass
  // up into it stays bounded (nothing reads it).
  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 1> kInitOutputs = {
      ArgData{FieldEnum::kBodyForce, ArgIndex::kTarget},
  };
  void Initialize(Force* base_out) const { *base_out = Force::Zero(); }

  static constexpr std::array<ArgData, 6> kInputs = {
      ArgData{FieldEnum::kJointSubspace, ArgIndex::kTarget},
      ArgData{FieldEnum::kParentToBodyTransform, ArgIndex::kTarget},
      ArgData{FieldEnum::kWorldTransform, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyWrenchTarget, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyWrenchSource, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyForce, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kConstraintForce, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyForce, ArgIndex::kParent},
  };
  void operator()(
      const MotionSubspace& S,
      const Transform& x_up,
      const Transform& x_world,
      const Force& wrench_target,
      const Force& wrench_source,
      const Force& f_children,
      Force* constraint_force_out,
      Force* f_parent_out
  ) const {
    Force f =
        f_children + x_world.Inverse().Apply(wrench_target + wrench_source);
    *constraint_force_out = S.Transpose() * f;
    *f_parent_out += x_up.Apply(f);
  }
};

// ---------------------------------------------------------------------------
// Contact rows
// ---------------------------------------------------------------------------

// Everything per contact row that holds for the whole solve
// (mj_instantiateContact, mj_makeImpedance, mj_referenceConstraint):
//
//   - the pair's friction mu = max(mu_s, mu_t) and its solref/solimp, each
//     the mean of the two bodies' (MuJoCo's solmix 1:1 mix), unset values
//     read as MuJoCo's defaults;
//   - the pyramid edges n +- mu t1, n +- mu t2, along mju_makeFrame's
//     tangents;
//   - per slot, r = distance - margin, the impedance d(r) (the solimp
//     sigmoid), and per row aref = -B (J v) - K d r with
//     K = 1 / (dw^2 tc^2 dr^2), B = 2 / (dw tc), tc = max(timeconst, 2 dt);
//   - per row R = 2 mu^2 (1 - d) / d * A_ii -- mj_makeImpedance's normal-row
//     R scaled onto every pyramid edge (impratio 1), A_ii mj_diagApprox's
//     pyramidal (1 + mu^2)(invweight_t + invweight_s) -- and D = 1 / R;
//   - the residual J qacc_smooth - aref the solve starts from, J read off
//     the body accelerations ApplyInertiaForwardOp<kQaccSmooth> just left.
//
// A row is active while its slot is (kContactActive) and r < 0; every
// inactive row gets D = 0 and a zero aref/residual, and an inactive slot a
// zero point, so nothing a non-contact lane computed (possibly from a
// degenerate shape) ever reaches a sum.
struct SetupRowsOp : SolverOp {
  explicit SetupRowsOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 20> kInputs = {
      ArgData{FieldEnum::kContactNormal, ArgIndex::kTarget},
      ArgData{FieldEnum::kContactDistance, ArgIndex::kTarget},
      ArgData{FieldEnum::kContactPoint, ArgIndex::kTarget},
      ArgData{FieldEnum::kContactActive, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionMargin, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionMargin, ArgIndex::kSource},
      ArgData{FieldEnum::kContactFriction, ArgIndex::kTarget},
      ArgData{FieldEnum::kContactFriction, ArgIndex::kSource},
      ArgData{FieldEnum::kContactSolref, ArgIndex::kTarget},
      ArgData{FieldEnum::kContactSolref, ArgIndex::kSource},
      ArgData{FieldEnum::kContactSolimp, ArgIndex::kTarget},
      ArgData{FieldEnum::kContactSolimp, ArgIndex::kSource},
      ArgData{FieldEnum::kInvWeight, ArgIndex::kTarget},
      ArgData{FieldEnum::kInvWeight, ArgIndex::kSource},
      ArgData{FieldEnum::kWorldTransform, ArgIndex::kTarget},
      ArgData{FieldEnum::kWorldTransform, ArgIndex::kSource},
      ArgData{FieldEnum::kSpatialVelocity, ArgIndex::kTarget},
      ArgData{FieldEnum::kSpatialVelocity, ArgIndex::kSource},
      ArgData{FieldEnum::kBodyAcceleration, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyAcceleration, ArgIndex::kSource},
  };
  static constexpr std::array<ArgData, 5> kOutputs = {
      ArgData{FieldEnum::kEdgeDirections, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowPoint, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowReference, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowWeight, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowResidual, ArgIndex::kTarget},
  };
  void operator()(
      const Vector3& normal,
      const math::Matrix<MathematicalT, narrow_phase::kSweptBoxContactSlots, 1>&
          distance,
      const math::Matrix<MathematicalT, 3, narrow_phase::kSweptBoxContactSlots>&
          point,
      const narrow_phase::ContactActiveMask& active,
      const Scalar& margin_target,
      const Scalar& margin_source,
      const Scalar& friction_target,
      const Scalar& friction_source,
      const Solref& solref_target,
      const Solref& solref_source,
      const Solimp& solimp_target,
      const Solimp& solimp_source,
      const InvWeight& invweight_target,
      const InvWeight& invweight_source,
      const Transform& x_target,
      const Transform& x_source,
      const Velocity& v_target,
      const Velocity& v_source,
      const Acceleration& a_target,
      const Acceleration& a_source,
      EdgeDirections* edges_out,
      math::Matrix<MathematicalT, 3, narrow_phase::kSweptBoxContactSlots>*
          point_out,
      ContactRows* reference_out,
      ContactRows* weight_out,
      ContactRows* residual_out
  ) const;

 private:
  ScalarOperationT dt_;
};

// J search per row: the relative acceleration of the two bodies' points at
// each slot, along each edge, read off the body accelerations
// ApplyInertiaForwardOp<kSearch> just left.
struct ProjectSearchOp : SolverOp {
  static constexpr std::array<ArgData, 6> kInputs = {
      ArgData{FieldEnum::kEdgeDirections, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowPoint, ArgIndex::kTarget},
      ArgData{FieldEnum::kWorldTransform, ArgIndex::kTarget},
      ArgData{FieldEnum::kWorldTransform, ArgIndex::kSource},
      ArgData{FieldEnum::kBodyAcceleration, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyAcceleration, ArgIndex::kSource},
  };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{FieldEnum::kRowSearch, ArgIndex::kTarget},
  };
  void operator()(
      const EdgeDirections& edges,
      const math::Matrix<MathematicalT, 3, narrow_phase::kSweptBoxContactSlots>&
          point,
      const Transform& x_target,
      const Transform& x_source,
      const Acceleration& a_target,
      const Acceleration& a_source,
      ContactRows* search_out
  ) const;
};

// Steps the residual to this iteration's alpha (J qacc - aref, after
// StepJointOp's qacc += alpha search), then each row's force
// f = -D (J qacc - aref) while that's negative (mj_constraintUpdate's
// QUADRATIC state), and the pair's resulting world wrench: + on the target,
// - on the source, each a sum of f e at its slot's point. The target's adds
// up lane for lane; the source's is a reduction into its one row.
struct ContactForceOp : SolverOp {
  static constexpr std::array<ArgData, 5> kInputs = {
      ArgData{FieldEnum::kStep, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowWeight, ArgIndex::kTarget},
      ArgData{FieldEnum::kEdgeDirections, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowPoint, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 3> kOutputs = {
      ArgData{FieldEnum::kRowResidual, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyWrenchTarget, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyWrenchSource, ArgIndex::kSource},
  };
  void operator()(
      const Scalar& alpha,
      const ContactRows& search,
      const ContactRows& weight,
      const EdgeDirections& edges,
      const math::Matrix<MathematicalT, 3, narrow_phase::kSweptBoxContactSlots>&
          point,
      ContactRows* residual_out,
      Force* wrench_target_out,
      Force* wrench_source_out
  ) const;
};

// The rows' cost at qacc_smooth (the residual as set up) and at the
// warmstart (the residual plus J (qacc_warm - qacc_smooth)), summed into
// two whole-sim scalars for ChooseWarmstartOp.
struct WarmstartCostOp : SolverOp {
  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 2> kInitOutputs = {
      ArgData{FieldEnum::kRowCostSmooth, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowCostWarm, ArgIndex::kTarget},
  };
  void Initialize(GlobalScalar* smooth_out, GlobalScalar* warm_out) const {
    *smooth_out = GlobalScalar(0.0F);
    *warm_out = GlobalScalar(0.0F);
  }

  static constexpr std::array<ArgData, 3> kInputs = {
      ArgData{FieldEnum::kRowResidual, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowWeight, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kRowCostSmooth, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowCostWarm, ArgIndex::kTarget},
  };
  void operator()(
      const ContactRows& residual,
      const ContactRows& search,
      const ContactRows& weight,
      Scalar* smooth_out,
      Scalar* warm_out
  ) const;
};

// The rows' slope and curvature of the cost along the search direction at
// the current alpha, PrimalEval's: sum over rows with
// x = residual + alpha (J search) < 0 of D x (J search) and
// D (J search)^2.
struct LineSearchRowsOp : SolverOp {
  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 2> kInitOutputs = {
      ArgData{FieldEnum::kRowSlope, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowCurvature, ArgIndex::kTarget},
  };
  void Initialize(GlobalScalar* slope_out, GlobalScalar* curvature_out) const {
    *slope_out = GlobalScalar(0.0F);
    *curvature_out = GlobalScalar(0.0F);
  }

  static constexpr std::array<ArgData, 4> kInputs = {
      ArgData{FieldEnum::kStep, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowResidual, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowWeight, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kRowSlope, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowCurvature, ArgIndex::kTarget},
  };
  void operator()(
      const Scalar& alpha,
      const ContactRows& residual,
      const ContactRows& search,
      const ContactRows& weight,
      Scalar* slope_out,
      Scalar* curvature_out
  ) const;
};

// ---------------------------------------------------------------------------
// Joint-space vector updates
// ---------------------------------------------------------------------------

// Starts the solve at qacc_smooth (ABA's qdd), with the previous tick's
// constrained qacc kept as the warmstart direction: search = qacc_warm -
// qacc_smooth, qacc = qacc_smooth, M (qacc - qacc_smooth) = 0.
struct BeginSolveOp : SolverOp {
  static constexpr std::array<ArgData, 1> kInputs = {
      ArgData{FieldEnum::kJointAcceleration, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 4> kOutputs = {
      ArgData{FieldEnum::kQaccSmooth, ArgIndex::kTarget},
      ArgData{FieldEnum::kQacc, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kMa, ArgIndex::kTarget},
  };
  void operator()(
      const Acceleration& qdd,
      Acceleration* qacc_smooth_out,
      Acceleration* qacc_out,
      Acceleration* search_out,
      Force* ma_out
  ) const {
    *qacc_smooth_out = qdd;
    *search_out = *qacc_out - qdd;
    *qacc_out = qdd;
    *ma_out = Force::Zero();
  }
};

// qacc += alpha search, M (qacc - qacc_smooth) += alpha M search -- and
// clears the body accumulators the rest of the iteration adds into.
struct StepJointOp : SolverOp {
  static constexpr std::array<ArgData, 3> kInputs = {
      ArgData{FieldEnum::kStep, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kMSearch, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 6> kOutputs = {
      ArgData{FieldEnum::kQacc, ArgIndex::kTarget},
      ArgData{FieldEnum::kMa, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyWrenchTarget, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyWrenchSource, ArgIndex::kTarget},
      ArgData{FieldEnum::kBodyForce, ArgIndex::kTarget},
      ArgData{FieldEnum::kInverseBiasForce, ArgIndex::kTarget},
  };
  void operator()(
      const Scalar& alpha,
      const Acceleration& search,
      const Force& m_search,
      Acceleration* qacc_out,
      Force* ma_out,
      Force* wrench_target_out,
      Force* wrench_source_out,
      Force* f_out,
      Force* p_out
  ) const {
    *qacc_out += search * alpha.Value();
    *ma_out += m_search * alpha.Value();
    *wrench_target_out = Force::Zero();
    *wrench_source_out = Force::Zero();
    *f_out = Force::Zero();
    *p_out = Force::Zero();
  }
};

// grad = M (qacc - qacc_smooth) - J^T f (mj_solPrimal's grad, Ma -
// qfrc_smooth - qfrc_constraint), keeping the previous grad and Mgrad for
// the conjugate update.
struct GradientOp : SolverOp {
  static constexpr std::array<ArgData, 3> kInputs = {
      ArgData{FieldEnum::kMa, ArgIndex::kTarget},
      ArgData{FieldEnum::kConstraintForce, ArgIndex::kTarget},
      ArgData{FieldEnum::kMgrad, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 3> kOutputs = {
      ArgData{FieldEnum::kGrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kGradPrev, ArgIndex::kTarget},
      ArgData{FieldEnum::kMgradPrev, ArgIndex::kTarget},
  };
  void operator()(
      const Force& ma,
      const Force& constraint_force,
      const Acceleration& m_grad,
      Force* grad_out,
      Force* grad_prev_out,
      Acceleration* m_grad_prev_out
  ) const {
    *grad_prev_out = *grad_out;
    *grad_out = ma - constraint_force;
    *m_grad_prev_out = m_grad;
  }
};

// search . M search and search . Ma, the Gauss part of the line search's
// quadratic (PrimalSearch's quadGauss).
struct GaussTermsOp : SolverOp {
  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 2> kInitOutputs = {
      ArgData{FieldEnum::kSearchMSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchMa, ArgIndex::kTarget},
  };
  void Initialize(GlobalScalar* s_m_s_out, GlobalScalar* s_ma_out) const {
    *s_m_s_out = GlobalScalar(0.0F);
    *s_ma_out = GlobalScalar(0.0F);
  }

  static constexpr std::array<ArgData, 3> kInputs = {
      ArgData{FieldEnum::kSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kMSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kMa, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kSearchMSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchMa, ArgIndex::kTarget},
  };
  void operator()(
      const Acceleration& search,
      const Force& m_search,
      const Force& ma,
      Scalar* s_m_s_out,
      Scalar* s_ma_out
  ) const {
    *s_m_s_out += Scalar(search.AsVector6().Dot(m_search.AsVector6()));
    *s_ma_out += Scalar(search.AsVector6().Dot(ma.AsVector6()));
  }
};

// The dot products Hager-Zhang's beta is built from (mj_solPrimal's CG
// update), d being the search direction about to be replaced.
struct ConjugacyTermsOp : SolverOp {
  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 6> kInitOutputs = {
      ArgData{FieldEnum::kSearchDotDiff, ArgIndex::kTarget},
      ArgData{FieldEnum::kDiffDotMDiff, ArgIndex::kTarget},
      ArgData{FieldEnum::kDiffDotMgrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchDotGrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchNormSq, ArgIndex::kTarget},
      ArgData{FieldEnum::kGradNormSq, ArgIndex::kTarget},
  };
  void Initialize(
      GlobalScalar* d_y_out,
      GlobalScalar* y_my_out,
      GlobalScalar* y_mgrad_out,
      GlobalScalar* d_grad_out,
      GlobalScalar* d_d_out,
      GlobalScalar* grad_grad_out
  ) const {
    *d_y_out = GlobalScalar(0.0F);
    *y_my_out = GlobalScalar(0.0F);
    *y_mgrad_out = GlobalScalar(0.0F);
    *d_grad_out = GlobalScalar(0.0F);
    *d_d_out = GlobalScalar(0.0F);
    *grad_grad_out = GlobalScalar(0.0F);
  }

  static constexpr std::array<ArgData, 5> kInputs = {
      ArgData{FieldEnum::kSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kGrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kGradPrev, ArgIndex::kTarget},
      ArgData{FieldEnum::kMgrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kMgradPrev, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 6> kOutputs = {
      ArgData{FieldEnum::kSearchDotDiff, ArgIndex::kTarget},
      ArgData{FieldEnum::kDiffDotMDiff, ArgIndex::kTarget},
      ArgData{FieldEnum::kDiffDotMgrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchDotGrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchNormSq, ArgIndex::kTarget},
      ArgData{FieldEnum::kGradNormSq, ArgIndex::kTarget},
  };
  void operator()(
      const Acceleration& search,
      const Force& grad,
      const Force& grad_prev,
      const Acceleration& m_grad,
      const Acceleration& m_grad_prev,
      Scalar* d_y_out,
      Scalar* y_my_out,
      Scalar* y_mgrad_out,
      Scalar* d_grad_out,
      Scalar* d_d_out,
      Scalar* grad_grad_out
  ) const;
};

// search = -Mgrad + beta search.
struct UpdateSearchOp : SolverOp {
  static constexpr std::array<ArgData, 2> kInputs = {
      ArgData{FieldEnum::kMgrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kBeta, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{FieldEnum::kSearch, ArgIndex::kTarget},
  };
  void operator()(
      const Acceleration& m_grad, const Scalar& beta, Acceleration* search_out
  ) const {
    *search_out = *search_out * beta.Value() - m_grad;
  }
};

// The constrained qacc, handed back to ABA's qdd for VI to integrate.
struct FinishSolveOp : SolverOp {
  static constexpr std::array<ArgData, 1> kInputs = {
      ArgData{FieldEnum::kQacc, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{FieldEnum::kJointAcceleration, ArgIndex::kTarget},
  };
  void operator()(const Acceleration& qacc, Acceleration* qdd_out) const {
    *qdd_out = qacc;
  }
};

// ---------------------------------------------------------------------------
// Whole-sim scalar updates (GlobalTraversal)
// ---------------------------------------------------------------------------

// MuJoCo's warmstart(): start from whichever of qacc_smooth and the
// previous tick's qacc costs less -- alpha = 1 along search = qacc_warm -
// qacc_smooth takes the warmstart, alpha = 0 keeps qacc_smooth -- and make
// the first direction steepest descent.
struct ChooseWarmstartOp : SolverOp {
  static constexpr std::array<ArgData, 4> kInputs = {
      ArgData{FieldEnum::kRowCostSmooth, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowCostWarm, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchMSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchMa, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kStep, ArgIndex::kTarget},
      ArgData{FieldEnum::kRestart, ArgIndex::kTarget},
  };
  void operator()(
      const GlobalScalar& cost_smooth,
      const GlobalScalar& cost_warm,
      const GlobalScalar& s_m_s,
      const GlobalScalar& s_ma,
      GlobalScalar* alpha_out,
      GlobalScalar* restart_out
  ) const;
};

// Hager-Zhang's beta, exactly as mj_solPrimal's CG update computes it
// (MuJoCo's default; PRP only under mjCG_PRP), or 0 on a restart.
struct ConjugateOp : SolverOp {
  static constexpr std::array<ArgData, 6> kInputs = {
      ArgData{FieldEnum::kSearchDotDiff, ArgIndex::kTarget},
      ArgData{FieldEnum::kDiffDotMDiff, ArgIndex::kTarget},
      ArgData{FieldEnum::kDiffDotMgrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchDotGrad, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchNormSq, ArgIndex::kTarget},
      ArgData{FieldEnum::kGradNormSq, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kBeta, ArgIndex::kTarget},
      ArgData{FieldEnum::kRestart, ArgIndex::kTarget},
  };
  void operator()(
      const GlobalScalar& d_y,
      const GlobalScalar& y_my,
      const GlobalScalar& y_mgrad,
      const GlobalScalar& d_grad,
      const GlobalScalar& d_d,
      const GlobalScalar& grad_grad,
      GlobalScalar* beta_out,
      GlobalScalar* restart_out
  ) const;
};

// alpha = 0 with an unbounded bracket, before the line search's Newton
// steps.
struct BeginLineSearchOp : SolverOp {
  static constexpr std::array<ArgData, 0> kInputs = {};
  static constexpr std::array<ArgData, 3> kOutputs = {
      ArgData{FieldEnum::kStep, ArgIndex::kTarget},
      ArgData{FieldEnum::kStepLow, ArgIndex::kTarget},
      ArgData{FieldEnum::kStepHigh, ArgIndex::kTarget},
  };
  void operator()(
      GlobalScalar* alpha_out, GlobalScalar* low_out, GlobalScalar* high_out
  ) const;
};

// One safeguarded 1-D Newton step on the cost along the search direction:
// slope = rows' + alpha sMs + sMa, curvature = rows' + sMs. Alpha moves to
// the minimum of the local quadratic, unless that leaves the bracket the
// slopes seen so far establish (the cost is convex, so its slope only
// grows with alpha), in which case it bisects the bracket.
struct NewtonStepOp : SolverOp {
  static constexpr std::array<ArgData, 4> kInputs = {
      ArgData{FieldEnum::kRowSlope, ArgIndex::kTarget},
      ArgData{FieldEnum::kRowCurvature, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchMSearch, ArgIndex::kTarget},
      ArgData{FieldEnum::kSearchMa, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 3> kOutputs = {
      ArgData{FieldEnum::kStep, ArgIndex::kTarget},
      ArgData{FieldEnum::kStepLow, ArgIndex::kTarget},
      ArgData{FieldEnum::kStepHigh, ArgIndex::kTarget},
  };
  void operator()(
      const GlobalScalar& row_slope,
      const GlobalScalar& row_curvature,
      const GlobalScalar& s_m_s,
      const GlobalScalar& s_ma,
      GlobalScalar* alpha_out,
      GlobalScalar* low_out,
      GlobalScalar* high_out
  ) const;
};

}  // namespace achilles::algorithms::contact_solver
