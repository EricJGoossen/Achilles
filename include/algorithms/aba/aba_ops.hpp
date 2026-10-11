#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <type_traits>
#include <utility>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/conventions.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/vector6.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/pass/op_invoker.hpp"
#include "util/simd_ops.hpp"

namespace achilles::algorithms::aba {

// Each pass below that needs to seed the reserved base row a root joint's
// parent index points at owns that responsibility itself, via
// kInitInputs/kInitOutputs/Initialize (see OpHasInit,
// engine/op_contract.hpp) -- rather than a separate SeedBaseOp/Seeds pass
// that would need its own, out-of-band knowledge of exactly which fields
// every other pass reads from a parent index. PropagateVelocityOp seeds
// kWorldTransform/kSpatialVelocity (x_world_parent/v_parent for a root
// joint); PropagateInertiaOp seeds kArticulatedInertia/
// kArticulatedBiasForce to zero (the accumulator += folds each root
// joint's contribution into, see its own Initialize below);
// PropagateAccelerationOp seeds kSpatialAcceleration (a_parent -- that's
// where gravity enters, as a fictitious base acceleration). Each pass's
// own Initialize runs once, at the base row, immediately before that
// pass's own Apply -- see RunPass in engine/algorithm_step.hpp.
//
// Wired into Step for the single-root case only. Still open: JointTopology
// has no notion of "how many reserved base rows" for a traversal to size
// itself to (that's separate from topology.Size(), which counts only real
// joints) -- a forest with multiple independently-seeded roots needs that
// first.

struct PropagateVelocityOp {
  using FieldEnum = ABAField;
  using ArgData = engine::ArgData<FieldEnum>;

  PropagateVelocityOp(const Transform& x_world_base, const Velocity& v_base)
      : x_world_base_(x_world_base), v_base_(v_base) {}

  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 2> kInitOutputs = {
      ArgData{FieldEnum::kWorldTransform, true},
      ArgData{FieldEnum::kSpatialVelocity, true},
  };

  // Seeds the base row's world transform/velocity -- what a root joint
  // reads as x_world_parent/v_parent below.
  void Initialize(Transform* x_world_out, Velocity* v_out) const;

  static constexpr std::array<ArgData, 7> kInputs = {
      ArgData{FieldEnum::kJointSubspace, true},
      ArgData{FieldEnum::kRigidBodyInertia, true},
      ArgData{FieldEnum::kWorldTransform, false},
      ArgData{FieldEnum::kFixedJointTransform, true},
      ArgData{FieldEnum::kJointPosition, true},
      ArgData{FieldEnum::kJointVelocity, true},
      ArgData{FieldEnum::kSpatialVelocity, false},
  };
  static constexpr std::array<ArgData, 6> kOutputs = {
      ArgData{FieldEnum::kArticulatedInertia, true},
      ArgData{FieldEnum::kParentToBodyTransform, true},
      ArgData{FieldEnum::kWorldTransform, true},
      ArgData{FieldEnum::kSpatialVelocity, true},
      ArgData{FieldEnum::kBiasAcceleration, true},
      ArgData{FieldEnum::kArticulatedBiasForce, true},
  };

  void operator()(
      const MotionSubspace& S,
      const Inertia& I,
      const Transform& x_world_parent,
      const Transform& x_tree,
      const Transform& x_joint,
      const Velocity& qd,
      const Velocity& v_parent,
      InertiaOperator<false>* I_A_out,
      Transform* x_up_out,
      Transform* x_world_out,
      Velocity* v_out,
      Acceleration* c_out,
      Force* p_out
  ) const;

 private:
  Transform x_world_base_;
  Velocity v_base_;
};
static_assert(
    engine::OpLike<PropagateVelocityOp>,
    "PropagateVelocityOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<PropagateVelocityOp, ABAView>,
    "PropagateVelocityOp's operator() must line up with kInputs/kOutputs"
);
static_assert(
    engine::OpHasInit<PropagateVelocityOp>,
    "PropagateVelocityOp must satisfy OpHasInit concept"
);

struct PropagateInertiaOp {
  using FieldEnum = ABAField;
  using ArgData = engine::ArgData<FieldEnum>;

  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 2> kInitOutputs = {
      ArgData{FieldEnum::kArticulatedInertia, true},
      ArgData{FieldEnum::kArticulatedBiasForce, true},
  };

  // Zeroes the base row's articulated-inertia/bias-force accumulator --
  // the value every root joint's own += (in operator() below) starts
  // from. Always exactly Zero(): unlike PropagateVelocityOp/
  // PropagateAccelerationOp's Initialize, there's no caller-configurable
  // state here.
  void Initialize(InertiaOperator<false>* I_A_base_out, Force* p_base_out)
      const;

  static constexpr std::array<ArgData, 7> kInputs = {
      ArgData{FieldEnum::kJointSubspace, true},
      ArgData{FieldEnum::kJointActivationMask, true},
      ArgData{FieldEnum::kParentToBodyTransform, true},
      ArgData{FieldEnum::kArticulatedInertia, true},
      ArgData{FieldEnum::kBiasAcceleration, true},
      ArgData{FieldEnum::kJointTorque, true},
      ArgData{FieldEnum::kArticulatedBiasForce, true},
  };
  static constexpr std::array<ArgData, 5> kOutputs = {
      ArgData{FieldEnum::kArticulatedInertia, false},
      ArgData{FieldEnum::kUTerm, true},
      ArgData{FieldEnum::kDInvTerm, true},
      ArgData{FieldEnum::kArticulatedBiasForce, false},
      ArgData{FieldEnum::kJointBiasForce, true},
  };

  void operator()(
      const MotionSubspace& S,
      const Mat6Mask& mask,
      const Transform& x_up,
      const InertiaOperator<false>& I_A,
      const Acceleration& c,
      const Force& tau,
      const Force& p,
      InertiaOperator<false>* I_A_parent_out,
      InertiaOperator<false>* U_out,
      InertiaOperator<true>* D_inv_out,
      Force* p_parent_out,
      Force* u_out
  ) const;

 private:
  // The one DOF slot (column of S) any lane of this batch group has
  // active, or -1 when the group's lanes between them have none or more
  // than one -- see SingleDof.
  static int SharedSingleDof(const Mat6Mask& mask);

  // operator()'s arguments, bundled so SingleDof's per-slot dispatch below
  // stays one line per slot.
  struct Args {
    const MotionSubspace& S;
    const Mat6Mask& mask;
    const Transform& x_up;
    const InertiaOperator<false>& I_A;
    const Acceleration& c;
    const Force& tau;
    const Force& p;
    InertiaOperator<false>* I_A_parent_out;
    InertiaOperator<false>* U_out;
    InertiaOperator<true>* D_inv_out;
    Force* p_parent_out;
    Force* u_out;
  };

  // operator()'s fast path for a batch group whose every lane is either a
  // 1-DOF joint driven by slot Dof or a no-DOF row (padding, a welded
  // joint), with every other column of S zero. That is the common case --
  // every revolute/prismatic joint, and a batch group of N copies of the
  // same joint always shares its mask -- and there U = I_A S has a single
  // nonzero column and D = S^T U is 1x1, so the 6x6 products and the
  // masked 6x6 inverse of the dense path collapse to a 6-vector, a
  // reciprocal and a rank-1 update. Every output equals the dense path's
  // up to float rounding (including D^-1 = 1 on each slot the mask leaves
  // inactive, and in every lane of a no-DOF row). Returns false, having
  // written nothing, when S has a nonzero entry outside that pattern; the
  // caller then runs the dense path.
  //
  // Dof is a template argument so every S(i, Dof)/U(i, Dof) index is a
  // constant -- with a runtime slot, S, U and D^-1 all have to be
  // materialized on the stack to be indexed. always_inline: clang's
  // flatten on OpInvoker::Invoke only inlines direct callees (operator()
  // here), and this has to land in the same function for the invoker's
  // dead output preloads to go away.
  template <std::size_t Dof>
  [[gnu::always_inline]] bool SingleDof(const Args& args) const;

  static inline const InertiaOperator<false> kIABase =
      InertiaOperator<false>::Zero();
  static inline const Force kPBase = Force::Zero();
};
static_assert(
    engine::OpLike<PropagateInertiaOp>,
    "PropagateInertiaOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<PropagateInertiaOp, ABAView>,
    "PropagateInertiaOp's operator() must line up with kInputs/kOutputs"
);
static_assert(
    engine::OpHasInit<PropagateInertiaOp>,
    "PropagateInertiaOp must satisfy OpHasInit concept"
);

struct PropagateAccelerationOp {
  using FieldEnum = ABAField;
  using ArgData = engine::ArgData<FieldEnum>;

  explicit PropagateAccelerationOp(const Acceleration& a_base)
      : a_base_(a_base) {}

  static constexpr std::array<ArgData, 0> kInitInputs = {};
  static constexpr std::array<ArgData, 1> kInitOutputs = {
      ArgData{FieldEnum::kSpatialAcceleration, true},
  };

  // Seeds the base row's spatial acceleration -- what a root joint reads
  // as a_parent below; this is how gravity (or any other fictitious base
  // acceleration) enters the algorithm.
  void Initialize(Acceleration* a_base_out) const;

  static constexpr std::array<ArgData, 7> kInputs = {
      ArgData{FieldEnum::kJointSubspace, true},
      ArgData{FieldEnum::kDInvTerm, true},
      ArgData{FieldEnum::kUTerm, true},
      ArgData{FieldEnum::kParentToBodyTransform, true},
      ArgData{FieldEnum::kBiasAcceleration, true},
      ArgData{FieldEnum::kSpatialAcceleration, false},
      ArgData{FieldEnum::kJointBiasForce, true},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kJointAcceleration, true},
      ArgData{FieldEnum::kSpatialAcceleration, true},
  };

  void operator()(
      const Matrix6x6& S,
      const InertiaOperator<true>& D_inv,
      const InertiaOperator<false>& U,
      const Transform& x_up,
      const Acceleration& c,
      const Acceleration& a_parent,
      const Force& u,
      Acceleration* qdd_out,
      Acceleration* a_out
  ) const;

 private:
  Acceleration a_base_;
};
static_assert(
    engine::OpLike<PropagateAccelerationOp>,
    "PropagateAccelerationOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<PropagateAccelerationOp, ABAView>,
    "PropagateAccelerationOp's operator() must line up with kInputs/kOutputs"
);
static_assert(
    engine::OpHasInit<PropagateAccelerationOp>,
    "PropagateAccelerationOp must satisfy OpHasInit concept"
);

// The Op bodies live here, inline, rather than in a separate .cpp: each is
// called once per batch group from inside OpInvoker's gather/call/scatter
// (engine/pass/op_invoker.hpp), and only once the body is visible at that
// call site can the compiler see that e.g. U_out is overwritten without
// being read -- dropping OpInvoker's preload of it -- and keep the 6x6
// temporaries out of a call boundary's memory round trip.

inline void PropagateVelocityOp::Initialize(
    Transform* x_world_out, Velocity* v_out
) const {
  *x_world_out = x_world_base_;
  *v_out = v_base_;
}

inline void PropagateVelocityOp::operator()(
    const MotionSubspace& S,
    const Inertia& I,
    const Transform& x_world_parent,
    const Transform& x_tree,
    const Transform& x_joint,
    const Velocity& qd,
    const Velocity& v_parent,
    InertiaOperator<false>* I_A_out,
    Transform* x_up_out,
    Transform* x_world_out,
    Velocity* v_out,
    Acceleration* c_out,
    Force* p_out
) const {
  Transform x_up = x_tree * x_joint;

  Velocity qd_spatial = S * qd;
  Velocity v = x_up.Inverse().Apply(v_parent) + qd_spatial;

  *I_A_out = I.AsArticulated();
  *x_up_out = x_up;
  *x_world_out = x_world_parent * x_up;
  *v_out = v;

  *c_out = v.Cross(qd_spatial);
  *p_out = v.CrossForce(I.Apply(v));
}

inline void PropagateInertiaOp::Initialize(
    InertiaOperator<false>* I_A_base_out, Force* p_base_out
) const {
  *I_A_base_out = kIABase;
  *p_base_out = kPBase;
}

inline void PropagateInertiaOp::operator()(
    const MotionSubspace& S,
    const Mat6Mask& mask,
    const Transform& x_up,
    const InertiaOperator<false>& I_A,
    const Acceleration& c,
    const Force& tau,
    const Force& p,
    InertiaOperator<false>* I_A_parent_out,
    InertiaOperator<false>* U_out,
    InertiaOperator<true>* D_inv_out,
    Force* p_parent_out,
    Force* u_out
) const {
  const Args args{
      S,
      mask,
      x_up,
      I_A,
      c,
      tau,
      p,
      I_A_parent_out,
      U_out,
      D_inv_out,
      p_parent_out,
      u_out
  };
  bool handled = false;
  switch (SharedSingleDof(mask)) {
    case 0:
      handled = SingleDof<0>(args);
      break;
    case 1:
      handled = SingleDof<1>(args);
      break;
    case 2:
      handled = SingleDof<2>(args);
      break;
    case 3:
      handled = SingleDof<3>(args);
      break;
    case 4:
      handled = SingleDof<4>(args);
      break;
    case 5:
      handled = SingleDof<5>(args);
      break;
    default:
      break;
  }
  if (handled) {
    return;
  }

  // Dense path: any mix of joint types, any number of DOFs.

  // U = I_A S
  InertiaOperator<false> U(I_A.AsMatrix() * S);

  // D = S^T U
  InertiaOperator<false> D = S.Transpose() * U;

  // u = tau - S^T p
  Force u = tau - S.Transpose() * p;

  // D^-1
  InertiaOperator<true> D_inv = D.MaskedInverse(mask);

  // U D^-1
  Matrix6x6 U_D_inv = U.AsMatrix() * D_inv.AsMatrix();

  // I_A' = I_A - U D^-1 U^T (this joint's contribution, before
  // transforming into the parent's frame)
  InertiaOperator<false> I_A_contribution = I_A - U_D_inv * U.Transpose();

  // p' = p + I_A' c + U D^-1 u (ditto)
  Force p_contribution = p + I_A_contribution.Apply(c) + U_D_inv * u;

  *U_out = U;
  *D_inv_out = D_inv;
  *u_out = u;

  // I_A' is symmetric (as I_A is, and D^-1 is), so only its upper
  // triangle needs transforming -- see Transform::ApplySymmetric.
  *I_A_parent_out +=
      InertiaOperator<false>(x_up.ApplySymmetric(I_A_contribution.AsMatrix()));
  *p_parent_out += x_up.Apply(p_contribution);
}

inline int PropagateInertiaOp::SharedSingleDof(const Mat6Mask& mask) {
  using MaskBatch = std::remove_cvref_t<decltype(mask.AsStorage())>;
  using Lane = typename MaskBatch::value_type;
  alignas(MaskBatch::arch_type::alignment()) std::array<Lane, MaskBatch::size>
      lanes{};
  mask.AsStorage().store_aligned(lanes.data());
  Lane active = 0;
  for (Lane lane : lanes) {
    active |= lane;
  }
  // Exactly one bit set across the whole group: every lane is either that
  // one slot or no slot at all.
  if (active == 0 || (active & (active - 1)) != 0) {
    return -1;
  }
  return std::countr_zero(static_cast<std::make_unsigned_t<Lane>>(active));
}

template <std::size_t Dof>
bool PropagateInertiaOp::SingleDof(const Args& args) const {
  constexpr std::size_t dof = Dof;
  const MotionSubspace& S = args.S;
  const Mat6Mask& mask = args.mask;
  const Transform& x_up = args.x_up;
  const InertiaOperator<false>& I_A = args.I_A;
  const Acceleration& c = args.c;
  const Force& tau = args.tau;
  const Force& p = args.p;
  using T = MathematicalT;
  constexpr std::size_t kN = 6;
  const T zero(0.0F);
  const T one(1.0F);

  // Lanes that have slot dof active; the rest are no-DOF rows.
  const auto live = util::Select(mask[dof], one, zero) != zero;

  // Only column dof of S may be nonzero, and only in a live lane. Checked
  // on the bit patterns: OR-ing together every entry that has to be zero
  // leaves no bit set only if all of them are. (A -0.0 entry counts as
  // nonzero here, which only sends the group down the dense path.)
  using Bits = xsimd::batch<std::int32_t>;
  auto bits = [](const T& x) { return xsimd::bitwise_cast<std::int32_t>(x); };
  Bits off_column(0);
  Bits on_column(0);
  for (std::size_t i = 0; i < kN; ++i) {
    for (std::size_t j = 0; j < kN; ++j) {
      if (j == dof) {
        on_column |= bits(S(i, j));
      } else {
        off_column |= bits(S(i, j));
      }
    }
  }
  const Bits stray = off_column | xsimd::select(mask[dof], Bits(0), on_column);
  if (xsimd::any(stray != Bits(0))) {
    return false;
  }

  const Matrix6x6& i_a = I_A.AsMatrix();
  std::array<T, kN> s;
  for (std::size_t i = 0; i < kN; ++i) {
    s[i] = S(i, dof);
  }

  // U = I_A S: column dof is I_A s, every other column is zero.
  std::array<T, kN> u_col;
  for (std::size_t i = 0; i < kN; ++i) {
    T sum = i_a(i, 0) * s[0];
    for (std::size_t k = 1; k < kN; ++k) {
      sum += i_a(i, k) * s[k];
    }
    u_col[i] = sum;
  }

  // D = S^T U = s . (I_A s). A no-DOF lane has s = 0, so D = 0 there; the
  // dense path's masked inverse substitutes 1 for it, as does this.
  T d = s[0] * u_col[0];
  for (std::size_t i = 1; i < kN; ++i) {
    d += s[i] * u_col[i];
  }
  const T d_inv = util::Select(live, one / d, one);

  // u = tau - S^T p: only entry dof of S^T p is nonzero.
  T s_dot_p = s[0] * p[0];
  for (std::size_t i = 1; i < kN; ++i) {
    s_dot_p += s[i] * p[i];
  }
  std::array<T, kN> u_vals;
  for (std::size_t i = 0; i < kN; ++i) {
    u_vals[i] = tau[i];
  }
  u_vals[dof] = u_vals[dof] - s_dot_p;
  const Force u(math::Vector6<T>(
      u_vals[0], u_vals[1], u_vals[2], u_vals[3], u_vals[4], u_vals[5]
  ));

  // U D^-1: column dof of U, scaled.
  std::array<T, kN> u_d_inv;
  for (std::size_t i = 0; i < kN; ++i) {
    u_d_inv[i] = u_col[i] * d_inv;
  }

  // I_A' = I_A - U D^-1 U^T: a symmetric rank-1 update, computed on the
  // upper triangle and mirrored.
  Matrix6x6 contribution;
  for (std::size_t i = 0; i < kN; ++i) {
    for (std::size_t j = i; j < kN; ++j) {
      const T value = i_a(i, j) - u_d_inv[i] * u_col[j];
      contribution(i, j) = value;
      contribution(j, i) = value;
    }
  }
  const InertiaOperator<false> I_A_contribution(contribution);

  // p' = p + I_A' c + U D^-1 u
  const Force p_contribution = p + I_A_contribution.Apply(c) +
                               Force(math::Vector6<T>(
                                   u_d_inv[0] * u_vals[dof],
                                   u_d_inv[1] * u_vals[dof],
                                   u_d_inv[2] * u_vals[dof],
                                   u_d_inv[3] * u_vals[dof],
                                   u_d_inv[4] * u_vals[dof],
                                   u_d_inv[5] * u_vals[dof]
                               ));

  Matrix6x6 U = Matrix6x6::Zero();
  Matrix6x6 D_inv = Matrix6x6::Identity();
  for (std::size_t i = 0; i < kN; ++i) {
    U(i, dof) = u_col[i];
  }
  D_inv(dof, dof) = d_inv;

  *args.U_out = InertiaOperator<false>(U);
  *args.D_inv_out = InertiaOperator<true>(D_inv);
  *args.u_out = u;

  *args.I_A_parent_out +=
      InertiaOperator<false>(x_up.ApplySymmetric(contribution));
  *args.p_parent_out += x_up.Apply(p_contribution);
  return true;
}

inline void PropagateAccelerationOp::Initialize(Acceleration* a_base_out
) const {
  *a_base_out = a_base_;
}

inline void PropagateAccelerationOp::operator()(
    const Matrix6x6& S,
    const InertiaOperator<true>& D_inv,
    const InertiaOperator<false>& U,
    const Transform& x_up,
    const Acceleration& c,
    const Acceleration& a_parent,
    const Force& u,
    Acceleration* qdd_out,
    Acceleration* a_out
) const {
  Acceleration a_pre = x_up.Inverse().Apply(a_parent) + c;

  Acceleration qdd = D_inv.Apply(u - U.Transpose().Apply(a_pre));

  Acceleration a = a_pre + S * qdd;

  *qdd_out = qdd;
  *a_out = a;
}

}  // namespace achilles::algorithms::aba
