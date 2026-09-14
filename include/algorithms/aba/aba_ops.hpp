#pragma once

#include <array>
#include <type_traits>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/conventions.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/op_invoker.hpp"

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
      : x_world_base(x_world_base), v_base(v_base) {}

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
      const Vector6& q,
      const Velocity& qd,
      const Velocity& v_parent,
      InertiaOperator<false>* I_A_out,
      Transform* x_up_out,
      Transform* x_world_out,
      Velocity* v_out,
      Acceleration* c_out,
      Force* p_out
  ) const;

  Transform x_world_base;
  Velocity v_base;
};
static_assert(
    engine::OpLike<PropagateVelocityOp> &&
    "PropagateVelocityOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<PropagateVelocityOp, ABAView> &&
    "PropagateVelocityOp's operator() must line up with kInputs/kOutputs"
);
static_assert(
    engine::OpHasInit<PropagateVelocityOp> &&
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

  static inline const InertiaOperator<false> kIABase =
      InertiaOperator<false>::Zero();
  static inline const Force kPBase = Force::Zero();
};
static_assert(
    engine::OpLike<PropagateInertiaOp> &&
    "PropagateInertiaOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<PropagateInertiaOp, ABAView> &&
    "PropagateInertiaOp's operator() must line up with kInputs/kOutputs"
);
static_assert(
    engine::OpHasInit<PropagateInertiaOp> &&
    "PropagateInertiaOp must satisfy OpHasInit concept"
);

struct PropagateAccelerationOp {
  using FieldEnum = ABAField;
  using ArgData = engine::ArgData<FieldEnum>;

  PropagateAccelerationOp(const Acceleration& a_base) : a_base(a_base) {}

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

  Acceleration a_base;
};
static_assert(
    engine::OpLike<PropagateAccelerationOp> &&
    "PropagateAccelerationOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<PropagateAccelerationOp, ABAView> &&
    "PropagateAccelerationOp's operator() must line up with kInputs/kOutputs"
);
static_assert(
    engine::OpHasInit<PropagateAccelerationOp> &&
    "PropagateAccelerationOp must satisfy OpHasInit concept"
);

}  // namespace achilles::algorithms::aba
