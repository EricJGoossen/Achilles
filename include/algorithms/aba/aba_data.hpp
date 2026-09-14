#pragma once

#include "algorithms/conventions.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/vector6.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"
#include "domain/topology/joint_topology.hpp"
#include "domain/topology/topology_contract.hpp"
#include "engine/field_contract.hpp"
#include "engine/view/planar_view.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::aba {

enum class ABAField {
  // --- Model/kinematic inputs ---
  kJointSubspace,        // S
  kJointActivationMask,  // mask
  kFixedJointTransform,  // x_tree (joint's placement relative to parent body)
  kRigidBodyInertia,     // I
  kJointPosition,        // q
  kJointVelocity,        // qd
  kJointTorque,          // tau

  // --- Per-body state, propagated outward then inward  ---
  kParentToBodyTransform,  // x_up (Op1 out; Op2/Op3 in)
  kWorldTransform,         // x_world (out) / x_world_parent (in)
  kSpatialVelocity,        // v (out) / v_parent (in)
  kBiasAcceleration,       // c (Op1 out; Op2/Op3 in)
  kArticulatedInertia,     // I_A (Op1 out, Op2 in) / I_A_parent (Op2 out)
  kArticulatedBiasForce,   // p (Op1 out, Op2 in) / p_parent (Op2 out)

  // --- Joint-space scratch, produced in Op2, consumed in Op3 only ---
  kUTerm,           // U (U = I_A S)
  kDInvTerm,        // D_inv (D = S^T U)
  kJointBiasForce,  // u (u = tau - S^T p)

  // --- Final outputs (Op3) ---
  kJointAcceleration,    // qdd
  kSpatialAcceleration,  // a (out) / a_parent (in, next body)

  kCount
};
static_assert(engine::FieldEnumLike<ABAField>);

template <ABAField F>
struct ABAFieldTraits;

template <>
struct ABAFieldTraits<ABAField::kJointSubspace> {
  using Assembler = math::Matrix6x6Assembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kJointActivationMask> {
  using Assembler = math::ActivationMaskAssembler<6>;
};
template <>
struct ABAFieldTraits<ABAField::kFixedJointTransform> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kRigidBodyInertia> {
  using Assembler = spatial::InertiaAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kJointPosition> {
  using Assembler = math::Vector6Assembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kJointVelocity> {
  using Assembler = spatial::SpatialVelocityAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kJointTorque> {
  using Assembler = spatial::SpatialForceAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kParentToBodyTransform> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kWorldTransform> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kSpatialVelocity> {
  using Assembler = spatial::SpatialVelocityAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kBiasAcceleration> {
  using Assembler = spatial::SpatialAccelerationAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kArticulatedInertia> {
  using Assembler = spatial::InertiaOperatorAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kArticulatedBiasForce> {
  using Assembler = spatial::SpatialForceAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kUTerm> {
  using Assembler = spatial::InertiaOperatorAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kDInvTerm> {
  using Assembler = spatial::InvertedInertiaOperatorAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kJointBiasForce> {
  using Assembler = spatial::SpatialForceAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kJointAcceleration> {
  using Assembler = spatial::SpatialAccelerationAssembler<ScalarOperationT>;
};
template <>
struct ABAFieldTraits<ABAField::kSpatialAcceleration> {
  using Assembler = spatial::SpatialAccelerationAssembler<ScalarOperationT>;
};
static_assert(engine::FieldTraitsLike<ABAField, ABAFieldTraits>);

using ABAView = engine::view::
    ViewFactory<engine::view::PlanarView, ABAField, ABAFieldTraits>;
static_assert(engine::view::ViewLike<ABAView, ABAField>);

using ABATopology = domain::topology::JointTopology;
static_assert(domain::topology::TopologyLike<ABATopology>);

}  // namespace achilles::algorithms::aba