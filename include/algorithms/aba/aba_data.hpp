#pragma once

#include <cstddef>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::aba {

enum class ABAField : uint8_t {
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

// Every ABA field is indexed per joint and every joint's row placement
// must respect the shared JointTopology's parent/child dependencies (see
// aba_step.hpp's tree traversal and domain::JointTopology's own batch-
// safety assertion), so every field below binds
// Ordering = topology::TopologicalOrdering -- there is no ABA field that
// could safely use the (looser, no-cross-batch-dependency) default.
//
// kName is only declared on the "Model/kinematic inputs" fields -- the
// ones an archetype (domain/archetype.hpp) actually supplies from
// user data. Every other field is produced or consumed purely at runtime
// (propagated state, joint-space scratch, final outputs), so it declares
// no kName: memory::SimAllocator's Populate pass (see
// engine/memory/sim_allocator.hpp) only ever looks for a name-matched
// ArchetypeField on a field that has one, and leaves everything else to
// Carve's zero-fill and each Op's own Initialize/Apply.
template <>
struct ABAFieldTraits<ABAField::kJointSubspace> {
  using Assembler = math::Matrix6x6Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_subspace";
};
template <>
struct ABAFieldTraits<ABAField::kJointActivationMask> {
  using Assembler = math::ActivationMaskAssembler<6>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_activation_mask";
};
template <>
struct ABAFieldTraits<ABAField::kFixedJointTransform> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "fixed_joint_transform";
};
template <>
struct ABAFieldTraits<ABAField::kRigidBodyInertia> {
  using Assembler = spatial::InertiaAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "rigid_body_inertia";
};
template <>
struct ABAFieldTraits<ABAField::kJointPosition> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_position";
};
template <>
struct ABAFieldTraits<ABAField::kJointVelocity> {
  using Assembler = spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_velocity";
};
template <>
struct ABAFieldTraits<ABAField::kJointTorque> {
  using Assembler = spatial::SpatialForceAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_torque";
};
template <>
struct ABAFieldTraits<ABAField::kParentToBodyTransform> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct ABAFieldTraits<ABAField::kWorldTransform> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct ABAFieldTraits<ABAField::kSpatialVelocity> {
  using Assembler = spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct ABAFieldTraits<ABAField::kBiasAcceleration> {
  using Assembler = spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct ABAFieldTraits<ABAField::kArticulatedInertia> {
  using Assembler = spatial::InertiaOperatorAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct ABAFieldTraits<ABAField::kArticulatedBiasForce> {
  using Assembler = spatial::SpatialForceAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct ABAFieldTraits<ABAField::kUTerm> {
  using Assembler = spatial::InertiaOperatorAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct ABAFieldTraits<ABAField::kDInvTerm> {
  using Assembler = spatial::InvertedInertiaOperatorAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct ABAFieldTraits<ABAField::kJointBiasForce> {
  using Assembler = spatial::SpatialForceAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct ABAFieldTraits<ABAField::kJointAcceleration> {
  using Assembler = spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct ABAFieldTraits<ABAField::kSpatialAcceleration> {
  using Assembler = spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
static_assert(engine::FieldTraitsLike<ABAField, ABAFieldTraits>);

// ABAAlgorithm (which needs ABAStep) lives in aba_step.hpp -- ABAView and
// ABATopology don't depend on stepping at all, so they're computed directly
// here instead of via ABAAlgorithm::View, keeping this file usable (e.g. by
// aba_ops.hpp's own static_asserts) without pulling in aba_step.hpp.
using ABAView = engine::view::ViewFactory<ABAField, ABAFieldTraits>;
static_assert(engine::view::ViewLike<ABAView, ABAField>);

using ABATopology = domain::JointTopology;
static_assert(domain::TopologyLike<ABATopology>);

}  // namespace achilles::algorithms::aba