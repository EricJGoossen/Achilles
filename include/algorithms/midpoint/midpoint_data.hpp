#pragma once

#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/matrix.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::midpoint {

enum class MidpointField : uint8_t {
  // --- Model/kinematic inputs ---
  kJointSubspace,      // S
  kJointAcceleration,  // qdd (ABA's result at the current midpoint trial)

  // --- Mutated in place: the midpoint trial ABA is evaluated at, then the
  // committed (q_{n+1}, qd_{n+1}) ---
  kJointPosition,  // q
  kJointVelocity,  // qd

  // --- Per-tick scratch, private to this algorithm ---
  kStartPosition,  // q_n
  kStartVelocity,  // qd_n
  kNextVelocity,   // qd_{n+1}, the current fixed-point estimate

  kCount
};
static_assert(engine::FieldEnumLike<MidpointField>);

template <MidpointField F>
struct MidpointFieldTraits;

// The first four fields are SharedAs-aliased onto ABA's own blocks
// (aba_data.hpp): every trial this algorithm writes into kJointPosition/
// kJointVelocity is exactly what ABA's next evaluation reads. The scratch
// fields are private and unnamed -- neither loaded from an archetype nor
// meaningful between ticks.
template <>
struct MidpointFieldTraits<MidpointField::kJointSubspace> {
  using Assembler = math::Matrix6x6Assembler<ScalarOperationT>;
  using SharedAs = JointSubspaceSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_subspace";
};
template <>
struct MidpointFieldTraits<MidpointField::kJointAcceleration> {
  using Assembler =
      domain::spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using SharedAs = JointAccelerationSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_acceleration";
};
template <>
struct MidpointFieldTraits<MidpointField::kJointPosition> {
  using Assembler = domain::spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = JointPositionSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_position";
};
template <>
struct MidpointFieldTraits<MidpointField::kJointVelocity> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using SharedAs = JointVelocitySlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_velocity";
};
template <>
struct MidpointFieldTraits<MidpointField::kStartPosition> {
  using Assembler = domain::spatial::TransformAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct MidpointFieldTraits<MidpointField::kStartVelocity> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct MidpointFieldTraits<MidpointField::kNextVelocity> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
static_assert(engine::FieldTraitsLike<MidpointField, MidpointFieldTraits>);

using MidpointView =
    engine::view::ViewFactory<MidpointField, MidpointFieldTraits>;
static_assert(engine::view::ViewLike<MidpointView, MidpointField>);

using MidpointTopology = domain::JointTopology;
static_assert(domain::TopologyLike<MidpointTopology>);

}  // namespace achilles::algorithms::midpoint
