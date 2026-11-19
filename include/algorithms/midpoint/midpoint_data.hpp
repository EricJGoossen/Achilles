#pragma once

#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/math/matrix.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/transform.hpp"
#include "domain/topology/joint_topology.hpp"
#include "domain/topology/topology_contract.hpp"
#include "engine/spec/field_contract.hpp"
#include "engine/spec/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "policy/layout/base_layout.hpp"
#include "policy/layout/planar_layout.hpp"
#include "policy/layout/segmented_layout.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "policy/ordering/linear_ordering.hpp"
#include "policy/ordering/list_ordering.hpp"
#include "policy/ordering/pair_ordering.hpp"
#include "policy/ordering/topological_ordering.hpp"

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
static_assert(engine::spec::FieldEnumLike<MidpointField>);

template <MidpointField F>
struct MidpointFieldTraits;

// The first four fields are SharedAs-aliased onto ABA's own blocks
// (aba_data.hpp): every trial this algorithm writes into kJointPosition/
// kJointVelocity is exactly what ABA's next evaluation reads. The scratch
// fields are private and unnamed -- neither loaded from a scene nor
// meaningful between ticks.
template <>
struct MidpointFieldTraits<MidpointField::kJointSubspace> {
  using Assembler = math::Matrix6x6Assembler<ScalarOperationT>;
  using SharedAs = JointSubspaceSlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_subspace";
};
template <>
struct MidpointFieldTraits<MidpointField::kJointAcceleration> {
  using Assembler =
      domain::spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using SharedAs = JointAccelerationSlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_acceleration";
};
template <>
struct MidpointFieldTraits<MidpointField::kJointPosition> {
  using Assembler = domain::spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = JointPositionSlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_position";
};
template <>
struct MidpointFieldTraits<MidpointField::kJointVelocity> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using SharedAs = JointVelocitySlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_velocity";
};
template <>
struct MidpointFieldTraits<MidpointField::kStartPosition> {
  using Assembler = domain::spatial::TransformAssembler<ScalarOperationT>;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
};
template <>
struct MidpointFieldTraits<MidpointField::kStartVelocity> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
};
template <>
struct MidpointFieldTraits<MidpointField::kNextVelocity> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
};
static_assert(engine::spec::
                  FieldTraitsLike<MidpointField, MidpointFieldTraits>);

using MidpointView =
    engine::view::ViewFactory<MidpointField, MidpointFieldTraits>;
static_assert(engine::spec::ViewLike<MidpointView, MidpointField>);

using MidpointTopology = domain::topology::JointTopology;
static_assert(domain::topology::TopologyLike<MidpointTopology>);

}  // namespace achilles::algorithms::midpoint
