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

namespace achilles::algorithms::pi {

enum class PIField : uint8_t {
  // --- Model/kinematic inputs ---
  kJointSubspace,  // S
  kJointVelocity,  // qd

  // --- Mutated in place: integrator output, next step's kinematic input ---
  kJointPosition,  // q

  kCount
};
static_assert(engine::spec::FieldEnumLike<PIField>);

template <PIField F>
struct PIFieldTraits;

template <>
struct PIFieldTraits<PIField::kJointSubspace> {
  // SharedAs = JointSubspaceSlot so this is ABA's own kJointSubspace
  // block (aba_data.hpp), not a private, unconnected copy.
  using Assembler = math::Matrix6x6Assembler<ScalarOperationT>;
  using SharedAs = JointSubspaceSlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_subspace";
};
template <>
struct PIFieldTraits<PIField::kJointVelocity> {
  // SharedAs = JointVelocitySlot so this is the same block VI integrates
  // qdd into and ABA reads back as qd.
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using SharedAs = JointVelocitySlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_velocity";
};
template <>
struct PIFieldTraits<PIField::kJointPosition> {
  // SharedAs = JointPositionSlot so this Op is mutating ABA's own
  // kJointPosition block, not a private, unconnected copy.
  using Assembler = domain::spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = JointPositionSlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_position";
};
static_assert(engine::spec::FieldTraitsLike<PIField, PIFieldTraits>);

using PIView = engine::view::ViewFactory<PIField, PIFieldTraits>;
static_assert(engine::spec::ViewLike<PIView, PIField>);

using PITopology = domain::topology::JointTopology;
static_assert(domain::topology::TopologyLike<PITopology>);

}  // namespace achilles::algorithms::pi
