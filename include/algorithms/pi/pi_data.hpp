#pragma once

#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/matrix.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::pi {

enum class PIField : uint8_t {
  // --- Model/kinematic inputs ---
  kJointSubspace,  // S
  kJointVelocity,  // qd

  // --- Mutated in place: integrator output, next step's kinematic input ---
  kJointPosition,  // q

  kCount
};
static_assert(engine::FieldEnumLike<PIField>);

template <PIField F>
struct PIFieldTraits;

template <>
struct PIFieldTraits<PIField::kJointSubspace> {
  using Assembler = math::Matrix6x6Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_subspace";
};
template <>
struct PIFieldTraits<PIField::kJointVelocity> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_velocity";
};
template <>
struct PIFieldTraits<PIField::kJointPosition> {
  using Assembler = domain::spatial::TransformAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_position";
};
static_assert(engine::FieldTraitsLike<PIField, PIFieldTraits>);

using PIView = engine::view::ViewFactory<PIField, PIFieldTraits>;
static_assert(engine::view::ViewLike<PIView, PIField>);

using PITopology = domain::JointTopology;
static_assert(domain::TopologyLike<PITopology>);

}  // namespace achilles::algorithms::pi
