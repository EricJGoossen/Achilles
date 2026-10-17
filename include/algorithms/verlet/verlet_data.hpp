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

namespace achilles::algorithms::verlet {

// Velocity Verlet needs no state of its own between its kicks and drift --
// every field here is SharedAs-aliased onto ABA's own block (aba_data.hpp),
// so its kicks read the qdd ABA just solved and its drift moves the q ABA
// reads next.
enum class VerletField : uint8_t {
  // --- Model/kinematic inputs ---
  kJointSubspace,      // S
  kJointAcceleration,  // qdd

  // --- Mutated in place: integrator output, next ABA evaluation's input ---
  kJointVelocity,  // qd
  kJointPosition,  // q

  kCount
};
static_assert(engine::FieldEnumLike<VerletField>);

template <VerletField F>
struct VerletFieldTraits;

template <>
struct VerletFieldTraits<VerletField::kJointSubspace> {
  using Assembler = math::Matrix6x6Assembler<ScalarOperationT>;
  using SharedAs = JointSubspaceSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_subspace";
};
template <>
struct VerletFieldTraits<VerletField::kJointAcceleration> {
  using Assembler =
      domain::spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using SharedAs = JointAccelerationSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_acceleration";
};
template <>
struct VerletFieldTraits<VerletField::kJointVelocity> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using SharedAs = JointVelocitySlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_velocity";
};
template <>
struct VerletFieldTraits<VerletField::kJointPosition> {
  using Assembler = domain::spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = JointPositionSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_position";
};
static_assert(engine::FieldTraitsLike<VerletField, VerletFieldTraits>);

using VerletView = engine::view::ViewFactory<VerletField, VerletFieldTraits>;
static_assert(engine::view::ViewLike<VerletView, VerletField>);

using VerletTopology = domain::JointTopology;
static_assert(domain::TopologyLike<VerletTopology>);

}  // namespace achilles::algorithms::verlet
