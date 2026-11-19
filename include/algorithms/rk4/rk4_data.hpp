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

namespace achilles::algorithms::rk4 {

enum class RK4Field : uint8_t {
  // --- Model/kinematic inputs ---
  kJointSubspace,      // S
  kJointAcceleration,  // qdd (ABA's result at the current stage, k_a)

  // --- Mutated in place: each stage's state (k_v is the stage's qd), then
  // the committed (q_{n+1}, qd_{n+1}) ---
  kJointPosition,  // q
  kJointVelocity,  // qd

  // --- Per-tick scratch, private to this algorithm ---
  kStartPosition,    // q_n
  kStartVelocity,    // qd_n
  kVelocitySum,      // k_v1 + 2 k_v2 + 2 k_v3 + k_v4, accumulated per stage
  kAccelerationSum,  // k_a1 + 2 k_a2 + 2 k_a3 + k_a4, accumulated per stage

  kCount
};
static_assert(engine::spec::FieldEnumLike<RK4Field>);

template <RK4Field F>
struct RK4FieldTraits;

// The first four fields are SharedAs-aliased onto ABA's own blocks
// (aba_data.hpp): every stage state this algorithm writes into
// kJointPosition/kJointVelocity is exactly what ABA's next evaluation
// reads. The scratch fields are private and unnamed -- neither loaded from
// a scene nor meaningful between ticks.
template <>
struct RK4FieldTraits<RK4Field::kJointSubspace> {
  using Assembler = math::Matrix6x6Assembler<ScalarOperationT>;
  using SharedAs = JointSubspaceSlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_subspace";
};
template <>
struct RK4FieldTraits<RK4Field::kJointAcceleration> {
  using Assembler =
      domain::spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using SharedAs = JointAccelerationSlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_acceleration";
};
template <>
struct RK4FieldTraits<RK4Field::kJointPosition> {
  using Assembler = domain::spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = JointPositionSlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_position";
};
template <>
struct RK4FieldTraits<RK4Field::kJointVelocity> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using SharedAs = JointVelocitySlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_velocity";
};
template <>
struct RK4FieldTraits<RK4Field::kStartPosition> {
  using Assembler = domain::spatial::TransformAssembler<ScalarOperationT>;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
};
template <>
struct RK4FieldTraits<RK4Field::kStartVelocity> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
};
template <>
struct RK4FieldTraits<RK4Field::kVelocitySum> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
};
template <>
struct RK4FieldTraits<RK4Field::kAccelerationSum> {
  using Assembler =
      domain::spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
};
static_assert(engine::spec::FieldTraitsLike<RK4Field, RK4FieldTraits>);

using RK4View = engine::view::ViewFactory<RK4Field, RK4FieldTraits>;
static_assert(engine::spec::ViewLike<RK4View, RK4Field>);

using RK4Topology = domain::topology::JointTopology;
static_assert(domain::topology::TopologyLike<RK4Topology>);

}  // namespace achilles::algorithms::rk4
