#pragma once

#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/spatial/dual.hpp"
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

namespace achilles::algorithms::vi {

enum class VIField : uint8_t {
  // --- Model/kinematic inputs ---
  kJointAcceleration,  // qdd
  kJointVelocity,      // qd, mutates as output

  kCount
};
static_assert(engine::spec::FieldEnumLike<VIField>);

template <VIField F>
struct VIFieldTraits;

template <>
struct VIFieldTraits<VIField::kJointAcceleration> {
  // SharedAs = JointAccelerationSlot so this is ABA's own kJointAcceleration
  // block (aba_data.hpp), not a private, unconnected copy -- otherwise
  // ABA's computed qdd would never actually reach this Op.
  using Assembler =
      domain::spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using SharedAs = JointAccelerationSlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_acceleration";
};
template <>
struct VIFieldTraits<VIField::kJointVelocity> {
  // SharedAs = JointVelocitySlot so this is the same block ABA reads as
  // qd and PI both reads and integrates from.
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using SharedAs = JointVelocitySlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "joint_velocity";
};
static_assert(engine::spec::FieldTraitsLike<VIField, VIFieldTraits>);

using VIView = engine::view::ViewFactory<VIField, VIFieldTraits>;
static_assert(engine::spec::ViewLike<VIView, VIField>);

using VITopology = domain::topology::JointTopology;
static_assert(domain::topology::TopologyLike<VITopology>);

}  // namespace achilles::algorithms::vi