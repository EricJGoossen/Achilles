#pragma once

#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/spatial/dual.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::vi {

enum class VIField : uint8_t {
  // --- Model/kinematic inputs ---
  kJointAcceleration,  // qdd
  kJointVelocity,      // qd, mutates as output

  kCount
};
static_assert(engine::FieldEnumLike<VIField>);

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
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_acceleration";
};
template <>
struct VIFieldTraits<VIField::kJointVelocity> {
  // SharedAs = JointVelocitySlot so this is the same block ABA reads as
  // qd and PI both reads and integrates from.
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using SharedAs = JointVelocitySlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_velocity";
};
static_assert(engine::FieldTraitsLike<VIField, VIFieldTraits>);

using VIView = engine::view::ViewFactory<VIField, VIFieldTraits>;
static_assert(engine::view::ViewLike<VIView, VIField>);

using VITopology = domain::JointTopology;
static_assert(domain::TopologyLike<VITopology>);

}  // namespace achilles::algorithms::vi