#pragma once

#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
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
  using Assembler =
      domain::spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "JointAcceleration";
};
template <>
struct VIFieldTraits<VIField::kJointVelocity> {
  using Assembler = domain::spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "JointVelocity";
};
static_assert(engine::FieldTraitsLike<VIField, VIFieldTraits>);

using VIView = engine::view::ViewFactory<VIField, VIFieldTraits>;
static_assert(engine::view::ViewLike<VIView, VIField>);

using VITopology = domain::JointTopology;
static_assert(domain::TopologyLike<VITopology>);

}  // namespace achilles::algorithms::vi