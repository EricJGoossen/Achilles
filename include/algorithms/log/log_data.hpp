#pragma once

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>

#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::log {

// Every field that can be logged. Read-only: each one is SharedAs the
// block another Algorithm (ABA, VI, PI) owns and writes, so LogAlgorithm
// adds no storage of its own. Adding a loggable field is adding an entry
// here (and a SharedAs tag on the owner, if it doesn't have one yet).
//
// Model constants (subspace, inertia, fixed transforms) are left out --
// they never change during a run -- as are ABA's internal scratch terms.
enum class LogField : uint8_t {
  kJointPosition,        // q
  kJointVelocity,        // qd
  kJointAcceleration,    // qdd
  kJointTorque,          // tau
  kWorldTransform,       // x_world
  kSpatialVelocity,      // v
  kSpatialAcceleration,  // a
  kCount
};
static_assert(engine::FieldEnumLike<LogField>);

inline constexpr std::size_t kNumLogFields =
    static_cast<std::size_t>(LogField::kCount);

template <LogField F>
struct LogFieldTraits;

// Same Ordering as the owning blocks (every one of them ABA's
// TopologicalOrdering) -- a shared block has one layout. kName matches the
// owner's kName (engine::FieldBlocksOf enforces that) and is what a
// LogSelection is built from.
template <>
struct LogFieldTraits<LogField::kJointPosition> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = JointPositionSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_position";
};
template <>
struct LogFieldTraits<LogField::kJointVelocity> {
  using Assembler = spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using SharedAs = JointVelocitySlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_velocity";
};
template <>
struct LogFieldTraits<LogField::kJointAcceleration> {
  using Assembler = spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using SharedAs = JointAccelerationSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_acceleration";
};
template <>
struct LogFieldTraits<LogField::kJointTorque> {
  using Assembler = spatial::SpatialForceAssembler<ScalarOperationT>;
  using SharedAs = JointTorqueSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "joint_torque";
};
template <>
struct LogFieldTraits<LogField::kWorldTransform> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = WorldTransformSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "world_transform";
};
template <>
struct LogFieldTraits<LogField::kSpatialVelocity> {
  using Assembler = spatial::SpatialVelocityAssembler<ScalarOperationT>;
  using SharedAs = SpatialVelocitySlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "spatial_velocity";
};
template <>
struct LogFieldTraits<LogField::kSpatialAcceleration> {
  using Assembler = spatial::SpatialAccelerationAssembler<ScalarOperationT>;
  using SharedAs = SpatialAccelerationSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "spatial_acceleration";
};
static_assert(engine::FieldTraitsLike<LogField, LogFieldTraits>);

using LogView = engine::view::ViewFactory<LogField, LogFieldTraits>;
static_assert(engine::view::ViewLike<LogView, LogField>);

// Which LogFields are currently being written -- changeable at any point
// during a run (see interface::Simulation::SetLogField).
using LogSelection = std::bitset<kNumLogFields>;

namespace detail {

template <std::size_t... Is>
constexpr std::array<std::string_view, kNumLogFields> LogFieldNames(
    std::index_sequence<Is...> /*indices*/
) {
  return {LogFieldTraits<static_cast<LogField>(Is)>::kName...};
}

template <std::size_t... Is>
constexpr std::array<std::size_t, kNumLogFields> LogFieldScalarCounts(
    std::index_sequence<Is...> /*indices*/
) {
  return {LogFieldTraits<static_cast<LogField>(Is)>::Assembler::kNumFields...};
}

inline constexpr auto kLogFieldNames =
    LogFieldNames(std::make_index_sequence<kNumLogFields>{});
inline constexpr auto kLogFieldScalars =
    LogFieldScalarCounts(std::make_index_sequence<kNumLogFields>{});

}  // namespace detail

constexpr std::string_view LogFieldName(LogField field) {
  return detail::kLogFieldNames.at(static_cast<std::size_t>(field));
}

// Scalars one joint's value of `field` is written as.
constexpr std::size_t LogFieldScalars(LogField field) {
  return detail::kLogFieldScalars.at(static_cast<std::size_t>(field));
}

constexpr std::optional<LogField> FindLogField(std::string_view name) {
  for (std::size_t i = 0; i < kNumLogFields; ++i) {
    if (LogFieldName(static_cast<LogField>(i)) == name) {
      return static_cast<LogField>(i);
    }
  }
  return std::nullopt;
}

}  // namespace achilles::algorithms::log
