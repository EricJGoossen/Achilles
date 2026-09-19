#pragma once

#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/vector3.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::viz {

// The visual-representation half of a joint's per-tick state: where it is
// (kWorldTransform, read-only here -- ABA's own Propagate*VelocityOp is
// still the only thing that ever writes it) and how to draw it
// (kVisualExtents/kVisualColor, static per-archetype appearance data, set
// once at load time and never mutated at runtime). Like VIField/PIField,
// this has nothing to do with *simulating* the system -- see VizAlgorithm
// in viz_step.hpp, which names no Step at all (engine::NoStep) for exactly
// that reason: there is no per-tick computation of its own to run, only
// state other algorithms (ABA) or the archetype loader populate.
//
// A joint with no "visual_extents" given in its archetype gets
// PlanarLayout's/Vector3's own zero-fill default (Vector3::Zero()), which a
// renderer is expected to treat as "draw nothing for this joint" -- the
// same "absent data reads as an inert default" convention every other
// field in this codebase already follows (see e.g. Arena::ZeroFill's own
// comment), rather than a separate present/absent flag.
enum class VizField : uint8_t {
  kWorldTransform,  // x_world -- ABA's own kWorldTransform, read-only here
  kVisualExtents,   // half-extents of the box drawn at this joint, or Zero()
                    // to draw nothing
  kVisualColor,     // rgb in [0, 1]; Zero() lets a renderer pick a fallback
  kCount
};
static_assert(engine::FieldEnumLike<VizField>);

template <VizField F>
struct VizFieldTraits;

// Every VizField binds Ordering = TopologicalOrdering, matching ABA's own
// choice for kWorldTransform (aba_data.hpp) -- not just for the shared
// field, but for kVisualExtents/kVisualColor too. A View addresses every
// one of its fields by the same "sorted row" index, so unless every field
// shares the exact ordering policy that produced that row numbering, row i
// of kVisualExtents and row i of kWorldTransform could silently refer to
// two different joints (see engine/memory/binding.hpp's own comment on
// FieldOrderingT). VI's own fields make the identical choice for the
// identical reason (vi_data.hpp).
template <>
struct VizFieldTraits<VizField::kWorldTransform> {
  // SharedAs = WorldTransformSlot so this is ABA's own kWorldTransform
  // block (aba_data.hpp), not a private, unconnected copy -- otherwise
  // this field would always read stale (zero-seeded) data, since nothing
  // in algorithms::viz itself ever writes it.
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = WorldTransformSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct VizFieldTraits<VizField::kVisualExtents> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "visual_extents";
};
template <>
struct VizFieldTraits<VizField::kVisualColor> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "visual_color";
};
static_assert(engine::FieldTraitsLike<VizField, VizFieldTraits>);

using VizView = engine::view::ViewFactory<VizField, VizFieldTraits>;
static_assert(engine::view::ViewLike<VizView, VizField>);

using VizTopology = domain::JointTopology;
static_assert(domain::TopologyLike<VizTopology>);

}  // namespace achilles::algorithms::viz
