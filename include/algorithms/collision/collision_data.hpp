#pragma once

#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::collision {

// The broad-phase half of collision: every body carries one box, fixed in
// its own frame (kCollisionCenter/kCollisionExtents -- static per-archetype
// data, set once at load time), and each tick that box is re-expressed as
// a world-axis-aligned bound (kWorldBoundCenter/kWorldBoundExtents) that a
// pair pass can overlap-test with nothing but per-axis compares.
//
// Center + half-extents rather than min/max corners, both in the body frame
// and in the world: the body-frame box maps onto its world bound with a
// single rotate (center) and a single |R| product (extents) -- see
// ComputeWorldBoundsOp, collision_ops.hpp -- and the pair test
// |c_a - c_b| <= h_a + h_b is the same per-axis compare either way, with a
// margin being nothing more than a += on the extents.
//
// The body-frame box is a *bound*, not the narrow-phase shape: whatever
// shape the fine check ends up testing (box, sphere, capsule, ...) only has
// to fit inside it. A body with no "collision_extents" in its archetype gets
// PlanarLayout's/Vector3's own zero-fill default (Vector3::Zero()) -- a
// point at its body origin -- the same "absent data reads as an inert
// default" convention algorithms::viz's own kVisualExtents follows
// (viz_data.hpp).
enum class CollisionField : uint8_t {
  // --- Model/kinematic inputs ---
  kWorldTransform,    // x_world -- ABA's own kWorldTransform, read-only here
  kCollisionCenter,   // c_local (bound center, in the body's own frame)
  kCollisionExtents,  // h_local (bound half-extents, along the body's axes)

  // --- Broad-phase outputs ---
  kWorldBoundCenter,   // c_world
  kWorldBoundExtents,  // h_world (half-extents, along the world axes)

  kCount
};
static_assert(engine::FieldEnumLike<CollisionField>);

template <CollisionField F>
struct CollisionFieldTraits;

// Every CollisionField binds Ordering = TopologicalOrdering, matching ABA's
// own choice for kWorldTransform (aba_data.hpp) -- for the same reason
// algorithms::viz does (see viz_data.hpp's own comment): a View addresses
// every one of its fields by the same sorted row index, so every field has
// to share the ordering policy that produced that numbering, or row i of
// kCollisionExtents and row i of kWorldTransform could silently refer to two
// different joints.
template <>
struct CollisionFieldTraits<CollisionField::kWorldTransform> {
  // SharedAs = WorldTransformSlot so this is ABA's own kWorldTransform
  // block (aba_data.hpp), not a private, unconnected copy -- nothing in
  // algorithms::collision itself ever writes it.
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = WorldTransformSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct CollisionFieldTraits<CollisionField::kCollisionCenter> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "collision_center";
};
template <>
struct CollisionFieldTraits<CollisionField::kCollisionExtents> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "collision_extents";
};
template <>
struct CollisionFieldTraits<CollisionField::kWorldBoundCenter> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct CollisionFieldTraits<CollisionField::kWorldBoundExtents> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
static_assert(engine::FieldTraitsLike<CollisionField, CollisionFieldTraits>);

using CollisionView =
    engine::view::ViewFactory<CollisionField, CollisionFieldTraits>;
static_assert(engine::view::ViewLike<CollisionView, CollisionField>);

using CollisionTopology = domain::JointTopology;
static_assert(domain::TopologyLike<CollisionTopology>);

}  // namespace achilles::algorithms::collision
