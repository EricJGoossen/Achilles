#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "util/simd_ops.hpp"

namespace achilles::algorithms::collision {

// How many distinct collision groups a body can belong to/collide with --
// the bit width of kCollisionGroup/kCollisionMask. 16 rather than the full
// 32 an int32 lane holds so that "every group" is a small positive literal
// (65535) in archetype data, which is read as a double and narrowed to
// int32 (see SimAllocator's Populate pass).
inline constexpr std::size_t kCollisionGroupCount = 16;

// Per-body group membership/filter bits, one ActivationMask per lane.
using CollisionGroupMask = math::
    ActivationMask<util::MaskStorageFor<MathematicalT>, kCollisionGroupCount>;

// How many targets one PairTraversal call tests a source against -- and so
// how many bits one kBoundsOverlap word holds.
inline constexpr std::size_t kPairLanes = util::LaneCountOf<MathematicalT>();

// One kBoundsOverlap word: bit t set when target (group * kPairLanes + t)
// overlaps the source row. Plain int32 storage, not a batch -- a whole lane
// group's result packs into one of these (see topology::PairGroupIndexing).
using PairLaneMask = math::ActivationMask<std::int32_t, kPairLanes>;

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
//
// Which pairs are even allowed to collide is per-body data too, Bullet-
// style: kCollisionGroup is the set of groups a body belongs to,
// kCollisionMask the set of groups it collides with, and a pair (a, b) is
// tested only when each one's group intersects the other's mask -- two ANDs
// per lane, no branch. That one rule covers every exclusion the broad phase
// needs: a parent/child pair that always touches at its joint is just two
// groups that leave each other out of their masks, and a padding row (or a
// body whose archetype gives no "collision_group") reads Zero() and so
// never collides with anything.
//
// The result, kBoundsOverlap, is the rows x (rows / kPairLanes) bit matrix
// of which pairs both pass the filter and overlap -- one PairLaneMask word
// per (source row, target lane group), see topology::PairGroupIndexing.
// Only the upper triangle is meaningful: a set bit is a real candidate pair
// only when its target row is greater than its source row (see
// engine::pass::PairTraversal, which also never writes the words below the
// diagonal group).
enum class CollisionField : uint8_t {
  // --- Model/kinematic inputs ---
  kWorldTransform,    // x_world -- ABA's own kWorldTransform, read-only here
  kCollisionCenter,   // c_local (bound center, in the body's own frame)
  kCollisionExtents,  // h_local (bound half-extents, along the body's axes)
  kCollisionGroup,    // groups this body belongs to
  kCollisionMask,     // groups this body collides with

  // --- Broad-phase outputs ---
  kWorldBoundCenter,   // c_world
  kWorldBoundExtents,  // h_world (half-extents, along the world axes)
  kBoundsOverlap,      // per (source row, target lane group) overlap bits

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
struct CollisionFieldTraits<CollisionField::kCollisionGroup> {
  using Assembler = math::ActivationMaskAssembler<kCollisionGroupCount>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "collision_group";
};
template <>
struct CollisionFieldTraits<CollisionField::kCollisionMask> {
  using Assembler = math::ActivationMaskAssembler<kCollisionGroupCount>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "collision_mask";
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
template <>
struct CollisionFieldTraits<CollisionField::kBoundsOverlap> {
  using Assembler = math::ActivationMaskAssembler<kPairLanes>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Indexing = engine::topology::PairGroupIndexing<kPairLanes>;
  using Layout = engine::topology::PlanarLayout;
};
static_assert(engine::FieldTraitsLike<CollisionField, CollisionFieldTraits>);

using CollisionView =
    engine::view::ViewFactory<CollisionField, CollisionFieldTraits>;
static_assert(engine::view::ViewLike<CollisionView, CollisionField>);

using CollisionTopology = domain::JointTopology;
static_assert(domain::TopologyLike<CollisionTopology>);

}  // namespace achilles::algorithms::collision
