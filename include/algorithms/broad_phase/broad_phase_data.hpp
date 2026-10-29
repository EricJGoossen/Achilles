#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/orderings/collision_pairs.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/scalar.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/orderings.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "util/simd_ops.hpp"

namespace achilles::algorithms::broad_phase {

// How many distinct collision groups a body can belong to/collide with --
// the bit width of kCollisionGroup/kCollisionMask. 16 rather than the full
// 32 an int32 lane holds so that "every group" is a small positive literal
// (65535) in archetype data, which is read as a double and narrowed to
// int32 (see SimAllocator's Populate pass).
inline constexpr std::size_t kCollisionGroupCount = 16;

// Per-body group membership/filter bits, one ActivationMask per lane.
using CollisionGroupMask = math::
    ActivationMask<util::MaskStorageFor<MathematicalT>, kCollisionGroupCount>;

// Every body carries one collision shape, and the broad phase derives a
// world-axis-aligned bound from it each tick that a pair pass can overlap-
// test with nothing but per-axis compares. It's the first of two collision
// Algorithms: the narrow phase (narrow_phase/narrow_phase_data.hpp) runs
// right after it and turns the candidate pairs it ends on into contacts,
// reading this phase's shape, world shape pose and kBoundsOverlap through
// SharedAs slots (shared_slots.hpp) -- the same memory, nothing copied.
//
// The shape is a sphere-swept box -- a box with every point of its surface
// pushed out by a radius, i.e. the set of points within kCollisionRadius of
// the inner box kCollisionExtents. That one type covers a sphere (zero
// extents), a capsule (one nonzero extent), a box (zero radius) and a
// rounded box, so the narrow phase has exactly one shape pair to handle and
// never branches on shape type. kCollisionPose places the shape's own frame
// in the body's, so a capsule can lie along any axis and a link's geometry
// needn't be centred on its joint. All four are static per-archetype data,
// set once at load time.
//
// More shape types later each bring their own shape fields and their own
// bound Op writing the same kWorldBoundCenter/kWorldBoundExtents (see
// ComputeSweptBoxBoundsOp, broad_phase_ops.hpp) -- the pair test only ever
// reads the bound, so nothing downstream of it changes per type.
//
// Center + half-extents rather than min/max corners: the shape maps onto its
// world bound with one rotate (center) and one |R| product (extents), and the
// pair test |c_a - c_b| <= h_a + h_b is a per-axis compare either way. The
// radius isn't rotated at all -- the swept sphere looks the same from every
// direction -- so the bound stays as tight as the inner box's own.
//
// kCollisionMargin is MuJoCo's geom margin: it inflates the world bound on
// every axis, so a pair becomes a candidate once its bounds come within
// that distance, not only once they touch -- catching a pair a tick early
// rather than after it's already interpenetrated. (MuJoCo's `gap`, which
// decides whether such a not-yet-touching contact produces force, belongs
// to the contact solver and has no broad-phase meaning.)
//
// A body that gives none of these in its archetype reads the zero-fill
// default: zero extents, radius and margin -- a point at its body origin --
// and a zero kCollisionPose, whose rotation part is read as identity (see
// ComputeSweptBoxBoundsOp), so the shape just sits on the body frame. That's
// the same "absent data reads as an inert default" convention algorithms::viz's
// own kVisualExtents follows (viz_data.hpp); such a body also has no
// kCollisionGroup, so it never collides anyway.
//
// Which pairs are even allowed to collide is per-body data too, Bullet-
// style: kCollisionGroup is the set of groups a body belongs to,
// kCollisionMask the set of groups it collides with, and a pair (a, b) is
// tested only when each one's group intersects the other's mask -- two ANDs
// per lane, no branch. A padding row (or a body whose archetype gives no
// "collision_group") reads Zero() and so never collides with anything.
//
// Groups and masks never change, so the pairs they rule out are dropped
// once, before the first tick: the pair test runs over CollisionPairs
// (orderings/collision_pairs.hpp), one row per pair of bodies that can collide,
// not every pair of rows. Its lanes are distinct, non-parent/child pairs with
// the partner row above the source row (engine::pass::PairStructure), and
// the pair test ANDs that in, still without a branch -- dropping a
// parent/child pair, which always touches at the joint it shares (MuJoCo's
// filterparent), a body paired with itself, the duplicate half of every
// pair, and every pair the filter rules out.
//
// The result, kBoundsOverlap, is one LaneMask word per lane group of
// CollisionPairs (topology::LaneGroupIndexing): bit t set when lane t's
// pair both passes the filter and overlaps. Every set bit is a real,
// distinct candidate pair with partner row > source row.
enum class BroadPhaseField : uint8_t {
  // --- Model/kinematic inputs ---
  kWorldTransform,    // x_world -- ABA's own kWorldTransform, read-only here
  kCollisionPose,     // x_shape (the shape's frame, in the body's own)
  kCollisionExtents,  // h (inner box half-extents, along the shape's axes)
  kCollisionRadius,   // r (sweep radius around the inner box)
  kCollisionMargin,   // m (bound inflation, on every world axis)
  kCollisionGroup,    // groups this body belongs to
  kCollisionMask,     // groups this body collides with

  // --- Broad-phase outputs ---
  kWorldShapePose,     // x_world x_shape (the shape's frame, in the world)
  kWorldBoundCenter,   // c_world
  kWorldBoundExtents,  // h_world (half-extents, along the world axes)
  kBoundsOverlap,      // per lane group of pairs, overlap bits

  kCount
};
static_assert(engine::FieldEnumLike<BroadPhaseField>);

template <BroadPhaseField F>
struct BroadPhaseFieldTraits;

// Every BroadPhaseField binds Ordering = TopologicalOrdering, matching ABA's
// own choice for kWorldTransform (aba_data.hpp) -- for the same reason
// algorithms::viz does (see viz_data.hpp's own comment): a View addresses
// every one of its fields by the same sorted row index, so every field has
// to share the ordering policy that produced that numbering, or row i of
// kCollisionExtents and row i of kWorldTransform could silently refer to two
// different joints.
template <>
struct BroadPhaseFieldTraits<BroadPhaseField::kWorldTransform> {
  // SharedAs = WorldTransformSlot so this is ABA's own kWorldTransform
  // block (aba_data.hpp), not a private, unconnected copy -- nothing in
  // algorithms::broad_phase itself ever writes it.
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = WorldTransformSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct BroadPhaseFieldTraits<BroadPhaseField::kCollisionPose> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = CollisionPoseSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "collision_pose";
};
template <>
struct BroadPhaseFieldTraits<BroadPhaseField::kCollisionExtents> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using SharedAs = CollisionExtentsSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "collision_extents";
};
template <>
struct BroadPhaseFieldTraits<BroadPhaseField::kCollisionRadius> {
  using Assembler = math::ScalarAssembler<ScalarOperationT>;
  using SharedAs = CollisionRadiusSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "collision_radius";
};
template <>
struct BroadPhaseFieldTraits<BroadPhaseField::kCollisionMargin> {
  using Assembler = math::ScalarAssembler<ScalarOperationT>;
  using SharedAs = CollisionMarginSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "collision_margin";
};
template <>
struct BroadPhaseFieldTraits<BroadPhaseField::kCollisionGroup> {
  using Assembler = math::ActivationMaskAssembler<kCollisionGroupCount>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = CollisionMaskFields::kGroup;
};
template <>
struct BroadPhaseFieldTraits<BroadPhaseField::kCollisionMask> {
  using Assembler = math::ActivationMaskAssembler<kCollisionGroupCount>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = CollisionMaskFields::kMask;
};
template <>
struct BroadPhaseFieldTraits<BroadPhaseField::kWorldShapePose> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = WorldShapePoseSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct BroadPhaseFieldTraits<BroadPhaseField::kWorldBoundCenter> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct BroadPhaseFieldTraits<BroadPhaseField::kWorldBoundExtents> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct BroadPhaseFieldTraits<BroadPhaseField::kBoundsOverlap> {
  using Assembler = math::ActivationMaskAssembler<kLanes>;
  using SharedAs = BoundsOverlapSlot;
  using Ordering = CollisionPairs;
  using Indexing = engine::topology::LaneGroupIndexing<kLanes>;
  using Layout = engine::topology::PlanarLayout;
};
static_assert(engine::FieldTraitsLike<BroadPhaseField, BroadPhaseFieldTraits>);

using BroadPhaseView =
    engine::view::ViewFactory<BroadPhaseField, BroadPhaseFieldTraits>;
static_assert(engine::view::ViewLike<BroadPhaseView, BroadPhaseField>);

using BroadPhaseTopology = domain::JointTopology;
static_assert(domain::TopologyLike<BroadPhaseTopology>);

}  // namespace achilles::algorithms::broad_phase
