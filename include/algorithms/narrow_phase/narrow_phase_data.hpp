#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/scalar.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::narrow_phase {

// How many contacts one shape-type pair's contact Op writes per body pair --
// MJX-style, a fixed count per type pair rather than a variable-length list,
// so every lane writes the same number of contacts and a consumer reads them
// all, masked by kContactActive. Four for the sphere-swept box pair, MJX's
// own box-box count: a face resting on a face (box on box, a capsule lying
// on a box) is held by a clipped manifold of up to four points, while a
// sphere, a capsule end, an edge crossing an edge, or any contact that isn't
// face-on fills only the first slot.
inline constexpr std::size_t kSweptBoxContactSlots = 4;

// One per-pair slot's worth of each per-slot contact quantity: slot k is
// row/column k.
using ContactDistances = math::Matrix<MathematicalT, kSweptBoxContactSlots, 1>;
using ContactPoints = math::Matrix<MathematicalT, 3, kSweptBoxContactSlots>;

// One bit per (slot, lane), bit (k * kLanes + t) for slot k of target lane
// t, packed into one int32 word per (source row, target lane group) the same
// way kBoundsOverlap packs one bit per lane.
using ContactActiveMask =
    math::ActivationMask<std::int32_t, kLanes * kSweptBoxContactSlots>;
static_assert(
    kLanes * kSweptBoxContactSlots <= 32,
    "Every (slot, lane) contact bit must fit in one int32 word -- a wider "
    "SIMD target needs fewer slots or a wider mask word."
);

// The narrow phase: the second of the two collision Algorithms, run right
// after the broad phase (broad_phase/broad_phase_data.hpp). For every pair
// the broad phase left a candidate bit for in kBoundsOverlap, it computes
// where and how deeply the two bodies' actual shapes touch -- the contact
// geometry a contact solver consumes. Everything it reads besides that bit
// is the broad phase's own data, through the same SharedAs slots
// (shared_slots.hpp): each body's static sphere-swept box (inner half-
// extents, radius, margin) and the world pose the broad phase already
// composed for it, kWorldShapePose. Nothing is copied between the two.
//
// Dense and masked, like the broad phase's own pair test: the contact Op
// runs over the same PairTraversal, so it computes every lane of every call,
// and only kContactActive says which of those lanes are real contacts. No
// lane ever branches on whether it's a candidate; a GPU grid over (source,
// target group) does the same work in every thread.
//
// Up to kSweptBoxContactSlots contacts per pair, as three per-pair fields
// -- the rows x rows matrix of each, one batch per (source row, target lane
// group), lane t for target (group * kLanes + t), see
// topology::PairGroupIndexing<kLanes, kLanes>:
//
//   - kContactNormal: world unit vector from the source body toward the
//     target body -- the direction pushing them apart moves the target. One
//     per pair, shared by every slot, as in MuJoCo/MJX.
//   - kContactDistance: per slot, the signed distance between the two
//     shapes' surfaces there -- positive while separated, negative once
//     interpenetrating, by the penetration depth.
//   - kContactPoint: per slot, the world point midway between the two
//     surfaces along the normal, i.e. where a contact force acts on both.
//
// and one per-group word, kContactActive (see ContactActiveMask): bit
// (k * kLanes + t) set when lane t's pair was a broad-phase candidate and its
// slot k is a real contact within the pair's margin, MuJoCo's
// max(m_s, m_t) -- a contact a solver should consider even though it may not
// yet touch. Slots whose bit is clear still hold whatever the Op computed (or
// the zero-fill) and mean nothing.
//
// More shape types later each bring their own contact Op per type pair
// (sphere-swept box x mesh, ...), writing these same four fields: the solver
// only ever reads contacts, never shapes, so nothing downstream changes per
// type. Which Op handles which pair is settled by grouping pairs by type, as
// MJX does, never by a per-lane branch.
enum class NarrowPhaseField : uint8_t {
  // --- Broad-phase inputs (shared, read-only here) ---
  kWorldShapePose,    // x (the shape's frame, in the world)
  kCollisionExtents,  // h (inner box half-extents, along the shape's axes)
  kCollisionRadius,   // r (sweep radius around the inner box)
  kCollisionMargin,   // m (contact margin)
  kBoundsOverlap,     // per (source row, target lane group) candidate bits

  // --- Narrow-phase outputs ---
  kContactNormal,    // n (world, source -> target)
  kContactDistance,  // d_k (signed surface distance, per slot)
  kContactPoint,     // p_k (world, per slot)
  kContactActive,    // per (source row, target lane group) (slot, lane) bits

  kCount
};
static_assert(engine::FieldEnumLike<NarrowPhaseField>);

template <NarrowPhaseField F>
struct NarrowPhaseFieldTraits;

// Every NarrowPhaseField binds Ordering = TopologicalOrdering for the same
// reason every BroadPhaseField does (see broad_phase_data.hpp): it shares
// rows with the broad phase's own fields, so it has to share the ordering
// that numbered them.
template <>
struct NarrowPhaseFieldTraits<NarrowPhaseField::kWorldShapePose> {
  // SharedAs = WorldShapePoseSlot so this is the broad phase's own
  // kWorldShapePose block, composed there once per tick, not recomposed
  // here per pair.
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = WorldShapePoseSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct NarrowPhaseFieldTraits<NarrowPhaseField::kCollisionExtents> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using SharedAs = CollisionExtentsSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "collision_extents";
};
template <>
struct NarrowPhaseFieldTraits<NarrowPhaseField::kCollisionRadius> {
  using Assembler = math::ScalarAssembler<ScalarOperationT>;
  using SharedAs = CollisionRadiusSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "collision_radius";
};
template <>
struct NarrowPhaseFieldTraits<NarrowPhaseField::kCollisionMargin> {
  using Assembler = math::ScalarAssembler<ScalarOperationT>;
  using SharedAs = CollisionMarginSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "collision_margin";
};
template <>
struct NarrowPhaseFieldTraits<NarrowPhaseField::kBoundsOverlap> {
  // SharedAs = BoundsOverlapSlot so this is the broad phase's own result,
  // read-only here.
  using Assembler = math::ActivationMaskAssembler<kLanes>;
  using SharedAs = BoundsOverlapSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Indexing = engine::topology::PairGroupIndexing<kLanes>;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct NarrowPhaseFieldTraits<NarrowPhaseField::kContactDistance> {
  using Assembler =
      math::MatrixAssembler<ScalarOperationT, kSweptBoxContactSlots, 1>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Indexing = engine::topology::PairGroupIndexing<kLanes, kLanes>;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct NarrowPhaseFieldTraits<NarrowPhaseField::kContactNormal> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Indexing = engine::topology::PairGroupIndexing<kLanes, kLanes>;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct NarrowPhaseFieldTraits<NarrowPhaseField::kContactPoint> {
  using Assembler =
      math::MatrixAssembler<ScalarOperationT, 3, kSweptBoxContactSlots>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Indexing = engine::topology::PairGroupIndexing<kLanes, kLanes>;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct NarrowPhaseFieldTraits<NarrowPhaseField::kContactActive> {
  using Assembler =
      math::ActivationMaskAssembler<kLanes * kSweptBoxContactSlots>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Indexing = engine::topology::PairGroupIndexing<kLanes>;
  using Layout = engine::topology::PlanarLayout;
};
static_assert(engine::
                  FieldTraitsLike<NarrowPhaseField, NarrowPhaseFieldTraits>);

using NarrowPhaseView =
    engine::view::ViewFactory<NarrowPhaseField, NarrowPhaseFieldTraits>;
static_assert(engine::view::ViewLike<NarrowPhaseView, NarrowPhaseField>);

using NarrowPhaseTopology = domain::JointTopology;
static_assert(domain::TopologyLike<NarrowPhaseTopology>);

}  // namespace achilles::algorithms::narrow_phase
