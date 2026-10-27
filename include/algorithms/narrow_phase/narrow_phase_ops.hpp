#pragma once

#include <array>

#include "algorithms/conventions.hpp"
#include "algorithms/narrow_phase/narrow_phase_data.hpp"
#include "engine/pass/op_invoker.hpp"

namespace achilles::algorithms::narrow_phase {

// The contact Op for the sphere-swept box x sphere-swept box pair (see
// broad_phase_data.hpp for the shape) -- one source body (kSource, the same
// value in every lane) against a whole lane group of target bodies
// (kTarget) at once, all in the world frame. It follows MuJoCo's own
// primitive colliders, merged: a sphere-swept box is a sphere, a capsule or
// a box depending on its extents, and MuJoCo's sphere-box, capsule-box and
// box-box routines are the three cases below. Every case is computed in
// every lane and one is selected per lane, never branched on.
//
// A sphere-swept box is its inner box grown by its radius, so every
// distance is measured between the inner boxes and both radii subtracted --
// exactly how MuJoCo reduces a sphere or a capsule to its center or
// segment. Three things are computed for every pair:
//
//   - The inner boxes' exact closest points, when they're disjoint: every
//     vertex of each clamped into the other (MuJoCo's sphere-box clamp),
//     and each edge-direction pair's supporting edges segment against
//     segment (its capsule-box edge loop). The closest of those 25 real
//     point pairs is exactly the gap.
//   - MuJoCo's box-box separating-axis test over the same 15 axes, with its
//     face preference: an edge axis only wins when it beats the best face
//     by a relative bias, and even then a face within 0.99 of it wins back
//     unless the edge is more than 5% better -- so near-ties never flicker
//     the normal between a face and an edge from one tick to the next.
//   - When a face wins, its manifold, as MJX's box-box builds it: the face
//     of the other box most opposed to it, each face's edges clipped to the
//     other's side planes, and four of the resulting points kept -- the
//     deepest, the one farthest from it, then the two that widen the patch
//     most.
//
// Which of them a lane's contact is:
//
//   - A face won and the inner boxes intersect, or they're disjoint and
//     their closest points meet along that face's normal (a box resting on
//     a box, a capsule lying on one): the face manifold, up to four slots --
//     its first pick the exact closest pair when they're disjoint, since the
//     clipped patch can miss it when it sits just past the face's rim.
//   - The inner boxes are disjoint and that isn't face-on (a sphere, a
//     capsule end, a corner or an edge approaching): the exact closest
//     pair, one slot -- exact for every contact shallower than r_s + r_t,
//     which pure box-box SAT is not for a sphere's or a capsule's
//     degenerate axes.
//   - An edge won and the inner boxes intersect: MuJoCo's edge-edge
//     contact, the winning axis's supporting edges' closest points, one
//     slot.
//
// Each slot's point is midway between the two surfaces along the normal,
// and a slot is active when the broad phase kept the pair, the slot holds a
// real point no earlier active slot already holds, and its distance is
// within m_s + m_t, MuJoCo's pair margin (mj_assignMargin's sum).
struct ComputeSweptBoxContactOp {
  using FieldEnum = NarrowPhaseField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  static constexpr std::array<ArgData, 9> kInputs = {
      ArgData{FieldEnum::kWorldShapePose, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionExtents, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionRadius, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionMargin, ArgIndex::kTarget},
      ArgData{FieldEnum::kWorldShapePose, ArgIndex::kSource},
      ArgData{FieldEnum::kCollisionExtents, ArgIndex::kSource},
      ArgData{FieldEnum::kCollisionRadius, ArgIndex::kSource},
      ArgData{FieldEnum::kCollisionMargin, ArgIndex::kSource},
      ArgData{FieldEnum::kBoundsOverlap, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 4> kOutputs = {
      ArgData{FieldEnum::kContactNormal, ArgIndex::kTarget},
      ArgData{FieldEnum::kContactDistance, ArgIndex::kTarget},
      ArgData{FieldEnum::kContactPoint, ArgIndex::kTarget},
      ArgData{FieldEnum::kContactActive, ArgIndex::kTarget},
  };

  void operator()(
      const Transform& x_target,
      const Vector3& h_target,
      const Scalar& r_target,
      const Scalar& m_target,
      const Transform& x_source,
      const Vector3& h_source,
      const Scalar& r_source,
      const Scalar& m_source,
      const LaneMask& candidate,
      Vector3* normal_out,
      ContactDistances* distance_out,
      ContactPoints* point_out,
      ContactActiveMask* active_out
  ) const;
};
static_assert(
    engine::OpLike<ComputeSweptBoxContactOp>,
    "ComputeSweptBoxContactOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<ComputeSweptBoxContactOp, NarrowPhaseView>,
    "ComputeSweptBoxContactOp's operator() must line up with kInputs/kOutputs"
);

}  // namespace achilles::algorithms::narrow_phase
