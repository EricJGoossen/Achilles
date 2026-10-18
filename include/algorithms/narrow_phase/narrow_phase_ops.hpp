#pragma once

#include <array>

#include "algorithms/conventions.hpp"
#include "algorithms/narrow_phase/narrow_phase_data.hpp"
#include "engine/pass/op_invoker.hpp"

namespace achilles::algorithms::narrow_phase {

// The contact Op for the sphere-swept box x sphere-swept box pair (see
// broad_phase_data.hpp for the shape) -- one source body (kSource, the same
// value in every lane) against a whole lane group of target bodies
// (kTarget) at once, all in the world frame.
//
// A sphere-swept box is its inner box grown by its radius, so two of them
// are exactly as far apart as their inner boxes are, less both radii. Two
// disjoint boxes' closest points are always either a vertex of one against
// the other box, or an edge of each -- so every candidate of both kinds is
// computed and the closest kept: each of the 16 vertices clamped into the
// other box (in that box's own frame, a per-axis clamp), and for each of the
// 9 edge-direction pairs the one edge of each box that supports it along
// that pair's normal, segment against segment. Every candidate is a real
// point pair, so their min is exactly the gap -- a fixed count of clamps and
// selects, no iteration, no branch whatever the geometry. The gap, less the
// radii, is the signed distance, and its direction the normal: exact for
// every contact shallower than r_s + r_t, which is every contact a sphere or
// a capsule ever makes.
//
// Once the inner boxes themselves intersect, the closest points meet and
// carry no direction. Those lanes take the separating-axis test's answer
// instead: the inner boxes' 15 candidate axes (each box's 3 face normals
// and the 9 edge-edge crosses), the axis of least penetration giving both
// the depth and the normal, and the source's deepest point along it the
// contact. The same test's sign is what says which lanes intersect -- SAT
// is exact for that -- and both answers are computed in every lane and one
// selected per lane, never branched on.
//
// The contact point is midway between the two surfaces along the normal,
// and a lane's contact is active when the broad phase kept the pair and the
// distance is within max(m_s, m_t), MuJoCo's pair margin.
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
      ArgData{FieldEnum::kContactDistance, ArgIndex::kTarget},
      ArgData{FieldEnum::kContactNormal, ArgIndex::kTarget},
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
      Scalar* distance_out,
      Vector3* normal_out,
      Vector3* point_out,
      LaneMask* active_out
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
