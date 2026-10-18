#include "algorithms/narrow_phase/narrow_phase_ops.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <xsimd/xsimd.hpp>

#include "algorithms/conventions.hpp"
#include "util/simd_ops.hpp"

namespace achilles::algorithms::narrow_phase {

namespace {

// Below this gap the closest points carry no usable direction, so the SAT
// axis supplies the normal instead; below this squared length an edge-edge
// cross axis is taken to be degenerate (parallel edges, whose separation a
// face axis already covers) and below this squared length an edge is taken
// to be a point.
constexpr float kGapEpsilon = 1e-6F;
constexpr float kAxisEpsilon = 1e-6F;
constexpr float kEdgeEpsilon = 1e-12F;
// Stands in for "no candidate yet" in the running SAT max/closest-pair min.
constexpr float kNoSeparation = -1e30F;
constexpr float kNoGap = 1e30F;

// The running min over candidate point pairs (on the source, on the
// target) of their squared gap.
struct ClosestPair {
  MathematicalT gap_sq;
  Vector3 on_source;
  Vector3 on_target;
};

ClosestPair KeepCloser(
    const ClosestPair& best, const Vector3& on_source, const Vector3& on_target
) {
  MathematicalT gap_sq = (on_target - on_source).SquaredNorm();
  auto closer = gap_sq < best.gap_sq;
  return ClosestPair{
      util::Select(closer, gap_sq, best.gap_sq),
      Vector3::Select(closer, on_source, best.on_source),
      Vector3::Select(closer, on_target, best.on_target)
  };
}

// Closest points of segments a0 + s u (s in [0, 1]) and b0 + t v (t in
// [0, 1]): s from the unclamped optimum, t from s, then s again from the
// clamped t -- each a clamp, so no case split on which end binds.
ClosestPair SegmentPair(
    const ClosestPair& best,
    const Vector3& a0,
    const Vector3& u,
    const Vector3& b0,
    const Vector3& v
) {
  Vector3 w = a0 - b0;
  MathematicalT uu = xsimd::max(u.Dot(u), MathematicalT(kEdgeEpsilon));
  MathematicalT vv = xsimd::max(v.Dot(v), MathematicalT(kEdgeEpsilon));
  MathematicalT uv = u.Dot(v);
  MathematicalT uw = u.Dot(w);
  MathematicalT vw = v.Dot(w);
  MathematicalT zero(0);
  MathematicalT one(1);

  // s = (uv vw - vv uw) / (uu vv - uv^2), 0 when parallel
  MathematicalT den = uu * vv - uv * uv;
  MathematicalT s = util::Select(
      den > MathematicalT(kEdgeEpsilon) * uu * vv,
      (uv * vw - vv * uw) / xsimd::max(den, MathematicalT(kEdgeEpsilon)),
      zero
  );
  s = xsimd::min(xsimd::max(s, zero), one);
  // t = clamp((uv s + vw) / vv), s = clamp((uv t - uw) / uu)
  MathematicalT t = xsimd::min(xsimd::max((uv * s + vw) / vv, zero), one);
  s = xsimd::min(xsimd::max((uv * t - uw) / uu, zero), one);
  return KeepCloser(best, b0 + v * t, a0 + u * s);
}

// Exact closest points of two disjoint boxes: the pair is always a vertex
// of one against the other box (a vertex clamped into it -- covering
// vertex-face, vertex-edge and vertex-vertex) or an edge of each against
// the other (both edges then support their boxes along the pair's normal,
// which is parallel to the two edges' cross and points the way the
// centers do, so each axis pair (i, j) names exactly one edge of each).
// Every candidate is a real point of each box, so their min is the gap.
ClosestPair ClosestPoints(
    const OrientedBox& target, const OrientedBox& source
) {
  ClosestPair best{MathematicalT(kNoGap), source.Center(), target.Center()};
  for (std::size_t corner = 0; corner < 8; ++corner) {
    Vector3 on_target = target.Vertex(corner);
    best = KeepCloser(best, source.Project(on_target), on_target);
    Vector3 on_source = source.Vertex(corner);
    best = KeepCloser(best, on_source, target.Project(on_source));
  }

  Vector3 d = target.Center() - source.Center();
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      Vector3 a = target.Axis(i);
      Vector3 b = source.Axis(j);
      // n = sgn(L . D) L, L = a_i x b_j
      Vector3 cross = a.Cross(b);
      Vector3 n = cross * util::Sign(cross.Dot(d));

      // the target's edge along a_i supporting -n, the source's along b_j
      // supporting n -- each the support point with its own axis zeroed,
      // then spanned along that axis
      Vector3 a_mid = target.Support(-n) -
                      a * (target.HalfExtent(i) * util::Sign(-a.Dot(n)));
      Vector3 b_mid =
          source.Support(n) - b * (source.HalfExtent(j) * util::Sign(b.Dot(n)));
      Vector3 a_half = a * target.HalfExtent(i);
      Vector3 b_half = b * source.HalfExtent(j);
      best = SegmentPair(
          best,
          a_mid - a_half,
          a_half * MathematicalT(2),
          b_mid - b_half,
          b_half * MathematicalT(2)
      );
    }
  }
  return best;
}

// The running max of the separation along each candidate axis, and the
// unit axis it came from, signed source -> target.
struct Separation {
  MathematicalT depth;
  Vector3 axis;
};

// s = |L . D| - w_t(L) - w_s(L), for unit L; kept when it beats the best so
// far and L isn't degenerate
Separation TestAxis(
    const Separation& best,
    const Vector3& axis,
    const Vector3& d,
    const OrientedBox& target,
    const OrientedBox& source
) {
  MathematicalT length_sq = axis.SquaredNorm();
  auto valid = length_sq > MathematicalT(kAxisEpsilon);
  Vector3 unit =
      axis / xsimd::sqrt(xsimd::max(length_sq, MathematicalT(kAxisEpsilon)));

  MathematicalT along = unit.Dot(d);
  MathematicalT s =
      xsimd::abs(along) - target.HalfWidth(unit) - source.HalfWidth(unit);
  s = util::Select(valid, s, MathematicalT(kNoSeparation));

  auto better = s > best.depth;
  return Separation{
      util::Select(better, s, best.depth),
      Vector3::Select(better, unit * util::Sign(along), best.axis)
  };
}

// The 15 SAT axes: 3 face normals of each box, then the 9 edge crosses. The
// max is positive exactly when the boxes are disjoint, and otherwise minus
// the depth of least penetration.
Separation SeparatingAxis(
    const OrientedBox& target, const OrientedBox& source
) {
  Vector3 d = target.Center() - source.Center();
  Separation best{MathematicalT(kNoSeparation), Vector3::UnitX()};
  for (std::size_t i = 0; i < 3; ++i) {
    best = TestAxis(best, target.Axis(i), d, target, source);
    best = TestAxis(best, source.Axis(i), d, target, source);
  }
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      best = TestAxis(
          best, target.Axis(i).Cross(source.Axis(j)), d, target, source
      );
    }
  }
  return best;
}

}  // namespace

void ComputeSweptBoxContactOp::operator()(
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
) const {
  OrientedBox target(
      x_target.Translation(), x_target.Rotation().ToRotationMatrix(), h_target
  );
  OrientedBox source(
      x_source.Translation(), x_source.Rotation().ToRotationMatrix(), h_source
  );

  // Disjoint inner boxes: their closest points p (source), q (target)
  ClosestPair closest = ClosestPoints(target, source);
  MathematicalT gap = xsimd::sqrt(closest.gap_sq);

  // Intersecting inner boxes: the axis of least penetration
  Separation sat = SeparatingAxis(target, source);

  // separated: s_max > 0 and |q - p| has a direction
  auto separated =
      (sat.depth > MathematicalT(0)) & (gap > MathematicalT(kGapEpsilon));

  // n = (q - p) / |q - p|, or the SAT axis
  Vector3 n = Vector3::Select(
      separated,
      (closest.on_target - closest.on_source) /
          xsimd::max(gap, MathematicalT(kGapEpsilon)),
      sat.axis
  );

  // d = (|q - p| or s_max) - r_t - r_s
  MathematicalT r_t = r_target.Value();
  MathematicalT r_s = r_source.Value();
  MathematicalT d = util::Select(separated, gap, sat.depth) - r_t - r_s;
  *distance_out = Scalar(d);
  *normal_out = n;

  // a = p, or the source's deepest point toward the target; the point is
  // midway between the source's surface a + n r_s and the target's, d on
  Vector3 a = Vector3::Select(separated, closest.on_source, source.Support(n));
  *point_out = a + n * (r_s + d * MathematicalT(0.5F));

  // candidate && d <= max(m_s, m_t)
  auto within = d <= xsimd::max(m_target.Value(), m_source.Value());
  *active_out =
      LaneMask(static_cast<std::int32_t>(within.mask() & candidate.AsStorage())
      );
}

}  // namespace achilles::algorithms::narrow_phase
