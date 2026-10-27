#include "algorithms/narrow_phase/narrow_phase_ops.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <xsimd/xsimd.hpp>

#include "algorithms/conventions.hpp"
#include "algorithms/narrow_phase/narrow_phase_data.hpp"
#include "util/simd_ops.hpp"

namespace achilles::algorithms::narrow_phase {

namespace {

using Mask = decltype(MathematicalT() == MathematicalT());

// Below this gap the closest points carry no usable direction; below this
// sin^2 an edge-edge cross axis is taken to be degenerate (parallel edges,
// whose separation a face axis already covers).
constexpr float kGapEpsilon = 1e-6F;
constexpr float kAxisEpsilon = 1e-6F;
// Stand in for "no candidate yet" in the running maxes/mins.
constexpr float kNoSeparation = -1e30F;
constexpr float kNoGap = 1e30F;

// MuJoCo's box-box face preference (engine_collision_box.c): an edge axis
// must beat the best face axis by this relative bias to win at all
// (mjBOXBOX_EDGEBIAS, 1e-6 in double; float rounding needs more), and a face
// axis within kFaceEdgeParallel of the winning edge axis wins back unless
// the edge is more than kFaceEdgeAdvantage better.
constexpr float kEdgeBias = 1e-5F;
constexpr float kFaceEdgeParallel = 0.99F;
constexpr float kFaceEdgeAdvantage = 0.05F;
constexpr float kMinVal = 1e-15F;

// Disjoint inner boxes whose closest points meet along the winning face's
// normal to within this cosine are face-on (a vertex or an edge or a face
// resting on that face), so get its manifold rather than one point.
constexpr float kFaceOnAlignment = 0.9999F;
// Two manifold points closer than this (squared) are the same point.
constexpr float kDuplicateSq = 1e-8F;

// 4 incident edges clipped to the reference face, and 4 reference edges
// clipped to the incident face, 2 points each.
constexpr std::size_t kClipPoints = 16;

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
  Mask closer = gap_sq < best.gap_sq;
  return ClosestPair{
      util::Select(closer, gap_sq, best.gap_sq),
      Vector3::Select(closer, on_source, best.on_source),
      Vector3::Select(closer, on_target, best.on_target)
  };
}

// The best face axis so far: its separation, the axis (signed source ->
// target), and whether it's the source's face or the target's.
struct FaceAxis {
  MathematicalT depth;
  Vector3 axis;
  Mask on_source;
};

// The best edge-edge axis so far, likewise, with its two supporting edges'
// closest points -- MuJoCo's edge-edge contact, should it win.
struct EdgeAxis {
  MathematicalT depth;
  Vector3 axis;
  Vector3 on_source;
  Vector3 on_target;
};

// Everything one pass over the 6 face axes and 9 edge pairs finds.
struct Survey {
  ClosestPair closest;
  FaceAxis face;
  EdgeAxis edge;
};

// s = |L . D| - w_t(L) - w_s(L), for unit L
MathematicalT Separation(
    const Vector3& unit, const Vector3& d, const Box& target, const Box& source
) {
  return xsimd::abs(unit.Dot(d)) - target.HalfWidth(unit) -
         source.HalfWidth(unit);
}

FaceAxis TestFace(
    const FaceAxis& best,
    const Vector3& axis,
    const Mask& on_source,
    const Vector3& d,
    const Box& target,
    const Box& source
) {
  MathematicalT s = Separation(axis, d, target, source);
  Mask better = s > best.depth;
  return FaceAxis{
      util::Select(better, s, best.depth),
      Vector3::Select(better, axis * util::Sign(axis.Dot(d)), best.axis),
      (better & on_source) | (!better & best.on_source)
  };
}

// One pass over every axis and feature pair:
//
//   - the 8 + 8 vertices clamped into the other box, and each edge-direction
//     pair's supporting edges segment against segment -- every candidate
//     for the inner boxes' closest pair, so their min is the gap. The
//     supporting edges are those along the pair's normal L = a_i x b_j,
//     signed the way the centers point, which for disjoint boxes is the way
//     the gap does;
//   - the 15 SAT axes: each box's 3 face normals, and the same 9 crosses,
//     whose supporting edges are then also MuJoCo's edge-edge contact.
Survey SurveyPair(const Box& target, const Box& source) {
  Vector3 d = target.Center() - source.Center();
  Mask yes = MathematicalT(0) == MathematicalT(0);

  ClosestPair closest{MathematicalT(kNoGap), source.Center(), target.Center()};
  for (std::size_t corner = 0; corner < 8; ++corner) {
    Vector3 on_target = target.Vertex(corner);
    closest = KeepCloser(closest, source.Project(on_target), on_target);
    Vector3 on_source = source.Vertex(corner);
    closest = KeepCloser(closest, on_source, target.Project(on_source));
  }

  FaceAxis face{MathematicalT(kNoSeparation), Vector3::UnitX(), !yes};
  for (std::size_t i = 0; i < 3; ++i) {
    face = TestFace(face, target.Axis(i), !yes, d, target, source);
    face = TestFace(face, source.Axis(i), yes, d, target, source);
  }

  EdgeAxis edge{
      MathematicalT(kNoSeparation),
      Vector3::UnitX(),
      source.Center(),
      target.Center()
  };
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      Vector3 a = target.Axis(i);
      Vector3 b = source.Axis(j);

      // n = sgn(L . D) L / |L|, L = a_i x b_j
      Vector3 cross = a.Cross(b);
      MathematicalT length_sq = cross.SquaredNorm();
      Mask valid = length_sq > MathematicalT(kAxisEpsilon);
      MathematicalT length =
          xsimd::sqrt(xsimd::max(length_sq, MathematicalT(kAxisEpsilon)));
      Vector3 n = cross * (util::Sign(cross.Dot(d)) / length);

      // the target's edge along a_i supporting -n, the source's along b_j
      // supporting n -- each the support point with its own axis zeroed,
      // then spanned along that axis
      Vector3 a_mid = target.Support(-n) -
                      a * (target.HalfExtent(i) * util::Sign(-a.Dot(n)));
      Vector3 b_mid =
          source.Support(n) - b * (source.HalfExtent(j) * util::Sign(b.Dot(n)));
      Vector3 a_half = a * target.HalfExtent(i);
      Vector3 b_half = b * source.HalfExtent(j);
      auto [on_target, on_source] =
          Segment(a_mid - a_half, a_mid + a_half)
              .ClosestPoints(Segment(b_mid - b_half, b_mid + b_half));
      closest = KeepCloser(closest, on_source, on_target);

      MathematicalT s = util::Select(
          valid, Separation(n, d, target, source), MathematicalT(kNoSeparation)
      );
      Mask better = s > edge.depth;
      edge = EdgeAxis{
          util::Select(better, s, edge.depth),
          Vector3::Select(better, n, edge.axis),
          Vector3::Select(better, on_source, edge.on_source),
          Vector3::Select(better, on_target, edge.on_target)
      };
    }
  }
  return Survey{closest, face, edge};
}

// Up to 4 contacts on a face: each one's point on the reference face's
// plane, its signed surface distance, and whether it holds a point within
// the margin -- plus whether clipping produced any point at all, margin
// aside.
struct Manifold {
  std::array<Vector3, kSweptBoxContactSlots> base{};
  std::array<MathematicalT, kSweptBoxContactSlots> depth{};
  std::array<Mask, kSweptBoxContactSlots> valid{};
  Mask any{};
};

// One clipped point the manifold's selection has kept so far.
struct Pick {
  Vector3 base;
  MathematicalT depth;
  Mask valid;
};

Pick Keep(
    const Pick& best,
    const Mask& better,
    const Vector3& base,
    const MathematicalT& depth
) {
  return Pick{
      Vector3::Select(better, base, best.base),
      util::Select(better, depth, best.depth),
      best.valid | better
  };
}

// The running argmax of score over the valid candidates.
template <typename Score>
Pick Farthest(
    const std::array<Vector3, kClipPoints>& base,
    const std::array<MathematicalT, kClipPoints>& depth,
    const std::array<Mask, kClipPoints>& valid,
    const Score& score
) {
  Pick best{base[0], depth[0], MathematicalT(0) != MathematicalT(0)};
  MathematicalT best_score(kNoSeparation);
  for (std::size_t i = 0; i < kClipPoints; ++i) {
    MathematicalT s = score(base[i]);
    Mask better = valid[i] & (s > best_score);
    best_score = util::Select(better, s, best_score);
    best = Keep(best, better, base[i], depth[i]);
  }
  return best;
}

// MJX's box-box face manifold (collision_convex.py): the reference face is
// the winning face, the incident face is the other box's face most opposed
// to it, and the contact patch is where they overlap -- the incident face's
// edges clipped to the reference face's side planes, and the reference
// face's edges, carried onto the incident face's plane along the normal,
// clipped to the incident face's, two points per edge. Each point's distance
// is its height above the reference face, less both radii; points beyond
// the margin are dropped. Four are kept, as MJX's _manifold_points picks
// them: the deepest (where MJX takes any one), the one farthest from it,
// the one farthest from their line, and the one that widens the patch most
// past both. When the inner boxes are disjoint the first pick is their exact
// closest pair instead -- the clipped patch can miss it when it sits just
// past the reference face's rim -- and the rest spread out from it. (A sphere's
// or a capsule's degenerate face clips to one or two distinct points, so some
// picks coincide; the caller drops those.)
Manifold FaceManifold(
    const FaceAxis& face,
    const Vector3& exact_point,
    const MathematicalT& exact_depth,
    const Mask& separated,
    const Box& target,
    const Box& source,
    const MathematicalT& r_ref,
    const MathematicalT& radii,
    const MathematicalT& margin
) {
  Box ref = Box::Select(face.on_source, source, target);
  Box inc = Box::Select(face.on_source, target, source);

  // n_out: the reference face's outward normal, toward the incident box
  Vector3 n_out = Vector3::Select(face.on_source, face.axis, -face.axis);
  Rectangle ref_face = ref.FaceAlong(n_out);
  Rectangle inc_face = inc.FaceAlong(-n_out);

  // p on the incident face -> q = p - n h on the reference face's plane,
  // d = h - r_t - r_s
  std::array<Vector3, kClipPoints> base{};
  std::array<MathematicalT, kClipPoints> depth{};
  std::array<Mask, kClipPoints> valid{};
  Mask any = MathematicalT(0) != MathematicalT(0);
  std::size_t next = 0;
  auto add = [&](const Vector3& p, const Mask& clipped) {
    MathematicalT h = ref_face.Height(p);
    base[next] = p - n_out * h;
    depth[next] = h - radii;
    valid[next] = clipped & (depth[next] <= margin);
    any = any | clipped;
    ++next;
  };
  for (std::size_t e = 0; e < 4; ++e) {
    Rectangle::Clipped incident = ref_face.Clip(inc_face.Edge(e));
    add(incident.segment.Start(), incident.valid);
    add(incident.segment.End(), incident.valid);

    Segment ref_edge = ref_face.Edge(e);
    Rectangle::Clipped reference = inc_face.Clip(Segment(
        inc_face.Hit(ref_edge.Start(), n_out),
        inc_face.Hit(ref_edge.End(), n_out)
    ));
    add(reference.segment.Start(), reference.valid);
    add(reference.segment.End(), reference.valid);
  }

  // a: the deepest, or the exact closest pair
  Pick a{base[0], MathematicalT(kNoGap), MathematicalT(0) != MathematicalT(0)};
  for (std::size_t i = 0; i < kClipPoints; ++i) {
    a = Keep(a, valid[i] & (depth[i] < a.depth), base[i], depth[i]);
  }
  // (the exact contact's own point, carried back onto the reference
  // plane the way every other pick's contact is carried off it)
  Vector3 exact_base =
      exact_point - n_out * (r_ref + exact_depth * MathematicalT(0.5F));
  a = Pick{
      Vector3::Select(separated, exact_base, a.base),
      util::Select(separated, exact_depth, a.depth),
      (separated & (exact_depth <= margin)) | (!separated & a.valid)
  };
  // b: farthest from a
  Pick b = Farthest(base, depth, valid, [&](const Vector3& p) {
    return (p - a.base).SquaredNorm();
  });
  // c: farthest from the line ab
  Vector3 ab = n_out.Cross(a.base - b.base);
  Pick c = Farthest(base, depth, valid, [&](const Vector3& p) {
    return xsimd::abs((p - a.base).Dot(ab));
  });
  // d: farthest past both bc and ac
  Vector3 bc = n_out.Cross(b.base - c.base);
  Vector3 ac = n_out.Cross(a.base - c.base);
  Pick d = Farthest(base, depth, valid, [&](const Vector3& p) {
    return xsimd::abs((p - b.base).Dot(bc)) + xsimd::abs((p - a.base).Dot(ac));
  });

  return Manifold{
      {a.base, b.base, c.base, d.base},
      {a.depth, b.depth, c.depth, d.depth},
      {a.valid, b.valid, c.valid, d.valid},
      any
  };
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
    Vector3* normal_out,
    ContactDistances* distance_out,
    ContactPoints* point_out,
    ContactActiveMask* active_out
) const {
  Box target(
      x_target.Translation(), x_target.Rotation().ToRotationMatrix(), h_target
  );
  Box source(
      x_source.Translation(), x_source.Rotation().ToRotationMatrix(), h_source
  );
  MathematicalT r_t = r_target.Value();
  MathematicalT r_s = r_source.Value();
  MathematicalT margin = m_target.Value() + m_source.Value();

  Survey survey = SurveyPair(target, source);
  const FaceAxis& face = survey.face;
  const EdgeAxis& edge = survey.edge;

  // Disjoint inner boxes: their exact gap and its direction
  MathematicalT gap = xsimd::sqrt(survey.closest.gap_sq);
  Vector3 gap_normal = (survey.closest.on_target - survey.closest.on_source) /
                       xsimd::max(gap, MathematicalT(kGapEpsilon));
  Mask separated = (xsimd::max(face.depth, edge.depth) > MathematicalT(0)) &
                   (gap > MathematicalT(kGapEpsilon));

  // Face preference: an edge wins by the bias, then a near-parallel face
  // wins back unless the edge is 5% better
  Mask edge_wins =
      edge.depth - MathematicalT(kEdgeBias) * xsimd::abs(edge.depth) >
      face.depth;
  MathematicalT face_slack =
      MathematicalT(kFaceEdgeAdvantage) * xsimd::abs(face.depth) +
      MathematicalT(kMinVal);
  Mask face_back = (xsimd::abs(face.axis.Dot(edge.axis)) >
                    MathematicalT(kFaceEdgeParallel)) &
                   (edge.depth < face.depth + face_slack);
  Mask face_wins = !edge_wins | face_back;

  // One contact otherwise: the exact closest pair, or the edge-edge one
  Vector3 single_n = Vector3::Select(separated, gap_normal, edge.axis);
  MathematicalT single_d = util::Select(separated, gap, edge.depth) - r_t - r_s;
  Vector3 single_a =
      Vector3::Select(separated, survey.closest.on_source, edge.on_source);
  Vector3 single_point =
      single_a + single_n * (r_s + single_d * MathematicalT(0.5F));

  // Face-on: a face won, and the inner boxes intersect or meet along it.
  // A manifold point sits on the reference face's plane; its contact is
  // midway between the surfaces, r_ref + d / 2 out along n_out
  Vector3 n_out = Vector3::Select(face.on_source, face.axis, -face.axis);
  MathematicalT r_ref = util::Select(face.on_source, r_s, r_t);
  Manifold manifold = FaceManifold(
      face,
      single_point,
      single_d,
      separated,
      target,
      source,
      r_ref,
      r_t + r_s,
      margin
  );
  Mask meets_face =
      xsimd::abs(gap_normal.Dot(face.axis)) > MathematicalT(kFaceOnAlignment);
  Mask face_on = face_wins & manifold.any & (!separated | meets_face);
  auto manifold_point = [&](std::size_t k) {
    return manifold.base[k] +
           n_out * (r_ref + manifold.depth[k] * MathematicalT(0.5F));
  };

  // n = the face's normal, or the one contact's
  *normal_out = Vector3::Select(face_on, face.axis, single_n);

  // Slot 0 is the manifold's first pick when face-on, or the one contact;
  // slots 1.. are the manifold's other picks, only when face-on
  std::array<Vector3, kSweptBoxContactSlots> point{};
  std::array<MathematicalT, kSweptBoxContactSlots> depth{};
  std::array<Mask, kSweptBoxContactSlots> active{};
  point[0] = Vector3::Select(face_on, manifold_point(0), single_point);
  depth[0] = util::Select(face_on, manifold.depth[0], single_d);
  active[0] = (face_on & manifold.valid[0]) | (!face_on & (single_d <= margin));
  for (std::size_t k = 1; k < kSweptBoxContactSlots; ++k) {
    point[k] = manifold_point(k);
    depth[k] = manifold.depth[k];
    active[k] = face_on & manifold.valid[k];
  }

  // Drop any slot that coincides with an earlier active one -- a
  // degenerate face's repeated picks
  for (std::size_t k = 1; k < kSweptBoxContactSlots; ++k) {
    for (std::size_t j = 0; j < k; ++j) {
      Mask same =
          (point[k] - point[j]).SquaredNorm() <= MathematicalT(kDuplicateSq);
      active[k] = active[k] & !(active[j] & same);
    }
  }

  ContactDistances distances;
  ContactPoints points;
  std::uint32_t bits = 0;
  for (std::size_t k = 0; k < kSweptBoxContactSlots; ++k) {
    distances(k, 0) = depth[k];
    points(0, k) = point[k].X();
    points(1, k) = point[k].Y();
    points(2, k) = point[k].Z();
    std::uint64_t lanes =
        active[k].mask() & static_cast<std::uint32_t>(candidate.AsStorage());
    bits |= static_cast<std::uint32_t>(lanes << (k * kLanes));
  }

  *distance_out = distances;
  *point_out = points;
  *active_out = ContactActiveMask(static_cast<std::int32_t>(bits));
}

}  // namespace achilles::algorithms::narrow_phase
