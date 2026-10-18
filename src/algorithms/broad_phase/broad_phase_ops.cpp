#include "algorithms/broad_phase/broad_phase_ops.hpp"

#include <cmath>
#include <cstdint>
#include <xsimd/xsimd.hpp>

#include "algorithms/broad_phase/broad_phase_data.hpp"
#include "algorithms/conventions.hpp"
#include "engine/pass/traversals.hpp"
#include "util/simd_ops.hpp"

namespace achilles::algorithms::broad_phase {

void ComputeSweptBoxBoundsOp::operator()(
    const Transform& x_world,
    const Transform& x_shape,
    const Vector3& h,
    const Scalar& radius,
    const Scalar& margin,
    Transform* x_world_shape_out,
    Vector3* c_world_out,
    Vector3* h_world_out
) const {
  // q_shape, read as identity when unset (zero)
  const Quaternion& q = x_shape.Rotation();
  MathematicalT unset = util::Select(
      q.SquaredNorm() == MathematicalT(0), MathematicalT(1), MathematicalT(0)
  );
  Quaternion q_shape(q.W() + unset, q.X(), q.Y(), q.Z());

  // x = x_world x_shape
  Transform x = x_world * Transform(x_shape.Translation(), q_shape);
  *x_world_shape_out = x;

  // c_world = x t_shape
  *c_world_out = x.Translation();

  // h_world = |R| h + (r + m), R = R_world R_shape
  Matrix3x3 r = x.Rotation().ToRotationMatrix();
  MathematicalT pad = (radius + margin).Value();
  *h_world_out = r.Abs() * h + Vector3(pad, pad, pad);
}

void TestBoundsOverlapOp::operator()(
    const Vector3& c_target,
    const Vector3& h_target,
    const CollisionGroupMask& group_target,
    const CollisionGroupMask& mask_target,
    const Vector3& c_source,
    const Vector3& h_source,
    const CollisionGroupMask& group_source,
    const CollisionGroupMask& mask_source,
    const engine::pass::PairStructure& structure,
    LaneMask* overlap_out
) const {
  using xsimd::abs;

  // |c_t - c_s| <= h_t + h_s, per axis
  Vector3 d = c_target - c_source;
  Vector3 r = h_target + h_source;
  auto overlap =
      (abs(d.X()) <= r.X()) & (abs(d.Y()) <= r.Y()) & (abs(d.Z()) <= r.Z());

  // (group_t & mask_s) != 0 && (group_s & mask_t) != 0
  auto allowed = !(group_target & mask_source).AllFalse() &
                 !(group_source & mask_target).AllFalse();

  *overlap_out = LaneMask(static_cast<std::int32_t>(
      overlap.mask() & allowed.mask() & structure.distinct_lanes
  ));
}

}  // namespace achilles::algorithms::broad_phase
