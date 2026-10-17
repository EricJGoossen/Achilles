#include "algorithms/collision/collision_ops.hpp"

#include <cmath>
#include <cstdint>
#include <xsimd/xsimd.hpp>

#include "algorithms/collision/collision_data.hpp"
#include "algorithms/conventions.hpp"
#include "engine/pass/traversals.hpp"

namespace achilles::algorithms::collision {

void ComputeWorldBoundsOp::operator()(
    const Transform& x_world,
    const Vector3& c_local,
    const Vector3& h_local,
    const Scalar& margin,
    Vector3* c_world_out,
    Vector3* h_world_out
) const {
  Matrix3x3 r = x_world.Rotation().ToRotationMatrix();

  // c_world = x_world c_local
  *c_world_out = x_world.Translation() + r * c_local;

  // h_world = |R| h_local + m
  MathematicalT m = margin.Value();
  *h_world_out = r.Abs() * h_local + Vector3(m, m, m);
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
    PairLaneMask* overlap_out
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

  *overlap_out = PairLaneMask(static_cast<std::int32_t>(
      overlap.mask() & allowed.mask() & structure.distinct_lanes
  ));
}

}  // namespace achilles::algorithms::collision
