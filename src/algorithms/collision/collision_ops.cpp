#include "algorithms/collision/collision_ops.hpp"

#include "algorithms/conventions.hpp"

namespace achilles::algorithms::collision {

void ComputeWorldBoundsOp::operator()(
    const Transform& x_world,
    const Vector3& c_local,
    const Vector3& h_local,
    Vector3* c_world_out,
    Vector3* h_world_out
) const {
  Matrix3x3 r = x_world.Rotation().ToRotationMatrix();

  // c_world = x_world c_local
  *c_world_out = x_world.Translation() + r * c_local;

  // h_world = |R| h_local
  *h_world_out = r.Abs() * h_local;
}

}  // namespace achilles::algorithms::collision
