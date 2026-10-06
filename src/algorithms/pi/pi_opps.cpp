#include "algorithms/conventions.hpp"
#include "algorithms/pi/pi_ops.hpp"

namespace achilles::algorithms::pi {

void IntegratePositionOp::operator()(
    const MotionSubspace& S, const Velocity& qd, Transform* x_joint_out
) const {
  Velocity qd_spatial = S * qd;
  *x_joint_out *= Transform::Exp(qd_spatial * dt_);
  // See Transform::NormalizeRotationInPlace's own comment -- this is the
  // one place a joint's own rotation accumulates via repeated composition
  // across ticks, so it's the one place drift can actually build up.
  x_joint_out->NormalizeRotationInPlace();
}

}  // namespace achilles::algorithms::pi
