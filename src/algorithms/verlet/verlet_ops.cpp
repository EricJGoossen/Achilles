#include "algorithms/verlet/verlet_ops.hpp"

#include "algorithms/conventions.hpp"

namespace achilles::algorithms::verlet {

void KickVelocityOp::operator()(const Acceleration& qdd, Velocity* qd_out)
    const {
  *qd_out += qdd.Integrate(dt_);
}

void DriftPositionOp::operator()(
    const MotionSubspace& S, const Velocity& qd, Transform* x_joint_out
) const {
  Velocity qd_spatial = S * qd;
  *x_joint_out *= Transform::Exp(qd_spatial * dt_);
  // See Transform::NormalizeRotationInPlace's own comment -- this
  // composition accumulates across ticks.
  x_joint_out->NormalizeRotationInPlace();
}

}  // namespace achilles::algorithms::verlet
