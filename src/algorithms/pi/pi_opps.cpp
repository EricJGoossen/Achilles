#include "algorithms/conventions.hpp"
#include "algorithms/pi/pi_ops.hpp"

namespace achilles::algorithms::pi {

void IntegratePositionOp::operator()(
    const MotionSubspace& S, const Velocity& qd, Transform* x_joint_out
) const {
  Velocity qd_spatial = S * qd;
  *x_joint_out *= Transform::Exp(qd_spatial * dt_);
}

}  // namespace achilles::algorithms::pi
