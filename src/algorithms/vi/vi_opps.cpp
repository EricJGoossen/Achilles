#include "algorithms/conventions.hpp"
#include "algorithms/vi/vi_ops.hpp"

namespace achilles::algorithms::vi {

void IntegrateVelocityOp::operator()(
    const Acceleration& qdd, ScalarOperationT dt, Velocity* qd_out
) const {
  *qd_out += qdd.Integrate(dt);
}

}  // namespace achilles::algorithms::vi