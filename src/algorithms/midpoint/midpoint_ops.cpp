#include "algorithms/midpoint/midpoint_ops.hpp"

#include "algorithms/conventions.hpp"

namespace achilles::algorithms::midpoint {

void BeginStepOp::operator()(
    const Transform& q,
    const Velocity& qd,
    Transform* q_start_out,
    Velocity* qd_start_out,
    Velocity* qd_next_out
) const {
  *q_start_out = q;
  *qd_start_out = qd;
  *qd_next_out = qd;
}

void MidpointTrialOp::operator()(
    const MotionSubspace& S,
    const Transform& q_start,
    const Velocity& qd_start,
    const Velocity& qd_next,
    Transform* q_out,
    Velocity* qd_out
) const {
  Velocity qd_mid = (qd_start + qd_next) * ScalarOperationT{0.5};
  Velocity qd_spatial = S * qd_mid;
  *q_out = q_start * Transform::Exp(qd_spatial * (dt_ * ScalarOperationT{0.5}));
  *qd_out = qd_mid;
}

void UpdateNextVelocityOp::operator()(
    const Velocity& qd_start, const Acceleration& qdd_mid, Velocity* qd_next_out
) const {
  *qd_next_out = qd_start + qdd_mid.Integrate(dt_);
}

void CommitOp::operator()(
    const MotionSubspace& S,
    const Transform& q_start,
    const Velocity& qd_start,
    const Velocity& qd_next,
    Transform* q_out,
    Velocity* qd_out
) const {
  Velocity qd_mid = (qd_start + qd_next) * ScalarOperationT{0.5};
  Velocity qd_spatial = S * qd_mid;
  Transform q_next = q_start * Transform::Exp(qd_spatial * dt_);
  // See Transform::NormalizeRotationInPlace's own comment -- this is the
  // cross-tick-accumulating commit, unlike MidpointTrialOp's trials.
  q_next.NormalizeRotationInPlace();
  *q_out = q_next;
  *qd_out = qd_next;
}

}  // namespace achilles::algorithms::midpoint
