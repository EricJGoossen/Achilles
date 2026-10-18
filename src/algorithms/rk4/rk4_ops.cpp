#include "algorithms/rk4/rk4_ops.hpp"

#include "algorithms/conventions.hpp"

namespace achilles::algorithms::rk4 {

void BeginStepOp::operator()(
    const Transform& q,
    const Velocity& qd,
    Transform* q_start_out,
    Velocity* qd_start_out,
    Velocity* qd_sum_out,
    Acceleration* qdd_sum_out
) const {
  *q_start_out = q;
  *qd_start_out = qd;
  *qd_sum_out = Velocity::Zero();
  *qdd_sum_out = Acceleration::Zero();
}

void AccumulateStageOp::operator()(
    const Velocity& qd,
    const Acceleration& qdd,
    Velocity* qd_sum_out,
    Acceleration* qdd_sum_out
) const {
  *qd_sum_out += qd * weight_;
  *qdd_sum_out += qdd * weight_;
}

void AdvanceStageOp::operator()(
    const MotionSubspace& S,
    const Transform& q_start,
    const Velocity& qd_start,
    const Acceleration& qdd,
    Transform* q_out,
    Velocity* qd_out
) const {
  Velocity qd_spatial = S * *qd_out;
  *q_out = q_start * Transform::Exp(qd_spatial * dt_);
  *qd_out = qd_start + qdd.Integrate(dt_);
}

void CommitOp::operator()(
    const MotionSubspace& S,
    const Transform& q_start,
    const Velocity& qd_start,
    const Velocity& qd_sum,
    const Acceleration& qdd_sum,
    Transform* q_out,
    Velocity* qd_out
) const {
  const ScalarOperationT sixth_dt = dt_ / ScalarOperationT{6};
  Velocity qd_spatial = S * qd_sum;
  Transform q_next = q_start * Transform::Exp(qd_spatial * sixth_dt);
  // See Transform::NormalizeRotationInPlace's own comment -- this is the
  // cross-tick-accumulating commit, unlike AdvanceStageOp's stage states.
  q_next.NormalizeRotationInPlace();
  *q_out = q_next;
  *qd_out = qd_start + qdd_sum.Integrate(sixth_dt);
}

}  // namespace achilles::algorithms::rk4
