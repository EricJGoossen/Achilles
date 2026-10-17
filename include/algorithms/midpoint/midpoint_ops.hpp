#pragma once

#include <array>

#include "algorithms/conventions.hpp"
#include "algorithms/midpoint/midpoint_data.hpp"
#include "engine/pass/op_invoker.hpp"

namespace achilles::algorithms::midpoint {

// Snapshots the tick's starting state into the scratch fields and seeds
// the fixed-point estimate qd_{n+1} = qd_n.
struct BeginStepOp {
  using FieldEnum = MidpointField;
  using ArgData = engine::ArgData<FieldEnum>;

  static constexpr std::array<ArgData, 2> kInputs = {
      ArgData{FieldEnum::kJointPosition, true},
      ArgData{FieldEnum::kJointVelocity, true},
  };
  static constexpr std::array<ArgData, 3> kOutputs = {
      ArgData{FieldEnum::kStartPosition, true},
      ArgData{FieldEnum::kStartVelocity, true},
      ArgData{FieldEnum::kNextVelocity, true},
  };

  void operator()(
      const Transform& q,
      const Velocity& qd,
      Transform* q_start_out,
      Velocity* qd_start_out,
      Velocity* qd_next_out
  ) const;
};
static_assert(
    engine::OpLike<BeginStepOp>, "BeginStepOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<BeginStepOp, MidpointView>,
    "BeginStepOp's operator() must line up with kInputs/kOutputs"
);

// Writes the midpoint trial for the current estimate into the fields ABA
// reads: qd_mid = (qd_n + qd_{n+1}) / 2, q_mid = q_n * Exp(S qd_mid dt/2).
// The trial q_mid is discarded within the tick, so it isn't renormalized.
struct MidpointTrialOp {
  using FieldEnum = MidpointField;
  using ArgData = engine::ArgData<FieldEnum>;

  explicit MidpointTrialOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 4> kInputs = {
      ArgData{FieldEnum::kJointSubspace, true},
      ArgData{FieldEnum::kStartPosition, true},
      ArgData{FieldEnum::kStartVelocity, true},
      ArgData{FieldEnum::kNextVelocity, true},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kJointPosition, true},
      ArgData{FieldEnum::kJointVelocity, true},
  };

  void operator()(
      const MotionSubspace& S,
      const Transform& q_start,
      const Velocity& qd_start,
      const Velocity& qd_next,
      Transform* q_out,
      Velocity* qd_out
  ) const;

 private:
  ScalarOperationT dt_;
};
static_assert(
    engine::OpLike<MidpointTrialOp>,
    "MidpointTrialOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<MidpointTrialOp, MidpointView>,
    "MidpointTrialOp's operator() must line up with kInputs/kOutputs"
);

// One fixed-point update from ABA's acceleration at the midpoint trial:
// qd_{n+1} = qd_n + qdd_mid dt.
struct UpdateNextVelocityOp {
  using FieldEnum = MidpointField;
  using ArgData = engine::ArgData<FieldEnum>;

  explicit UpdateNextVelocityOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 2> kInputs = {
      ArgData{FieldEnum::kStartVelocity, true},
      ArgData{FieldEnum::kJointAcceleration, true},
  };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{FieldEnum::kNextVelocity, true},
  };

  void operator()(
      const Velocity& qd_start,
      const Acceleration& qdd_mid,
      Velocity* qd_next_out
  ) const;

 private:
  ScalarOperationT dt_;
};
static_assert(
    engine::OpLike<UpdateNextVelocityOp>,
    "UpdateNextVelocityOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<UpdateNextVelocityOp, MidpointView>,
    "UpdateNextVelocityOp's operator() must line up with kInputs/kOutputs"
);

// Commits the converged result: q_{n+1} = q_n * Exp(S qd_mid dt) -- the
// FULL-dt exponential of the converged midpoint velocity, not the half-dt
// one each trial uses -- and qd_{n+1} as estimated.
struct CommitOp {
  using FieldEnum = MidpointField;
  using ArgData = engine::ArgData<FieldEnum>;

  explicit CommitOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 4> kInputs = {
      ArgData{FieldEnum::kJointSubspace, true},
      ArgData{FieldEnum::kStartPosition, true},
      ArgData{FieldEnum::kStartVelocity, true},
      ArgData{FieldEnum::kNextVelocity, true},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kJointPosition, true},
      ArgData{FieldEnum::kJointVelocity, true},
  };

  void operator()(
      const MotionSubspace& S,
      const Transform& q_start,
      const Velocity& qd_start,
      const Velocity& qd_next,
      Transform* q_out,
      Velocity* qd_out
  ) const;

 private:
  ScalarOperationT dt_;
};
static_assert(engine::OpLike<CommitOp>, "CommitOp must satisfy OpLike concept");
static_assert(
    engine::OpArgsMatchView<CommitOp, MidpointView>,
    "CommitOp's operator() must line up with kInputs/kOutputs"
);

}  // namespace achilles::algorithms::midpoint
