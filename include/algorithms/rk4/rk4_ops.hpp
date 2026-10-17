#pragma once

#include <array>

#include "algorithms/conventions.hpp"
#include "algorithms/rk4/rk4_data.hpp"
#include "engine/pass/op_invoker.hpp"

namespace achilles::algorithms::rk4 {

// Snapshots the tick's starting state into the scratch fields and zeroes
// both stage sums. The current (q, qd) is left alone: it's stage 1's state.
struct BeginStepOp {
  using FieldEnum = RK4Field;
  using ArgData = engine::ArgData<FieldEnum>;

  static constexpr std::array<ArgData, 2> kInputs = {
      ArgData{FieldEnum::kJointPosition, true},
      ArgData{FieldEnum::kJointVelocity, true},
  };
  static constexpr std::array<ArgData, 4> kOutputs = {
      ArgData{FieldEnum::kStartPosition, true},
      ArgData{FieldEnum::kStartVelocity, true},
      ArgData{FieldEnum::kVelocitySum, true},
      ArgData{FieldEnum::kAccelerationSum, true},
  };

  void operator()(
      const Transform& q,
      const Velocity& qd,
      Transform* q_start_out,
      Velocity* qd_start_out,
      Velocity* qd_sum_out,
      Acceleration* qdd_sum_out
  ) const;
};
static_assert(
    engine::OpLike<BeginStepOp>, "BeginStepOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<BeginStepOp, RK4View>,
    "BeginStepOp's operator() must line up with kInputs/kOutputs"
);

// Folds the stage just evaluated into both sums with RK4's weight for it
// (1, 2, 2, 1): k_v is the stage's own qd (still in kJointVelocity), k_a
// the qdd ABA just solved for at that state. The sums' incoming values are
// the running totals this adds onto.
struct AccumulateStageOp {
  using FieldEnum = RK4Field;
  using ArgData = engine::ArgData<FieldEnum>;

  explicit AccumulateStageOp(ScalarOperationT weight) : weight_(weight) {}

  static constexpr std::array<ArgData, 2> kInputs = {
      ArgData{FieldEnum::kJointVelocity, true},
      ArgData{FieldEnum::kJointAcceleration, true},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kVelocitySum, true},
      ArgData{FieldEnum::kAccelerationSum, true},
  };

  void operator()(
      const Velocity& qd,
      const Acceleration& qdd,
      Velocity* qd_sum_out,
      Acceleration* qdd_sum_out
  ) const;

 private:
  ScalarOperationT weight_;
};
static_assert(
    engine::OpLike<AccumulateStageOp>,
    "AccumulateStageOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<AccumulateStageOp, RK4View>,
    "AccumulateStageOp's operator() must line up with kInputs/kOutputs"
);

// Writes the next stage's state from the previous stage's derivatives,
// advanced from the tick's start over h = `dt` (dt/2 for stages 2 and 3,
// the full dt for stage 4):
//     q  = q_n * Exp(S k_v h),   qd = qd_n + k_a h
// k_v is the previous stage's qd -- kJointVelocity's incoming value, read
// before this overwrites it; k_a is ABA's qdd for that stage.
//
// The position composes on SE(3) rather than adding, the same way pi's
// IntegratePositionOp does -- see rk4_step.hpp's comment on what that does
// to the fourth-order claim for multi-axis joints.
struct AdvanceStageOp {
  using FieldEnum = RK4Field;
  using ArgData = engine::ArgData<FieldEnum>;

  explicit AdvanceStageOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 4> kInputs = {
      ArgData{FieldEnum::kJointSubspace, true},
      ArgData{FieldEnum::kStartPosition, true},
      ArgData{FieldEnum::kStartVelocity, true},
      ArgData{FieldEnum::kJointAcceleration, true},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kJointPosition, true},
      ArgData{FieldEnum::kJointVelocity, true},
  };

  void operator()(
      const MotionSubspace& S,
      const Transform& q_start,
      const Velocity& qd_start,
      const Acceleration& qdd,
      Transform* q_out,
      Velocity* qd_out
  ) const;

 private:
  ScalarOperationT dt_;
};
static_assert(
    engine::OpLike<AdvanceStageOp>, "AdvanceStageOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<AdvanceStageOp, RK4View>,
    "AdvanceStageOp's operator() must line up with kInputs/kOutputs"
);

// Commits the blended step:
//     qd_{n+1} = qd_n + dt/6 (k_a1 + 2 k_a2 + 2 k_a3 + k_a4)
//     q_{n+1}  = q_n * Exp(S dt/6 (k_v1 + 2 k_v2 + 2 k_v3 + k_v4))
// -- one exponential of the blended twist, not four composed ones.
struct CommitOp {
  using FieldEnum = RK4Field;
  using ArgData = engine::ArgData<FieldEnum>;

  explicit CommitOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 5> kInputs = {
      ArgData{FieldEnum::kJointSubspace, true},
      ArgData{FieldEnum::kStartPosition, true},
      ArgData{FieldEnum::kStartVelocity, true},
      ArgData{FieldEnum::kVelocitySum, true},
      ArgData{FieldEnum::kAccelerationSum, true},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kJointPosition, true},
      ArgData{FieldEnum::kJointVelocity, true},
  };

  void operator()(
      const MotionSubspace& S,
      const Transform& q_start,
      const Velocity& qd_start,
      const Velocity& qd_sum,
      const Acceleration& qdd_sum,
      Transform* q_out,
      Velocity* qd_out
  ) const;

 private:
  ScalarOperationT dt_;
};
static_assert(engine::OpLike<CommitOp>, "CommitOp must satisfy OpLike concept");
static_assert(
    engine::OpArgsMatchView<CommitOp, RK4View>,
    "CommitOp's operator() must line up with kInputs/kOutputs"
);

}  // namespace achilles::algorithms::rk4
