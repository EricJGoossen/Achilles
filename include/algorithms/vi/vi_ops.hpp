#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/vi/vi_data.hpp"
#include "engine/pass/op_invoker.hpp"

namespace achilles::algorithms::vi {

struct IntegrateVelocityOp {
  using FieldEnum = VIField;
  using ArgData = engine::ArgData<FieldEnum>;

  explicit IntegrateVelocityOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 1> kInputs = {
      ArgData{FieldEnum::kJointAcceleration, true},
  };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{FieldEnum::kJointVelocity, true},
  };

  void operator()(const Acceleration& qdd, Velocity* qd_out) const;

 private:
  ScalarOperationT dt_;
};
static_assert(
    engine::OpLike<IntegrateVelocityOp>,
    "IntegrateVelocityOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<IntegrateVelocityOp, VIView>,
    "IntegrateVelocityOp's operator() must line up with kInputs/kOutputs"
);

}  // namespace achilles::algorithms::vi