#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/vi/vi_data.hpp"
#include "engine/op_invoker/op_initializer.hpp"
#include "engine/op_invoker/op_invoker.hpp"

namespace achilles::algorithms::vi {

struct IntegrateVelocityOp {
  using FieldEnum = VIField;
  using ArgData = engine::spec::ArgData<FieldEnum>;
  using ArgIndex = engine::spec::ArgIndex;

  explicit IntegrateVelocityOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 1> kInputs = {
      ArgData{FieldEnum::kJointAcceleration, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{FieldEnum::kJointVelocity, ArgIndex::kTarget},
  };

  void operator()(const Acceleration& qdd, Velocity* qd_out) const;

 private:
  ScalarOperationT dt_;
};
static_assert(
    engine::spec::OpLike<IntegrateVelocityOp>,
    "IntegrateVelocityOp must satisfy OpLike concept"
);
static_assert(
    engine::spec::OpArgsMatchView<IntegrateVelocityOp, VIView>,
    "IntegrateVelocityOp's operator() must line up with kInputs/kOutputs"
);

}  // namespace achilles::algorithms::vi