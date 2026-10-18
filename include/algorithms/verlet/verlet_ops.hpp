#pragma once

#include <array>

#include "algorithms/conventions.hpp"
#include "algorithms/verlet/verlet_data.hpp"
#include "engine/pass/op_invoker.hpp"

namespace achilles::algorithms::verlet {

// One velocity kick, qd += qdd * dt. Verlet runs it twice per tick, each
// time with half the tick's dt (see KickStep) -- the same update as vi's
// IntegrateVelocityOp, just never over a full dt. kJointVelocity's incoming
// value is the velocity the kick adds onto.
struct KickVelocityOp {
  using FieldEnum = VerletField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  explicit KickVelocityOp(ScalarOperationT dt) : dt_(dt) {}

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
    engine::OpLike<KickVelocityOp>, "KickVelocityOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<KickVelocityOp, VerletView>,
    "KickVelocityOp's operator() must line up with kInputs/kOutputs"
);

// The full-dt position drift between Verlet's two kicks, from the
// half-kicked velocity: composes q with Exp(S qd dt) on SE(3), the same
// update as pi's IntegratePositionOp.
struct DriftPositionOp {
  using FieldEnum = VerletField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  explicit DriftPositionOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 2> kInputs = {
      ArgData{FieldEnum::kJointSubspace, ArgIndex::kTarget},
      ArgData{FieldEnum::kJointVelocity, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{FieldEnum::kJointPosition, ArgIndex::kTarget},
  };

  void operator()(
      const MotionSubspace& S, const Velocity& qd, Transform* x_joint_out
  ) const;

 private:
  ScalarOperationT dt_;
};
static_assert(
    engine::OpLike<DriftPositionOp>,
    "DriftPositionOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<DriftPositionOp, VerletView>,
    "DriftPositionOp's operator() must line up with kInputs/kOutputs"
);

}  // namespace achilles::algorithms::verlet
