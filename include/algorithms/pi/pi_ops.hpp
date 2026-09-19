#pragma once

#include <array>

#include "algorithms/conventions.hpp"
#include "algorithms/pi/pi_data.hpp"
#include "engine/pass/op_invoker.hpp"

namespace achilles::algorithms::pi {

// Integrates the joint's velocity into its pose by composing on the SE(3)
// manifold rather than adding into raw coordinates: qd (in the joint's own
// generalized coordinates) is lifted into a real spatial twist via S, then
// exponentiated into an incremental Transform and composed onto the
// existing pose. This is the position-side counterpart of vi's
// IntegrateVelocityOp, but composition (`*`) replaces addition because a
// pose isn't a vector space the way a velocity is -- see
// domain/spatial/transform.hpp's Exp and aba_ops.cpp's PropagateVelocityOp,
// which now consumes kJointPosition directly as x_joint with no Exp step
// of its own.
struct IntegratePositionOp {
  using FieldEnum = PIField;
  using ArgData = engine::ArgData<FieldEnum>;

  explicit IntegratePositionOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 2> kInputs = {
      ArgData{FieldEnum::kJointSubspace, true},
      ArgData{FieldEnum::kJointVelocity, true},
  };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{FieldEnum::kJointPosition, true},
  };

  void operator()(
      const MotionSubspace& S, const Velocity& qd, Transform* x_joint_out
  ) const;

 private:
  ScalarOperationT dt_;
};
static_assert(
    engine::OpLike<IntegratePositionOp>,
    "IntegratePositionOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<IntegratePositionOp, PIView>,
    "IntegratePositionOp's operator() must line up with kInputs/kOutputs"
);

}  // namespace achilles::algorithms::pi
