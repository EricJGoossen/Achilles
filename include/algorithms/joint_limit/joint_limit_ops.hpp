#pragma once

#include <array>

#include "algorithms/conventions.hpp"
#include "algorithms/joint_limit/joint_limit_data.hpp"
#include "engine/op_contract.hpp"

namespace achilles::algorithms::joint_limit {

// Every DOF's limit row (see joint_limit_data.hpp): theta = S^T Log(q), its
// distance to the nearer end of [lower, upper] and which end that is, and
// whether it's limited and within the margin. Each joint on its own, so a
// linear pass.
struct DetectJointLimitOp {
  using FieldEnum = JointLimitField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  static constexpr std::array<ArgData, 6> kInputs = {
      ArgData{FieldEnum::kJointSubspace, ArgIndex::kTarget},
      ArgData{FieldEnum::kJointPosition, ArgIndex::kTarget},
      ArgData{FieldEnum::kJointLimited, ArgIndex::kTarget},
      ArgData{FieldEnum::kJointRangeLower, ArgIndex::kTarget},
      ArgData{FieldEnum::kJointRangeUpper, ArgIndex::kTarget},
      ArgData{FieldEnum::kJointLimitMargin, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 3> kOutputs = {
      ArgData{FieldEnum::kLimitDistance, ArgIndex::kTarget},
      ArgData{FieldEnum::kLimitSide, ArgIndex::kTarget},
      ArgData{FieldEnum::kLimitActive, ArgIndex::kTarget},
  };

  void operator()(
      const MotionSubspace& S,
      const Transform& q,
      const Mat6Mask& limited,
      const JointCoordinates& lower,
      const JointCoordinates& upper,
      const Scalar& margin,
      JointCoordinates* distance_out,
      JointCoordinates* side_out,
      Mat6Mask* active_out
  ) const;
};
static_assert(
    engine::OpLike<DetectJointLimitOp>,
    "DetectJointLimitOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<DetectJointLimitOp, JointLimitView>,
    "DetectJointLimitOp's operator() must line up with kInputs/kOutputs"
);

}  // namespace achilles::algorithms::joint_limit
