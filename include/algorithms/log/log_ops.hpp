#pragma once

#include <array>

#include "algorithms/conventions.hpp"
#include "algorithms/log/log_data.hpp"
#include "domain/for_each_scalar.hpp"
#include "engine/op_contract.hpp"
#include "util/logger.hpp"

namespace achilles::algorithms::log {

// Writes one joint's value of one LogField to a logger. Reads every
// LogField (an ordinary read-only Op: every field is an input, nothing is
// an output) and writes only `field`'s scalars, in Assembler leaf order --
// LogStep runs one pass of this per selected field, so each field's values
// for every joint land in one record.
template <util::LoggerLike LoggerT>
class LogOp {
 public:
  using FieldEnum = LogField;
  using ArgData = engine::ArgData<FieldEnum>;

  LogOp(LoggerT& logger, LogField field) : logger_(&logger), field_(field) {}

  static constexpr std::array<ArgData, kNumLogFields> kInputs = {
      ArgData{FieldEnum::kJointPosition, true},
      ArgData{FieldEnum::kJointVelocity, true},
      ArgData{FieldEnum::kJointAcceleration, true},
      ArgData{FieldEnum::kJointTorque, true},
      ArgData{FieldEnum::kWorldTransform, true},
      ArgData{FieldEnum::kSpatialVelocity, true},
      ArgData{FieldEnum::kSpatialAcceleration, true},
  };
  static constexpr std::array<ArgData, 0> kOutputs = {};

  void operator()(
      const spatial::Transform<ScalarOperationT>& joint_position,
      const spatial::SpatialVelocity<ScalarOperationT>& joint_velocity,
      const spatial::SpatialAcceleration<ScalarOperationT>& joint_acceleration,
      const spatial::SpatialForce<ScalarOperationT>& joint_torque,
      const spatial::Transform<ScalarOperationT>& world_transform,
      const spatial::SpatialVelocity<ScalarOperationT>& spatial_velocity,
      const spatial::SpatialAcceleration<ScalarOperationT>& spatial_acceleration
  ) const {
    switch (field_) {
      case LogField::kJointPosition:
        WriteValue(joint_position);
        return;
      case LogField::kJointVelocity:
        WriteValue(joint_velocity);
        return;
      case LogField::kJointAcceleration:
        WriteValue(joint_acceleration);
        return;
      case LogField::kJointTorque:
        WriteValue(joint_torque);
        return;
      case LogField::kWorldTransform:
        WriteValue(world_transform);
        return;
      case LogField::kSpatialVelocity:
        WriteValue(spatial_velocity);
        return;
      case LogField::kSpatialAcceleration:
        WriteValue(spatial_acceleration);
        return;
      case LogField::kCount:
        return;
    }
  }

 private:
  template <typename ValueT>
  void WriteValue(const ValueT& value) const {
    domain::ForEachScalar(value, [this](auto scalar) {
      logger_->Write(scalar);
    });
  }

  LoggerT* logger_;
  LogField field_;
};
static_assert(engine::OpLike<LogOp<util::NullLogger>>);
static_assert(
    engine::OpArgsMatchView<LogOp<util::NullLogger>, LogView>,
    "LogOp's operator() must line up with kInputs"
);

}  // namespace achilles::algorithms::log
