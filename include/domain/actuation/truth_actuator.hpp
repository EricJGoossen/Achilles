#pragma once

#include <string_view>

#include "domain/actuation/actuator.hpp"

namespace achilles::domain::actuation {

// A one-tick effort: what Apply() adds to, and the tick uses up.
struct EffortCommand {
  float effort = 0.0F;
};

// The effort the joint got.
struct EffortOutput {
  float effort = 0.0F;
};

// An ideal actuator: puts its command on the joint exactly, with no error --
// the mirror of a truth sensor, for a controller that's allowed to cheat or for
// checking a real actuator model against. The effort commanded since the last
// tick is the effort of that tick alone.
template <EffortKind E>
class TruthActuator : public Actuator<
                          TruthActuator<E>,
                          SimInputs<>,
                          NoState,
                          EffortCommand,
                          EffortOutput> {
  using Base = Actuator<
      TruthActuator<E>,
      SimInputs<>,
      NoState,
      EffortCommand,
      EffortOutput>;

 public:
  static constexpr EffortKind kEffort = E;
  static constexpr std::string_view kListName =
      E == EffortKind::kAngular ? "angular" : "linear";

  TruthActuator() = default;
  TruthActuator(
      AttachmentOptions options, const domain::math::Vector3<float>& axis
  )
      : Base(options, axis) {}

  // Takes only what every actuator has: period, and axis, which is required.
  static TruthActuator FromConfig(AttachmentConfig& config) {
    const AttachmentOptions options = config.Options();
    return {options, config.RequiredVector("axis")};
  }

  void Drive(
      const AttachmentTick& /*tick*/,
      EffortCommand* command,
      NoState* /*state*/,
      EffortOutput* out
  ) const {
    out->effort = command->effort;
    command->effort = 0.0F;
  }
};

// The truth actuators, by what they push with.
namespace truth {

using Torque = TruthActuator<EffortKind::kAngular>;
using Force = TruthActuator<EffortKind::kLinear>;

}  // namespace truth

}  // namespace achilles::domain::actuation
