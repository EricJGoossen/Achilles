#pragma once

#include <string>
#include <string_view>
#include <tuple>

#include "display/display.hpp"
#include "display/frame.hpp"
#include "display/input.hpp"
#include "domain/actuation/actuator.hpp"
#include "domain/actuation/truth_actuator.hpp"
#include "util/tmp.hpp"

namespace achilles::display {

// Drives one actuator by hand: a slider from -max_effort to max_effort (N
// for a linear actuator, N*m for an angular one) that springs back to 0 when
// let go, and the effort the joint last got. Only an effort -- no target,
// no loop -- so a free joint keeps the speed it was given once it's let go.
template <EffortActuator K>
struct EffortControl {
  using Attachments = util::TypeList<K>;
  static constexpr bool kAngular =
      K::kEffort == domain::actuation::EffortKind::kAngular;
  static constexpr std::string_view kName = kAngular ? "torque" : "force";
  static constexpr std::string_view kUnit = kAngular ? " N*m" : " N";
  static constexpr float kWidth = 380.0F;
  static constexpr float kHeight = 110.0F;

  // A display's settings: public on purpose (see display.hpp).
  float max_effort = 5.0F;

  void Draw(const HandlesForT<Attachments>& handles, Frame* frame) const {
    frame->sliders.push_back(
        {.label = kAngular ? "torque (N*m)" : "force (N)",
         .min = -max_effort,
         .max = max_effort}
    );
    frame->text.push_back(
        "last applied: " + Fixed(std::get<0>(handles).Read().value.effort) +
        std::string(kUnit)
    );
  }

  EffortsForT<Attachments> Efforts(const Input& input) const {
    return {input.Value(0)};
  }
};

using TorqueControl = EffortControl<domain::actuation::truth::Torque>;
using ForceControl = EffortControl<domain::actuation::truth::Force>;

}  // namespace achilles::display
