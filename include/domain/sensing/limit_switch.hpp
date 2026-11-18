#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string_view>

#include "domain/error/effects.hpp"
#include "domain/math/vector3.hpp"
#include "domain/random/counter_rng.hpp"
#include "domain/sensing/sensor.hpp"

namespace achilles::domain::sensing {

// One switch sample: whether it's pressed, and the joint position it
// thresholded (radians or meters, error included) -- for a plot.
struct SwitchReading {
  float position = 0.0F;
  bool pressed = false;
};

// What a switch remembers: its position error's state, and whether it was
// pressed last sample (for hysteresis).
struct SwitchState {
  domain::error::ErrorChain<float>::State error;
  bool pressed = false;
};

// A limit switch or beam break on its mount joint: pressed while the joint's
// position along one axis of its own frame -- the rotation about it (rotary)
// or the translation along it (linear) -- is at or past a threshold:
// `press_above` (pressed at >= it) and/or `press_below` (pressed at <= it).
// Once pressed it releases only `hysteresis` back past the threshold, as a
// real switch's travel does. The position goes through an error model first
// (prefix position_: bias for a switch mounted off its nominal point, noise
// for chatter, latency for a debounce delay). `normally_closed` inverts what
// it reports, for a switch wired that way.
class LimitSwitch : public Sensor<
                        LimitSwitch,
                        SimInputs<input::JointPosition>,
                        SwitchState,
                        SwitchReading> {
 public:
  using Position = input::JointPosition::Value;

  static constexpr std::string_view kListName = "switch";

  struct Params {
    domain::math::Vector3<float> axis = domain::math::Vector3<float>::UnitX();
    bool linear = false;
    bool has_above = false;
    float press_above = 0.0F;
    bool has_below = false;
    float press_below = 0.0F;
    float hysteresis = 0.0F;
    bool normally_closed = false;
    domain::error::ErrorChain<float>::Params position_error;
  };

  LimitSwitch() = default;
  // Throws std::invalid_argument for a zero axis, no threshold, or a
  // negative hysteresis.
  LimitSwitch(AttachmentOptions options, const Params& params)
      : Sensor(options), params_(params) {
    if (params_.axis.Norm() <= 0.0F) {
      throw std::invalid_argument("LimitSwitch: the axis must be nonzero.");
    }
    if (!params_.has_above && !params_.has_below) {
      throw std::invalid_argument(
          "LimitSwitch: needs press_above, press_below, or both."
      );
    }
    if (params_.hysteresis < 0.0F) {
      throw std::invalid_argument(
          "LimitSwitch: hysteresis must not be negative."
      );
    }
    params_.axis = params_.axis.Normalize();
  }

  // Settings: period, axis, linear, press_above, press_below, hysteresis,
  // normally_closed, and the position error model (prefix position_).
  static LimitSwitch FromConfig(AttachmentConfig& config) {
    const AttachmentOptions options = config.Options();
    Params params;
    params.axis = config.Vector("axis", params.axis);
    params.linear = config.Flag("linear", false);
    // A threshold that isn't given reads back as NaN, which no position
    // is at or past.
    constexpr float kUnset = std::numeric_limits<float>::quiet_NaN();
    params.press_above = config.Scalar("press_above", kUnset);
    params.press_below = config.Scalar("press_below", kUnset);
    params.has_above = !std::isnan(params.press_above);
    params.has_below = !std::isnan(params.press_below);
    params.hysteresis = config.Scalar("hysteresis", 0.0F);
    params.normally_closed = config.Flag("normally_closed", false);
    params.position_error = config.ErrorModel<float>("position_");
    return {options, params};
  }

  void Read(
      const AttachmentTick& tick,
      const Position& q,
      SwitchState* state,
      SwitchReading* out
  ) const {
    const float truth = params_.linear ? q.Translation().Dot(params_.axis)
                                       : q.Rotation().AngleAbout(params_.axis);
    domain::random::CounterRng rng(tick.stream, tick.tick);
    const float position = domain::error::ErrorChain<float>::Apply(
        params_.position_error, state->error, rng, tick.dt, truth
    );
    // Pressed past a threshold; held until `hysteresis` back from it.
    const float slack = state->pressed ? params_.hysteresis : 0.0F;
    const bool pressed =
        (params_.has_above && position >= params_.press_above - slack) ||
        (params_.has_below && position <= params_.press_below + slack);
    state->pressed = pressed;
    out->position = position;
    out->pressed = pressed != params_.normally_closed;
  }

 private:
  Params params_;
};

}  // namespace achilles::domain::sensing
