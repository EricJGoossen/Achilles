#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string_view>

#include "domain/actuation/actuator.hpp"
#include "domain/error/effects.hpp"
#include "domain/random/counter_rng.hpp"

namespace achilles::domain::actuation {

// What a motor's driver is told, held until the next command.
struct DcMotorCommand {
  enum class Mode : std::uint8_t { kCurrent, kVoltage };

  static DcMotorCommand Current(float amps) { return {Mode::kCurrent, amps}; }
  static DcMotorCommand Voltage(float volts) { return {Mode::kVoltage, volts}; }

  Mode mode = Mode::kCurrent;
  float value = 0.0F;
};

struct DcMotorState {
  domain::error::SpeedCurve::State speed_curve;
  domain::error::Saturate<float>::State current_limit;
  domain::error::LossChain::State losses;
  domain::error::ErrorChain<float>::State error;
};

// One update: the torque the joint got, and what a controller's current
// feedback reports.
struct DcMotorOutput {
  float effort = 0.0F;   // N*m, at the joint
  float current = 0.0F;  // A, in the winding
};

// A DC motor driving its joint through a gearbox: a brushed motor, or a
// brushless one behind a driver whose current loop makes it behave like one.
//
// What's the motor's own is how a command becomes a current: a current command
// is tracked instantly, and a voltage command sets the current through the
// winding resistance, less the back-EMF. Kt turns the current into rotor torque
// and the gear ratio carries it to the joint. Everything else is an effect: the
// speed curve the bus voltage allows, the driver's current limit, the
// mechanism's losses, and any error on the torque the joint gets.
//
// Every constant is at the rotor, as datasheets give them. `gear_ratio` is
// output turns per rotor turn, so 19:1 is 1/19: the joint turns gear_ratio
// times as fast and gets 1/gear_ratio times the torque.
class DcMotor : public Actuator<
                    DcMotor,
                    SimInputs<input::JointVelocity, input::JointSubspace>,
                    DcMotorState,
                    DcMotorCommand,
                    DcMotorOutput> {
 public:
  using Velocity = input::JointVelocity::Value;
  using Subspace = input::JointSubspace::Value;
  using ErrorModel = domain::error::ErrorChain<float>;

  static constexpr EffortKind kEffort = EffortKind::kAngular;
  static constexpr std::string_view kListName = "dc_motor";

  struct Params {
    float torque_constant = 0.0F;    // Kt, N*m per A
    float back_emf_constant = 0.0F;  // Ke, V per rad/s
    float resistance = 1.0F;         // ohm, phase to phase
    float bus_voltage = 0.0F;        // V
    float max_current = 0.0F;        // A, the driver's limit
    float gear_ratio = 1.0F;         // output turns per rotor turn
    domain::error::LossChain::Params losses;
    ErrorModel::Params error;
  };

  DcMotor() = default;
  // Throws std::invalid_argument for a motor that can't turn or whose math has
  // no answer: a torque constant, bus voltage, current limit, resistance, gear
  // ratio or Coulomb smoothing that isn't positive, an efficiency outside
  // (0, 1], or a negative back-EMF constant or friction.
  DcMotor(
      AttachmentOptions options,
      const domain::math::Vector3<float>& axis,
      const Params& params
  )
      : Actuator(options, axis), params_(params) {
    const auto& losses = params_.losses;
    if (params_.torque_constant <= 0.0F || params_.bus_voltage <= 0.0F ||
        params_.max_current <= 0.0F || params_.resistance <= 0.0F ||
        params_.gear_ratio <= 0.0F || losses.coulomb.smoothing <= 0.0F) {
      throw std::invalid_argument(
          "DcMotor: torque_constant, bus_voltage, max_current, resistance, "
          "gear_ratio and coulomb_smoothing must be positive."
      );
    }
    if (losses.transmission.efficiency <= 0.0F ||
        losses.transmission.efficiency > 1.0F) {
      throw std::invalid_argument(
          "DcMotor: gearbox_efficiency must be in (0, 1]."
      );
    }
    if (params_.back_emf_constant < 0.0F || losses.viscous.coefficient < 0.0F ||
        losses.coulomb.level < 0.0F) {
      throw std::invalid_argument(
          "DcMotor: back_emf_constant, viscous_friction and coulomb_friction "
          "must not be negative."
      );
    }
    speed_curve_.stall = params_.bus_voltage / params_.resistance;
    speed_curve_.free_rate =
        params_.back_emf_constant > 0.0F
            ? params_.bus_voltage / params_.back_emf_constant
            : std::numeric_limits<float>::infinity();
    current_limit_.limit = params_.max_current;
  }

  // Settings: period, axis (required), torque_constant, back_emf_constant,
  // resistance, bus_voltage, max_current, gear_ratio, gearbox_efficiency,
  // viscous_friction, coulomb_friction, coulomb_smoothing, and the error
  // model's on the joint's torque (see AttachmentConfig::ErrorModel).
  static DcMotor FromConfig(AttachmentConfig& config) {
    const AttachmentOptions options = config.Options();
    const domain::math::Vector3<float> axis = config.RequiredVector("axis");
    Params params;
    params.torque_constant =
        config.Scalar("torque_constant", params.torque_constant);
    params.back_emf_constant =
        config.Scalar("back_emf_constant", params.back_emf_constant);
    params.resistance = config.Scalar("resistance", params.resistance);
    params.bus_voltage = config.Scalar("bus_voltage", params.bus_voltage);
    params.max_current = config.Scalar("max_current", params.max_current);
    params.gear_ratio = config.Scalar("gear_ratio", params.gear_ratio);
    auto& losses = params.losses;
    losses.transmission.efficiency =
        config.Scalar("gearbox_efficiency", losses.transmission.efficiency);
    losses.viscous.coefficient =
        config.Scalar("viscous_friction", losses.viscous.coefficient);
    losses.coulomb.level =
        config.Scalar("coulomb_friction", losses.coulomb.level);
    losses.coulomb.smoothing =
        config.Scalar("coulomb_smoothing", losses.coulomb.smoothing);
    params.error = config.ErrorModel<float>();
    return {options, axis, params};
  }

  void Drive(
      const AttachmentTick& tick,
      const Velocity& qd,
      const Subspace& subspace,
      DcMotorCommand* command,
      DcMotorState* state,
      DcMotorOutput* out
  ) const {
    using domain::error::LossChain;
    using domain::error::Saturate;
    using domain::error::SpeedCurve;
    domain::random::CounterRng rng(tick.stream, tick.tick);
    const float rotor_rate = Rate(subspace, qd) / params_.gear_ratio;

    float current = command->value;
    if (command->mode == DcMotorCommand::Mode::kVoltage) {
      const float volts =
          std::clamp(command->value, -params_.bus_voltage, params_.bus_voltage);
      current = (volts - (params_.back_emf_constant * rotor_rate)) /
                params_.resistance;
    }
    current = SpeedCurve::Apply(
        speed_curve_, state->speed_curve, rng, tick.dt, rotor_rate, current
    );
    current = Saturate<float>::Apply(
        current_limit_, state->current_limit, rng, tick.dt, current
    );
    out->current = current;

    const float rotor_torque = LossChain::Apply(
        params_.losses,
        state->losses,
        rng,
        tick.dt,
        rotor_rate,
        params_.torque_constant * current
    );
    out->effort = ErrorModel::Apply(
        params_.error,
        state->error,
        rng,
        tick.dt,
        rotor_torque / params_.gear_ratio
    );
  }

 private:
  Params params_;
  domain::error::SpeedCurve::Params speed_curve_;
  domain::error::Saturate<float>::Params current_limit_;
};

}  // namespace achilles::domain::actuation
