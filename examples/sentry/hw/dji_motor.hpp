// DJI motors and their speed controllers, as the MCB sees them on CAN.
//
// The firmware (taproot's DjiMotor / DjiMotorTxHandler) packs four int16
// commands per frame -- 0x200 for ids 1-4, 0x1FF for ids 5-8, 0x1FE for
// GM6020s in current mode -- and each controller answers at 1 kHz on
// 0x200 + id with its rotor angle (0-8191), rotor speed (rpm), torque
// current and temperature. This model sits between those frames and a
// joint of the sim:
//
//  - the command is a current setpoint (C620: +-16384 is +-20 A; GM6020 in
//    current mode: +-16384 is +-3 A), which the controller's current loop
//    tracks instantly here, but only as far as the 24 V bus can push
//    against the motor's back-EMF;
//  - rotor torque is Kt times that current, less a little friction, and
//    reaches the joint through the gearbox (`gear_ratio`, output turns per
//    rotor turn, as the firmware's own DjiMotor is given);
//  - feedback comes from the joint's shaft angle and speed, quantized as
//    the controller does.
//
// `inverted` mirrors the firmware's flag for the same motor. It records how
// the motor is mounted: an inverted motor's rotor turns the other way to
// the joint, so its raw angle counts down and its raw commands are negated
// -- which the firmware then undoes. The joint's positive direction is the
// one the firmware calls positive.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>

namespace sentry::hw {

struct DjiMotorSpec {
  float max_current;        // A at a command of 16384
  float torque_constant;    // N*m per A, at the rotor
  float back_emf_constant;  // V per rad/s, at the rotor
  float resistance;         // ohm, phase to phase
  float bus_voltage;        // V
  float viscous_friction;   // N*m per rad/s, at the rotor
  float coulomb_friction;   // N*m, at the rotor
  float gearbox_efficiency;
  bool current_mode_6020;  // commanded in 0x1FE frames
};

// M3508 on a C620: 0.3 N*m/A at the 3591/187 stock gearbox's output, so
// 0.0156 at the rotor; 482 rpm no-load at 24 V on the stock gearbox.
inline constexpr DjiMotorSpec kM3508{
    20.0F,
    0.3F * 187.0F / 3591.0F,
    24.0F /
        (482.0F * 3591.0F / 187.0F * 2.0F * std::numbers::pi_v<float> / 60.0F),
    0.194F,
    24.0F,
    2.0e-6F,
    2.0e-3F,
    0.9F,
    false
};
// GM6020 in current mode: 0.741 N*m/A, 320 rpm no-load at 24 V.
inline constexpr DjiMotorSpec kGm6020Current{
    3.0F,
    0.741F,
    24.0F / (320.0F * 2.0F * std::numbers::pi_v<float> / 60.0F),
    1.8F,
    24.0F,
    2.0e-4F,
    0.01F,
    1.0F,
    true
};

class DjiMotor {
 public:
  // `id` is the controller's CAN id, 1-8 (taproot's MOTORn); `encoder_offset`
  // the rotor's raw angle when the joint is at zero.
  DjiMotor(
      const DjiMotorSpec& spec,
      std::uint8_t bus,
      std::uint8_t id,
      float gear_ratio,
      bool inverted,
      std::uint16_t encoder_offset
  )
      : spec_(spec),
        bus_(bus),
        id_(id),
        gear_ratio_(gear_ratio),
        inverted_(inverted),
        encoder_offset_(encoder_offset) {}

  std::uint8_t Bus() const { return bus_; }

  // Takes this motor's command out of a command frame, if it's in it.
  // Returns whether it was.
  bool HandleCommand(
      std::uint8_t bus,
      std::uint32_t frame_id,
      std::span<const std::uint8_t> data,
      float now
  ) {
    if (bus != bus_ || frame_id != CommandFrameId()) {
      return false;
    }
    const std::size_t slot = (id_ - 1U) % 4U;
    const auto raw =
        static_cast<std::int16_t>((data[2 * slot] << 8) | data[2 * slot + 1]);
    command_ = inverted_ ? -static_cast<float>(raw) : static_cast<float>(raw);
    last_command_time_ = now;
    return true;
  }

  // The torque on the joint (N*m, joint positive) for the commanded current
  // at joint speed `omega` (rad/s). Also latches the current for feedback.
  float JointTorque(float omega, float now) {
    // The controllers zero their output when commands stop coming.
    const float setpoint = now - last_command_time_ > kCommandTimeout
                               ? 0.0F
                               : command_ / 16384.0F * spec_.max_current;
    const float rotor_omega = omega / gear_ratio_;
    const float emf = spec_.back_emf_constant * rotor_omega;
    const float upper = (spec_.bus_voltage - emf) / spec_.resistance;
    const float lower = (-spec_.bus_voltage - emf) / spec_.resistance;
    current_ = std::clamp(
        std::clamp(setpoint, -spec_.max_current, spec_.max_current),
        lower,
        upper
    );
    float rotor_torque =
        spec_.torque_constant * current_ - spec_.viscous_friction * rotor_omega;
    // Coulomb friction, smoothed through zero speed.
    rotor_torque -= spec_.coulomb_friction * std::tanh(rotor_omega / 2.0F);
    // The gearbox loses a little when driving, nothing when driven.
    const bool driving = rotor_torque * rotor_omega >= 0.0F;
    return rotor_torque / gear_ratio_ *
           (driving ? spec_.gearbox_efficiency : 1.0F);
  }

  // The 1 kHz feedback frame for a joint at `angle` (rad) and `omega`
  // (rad/s): id and 8 data bytes.
  std::uint32_t FeedbackId() const { return 0x200U + id_; }
  std::array<std::uint8_t, 8> Feedback(double angle, float omega) const {
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    const double rotor_turns = angle / gear_ratio_ / kTwoPi;
    double counts = std::fmod(rotor_turns * 8192.0 + encoder_offset_, 8192.0);
    if (counts < 0.0) {
      counts += 8192.0;
    }
    auto encoder =
        static_cast<std::uint16_t>(static_cast<std::uint32_t>(counts) % 8192U);
    float rpm = omega / gear_ratio_ * 60.0F / static_cast<float>(kTwoPi);
    float current_raw = current_ / spec_.max_current * 16384.0F;
    if (inverted_) {
      encoder = static_cast<std::uint16_t>(8191U - encoder);
      rpm = -rpm;
      current_raw = -current_raw;
    }
    const auto rpm_raw =
        static_cast<std::int16_t>(std::clamp(std::lround(rpm), -32768L, 32767L)
        );
    const auto current_int = static_cast<std::int16_t>(
        std::clamp(std::lround(current_raw), -32768L, 32767L)
    );
    return {
        static_cast<std::uint8_t>(encoder >> 8),
        static_cast<std::uint8_t>(encoder & 0xFF),
        static_cast<std::uint8_t>(static_cast<std::uint16_t>(rpm_raw) >> 8),
        static_cast<std::uint8_t>(static_cast<std::uint16_t>(rpm_raw) & 0xFF),
        static_cast<std::uint8_t>(static_cast<std::uint16_t>(current_int) >> 8),
        static_cast<std::uint8_t>(
            static_cast<std::uint16_t>(current_int) & 0xFF
        ),
        35,  // degrees C
        0,
    };
  }

 private:
  static constexpr float kCommandTimeout = 0.1F;  // s

  std::uint32_t CommandFrameId() const {
    if (spec_.current_mode_6020) {
      // A GM6020 answering on 0x205-0x208 is driver id 1-4, commanded in
      // 0x1FE's four slots.
      return 0x1FEU;
    }
    return id_ <= 4 ? 0x200U : 0x1FFU;
  }

  DjiMotorSpec spec_;
  std::uint8_t bus_;
  std::uint8_t id_;
  float gear_ratio_;
  bool inverted_;
  std::uint16_t encoder_offset_;
  float command_ = 0.0F;
  float last_command_time_ = -1.0F;
  float current_ = 0.0F;
};

}  // namespace sentry::hw
