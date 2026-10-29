#pragma once

#include <cstdint>
#include <numbers>
#include <stdexcept>
#include <string_view>

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "domain/math/vector3.hpp"
#include "domain/random/counter_rng.hpp"
#include "domain/sensing/effects.hpp"

namespace achilles::algorithms::sensing {

// One encoder sample: the joint's position along the encoder's axis
// (radians for a rotary encoder, meters for a linear one) and its rate.
struct EncoderReading {
  float position = 0.0F;
  float velocity = 0.0F;
};

// What an encoder remembers: its error models' state, and -- for a
// multi-turn rotary encoder -- the unwrapped angle so far.
struct EncoderState {
  domain::sensing::ErrorChain<float>::State position_error;
  domain::sensing::ErrorChain<float>::State velocity_error;
  float last_angle = 0.0F;
  float unwrapped = 0.0F;
  bool started = false;
};

// A joint encoder: measures its mount joint's motion along one axis of the
// joint's own frame -- the rotation about it (rotary) or the translation
// along it (linear). Position comes from the joint's displacement, velocity
// from its generalized velocity mapped through S; both then go through
// their own error model. A rotary encoder's counts are the position
// model's resolution (2*pi / counts-per-revolution); a single-turn one
// reports (-pi, pi], a multi-turn one the unwrapped total since Init. A
// real drive usually differentiates counts for velocity -- model that with
// the velocity model's noise, resolution and latency.
class Encoder
    : public Sensor<
          Encoder,
          SimInputs<
              SimInput<aba::ABAAlgorithm, aba::ABAField::kJointPosition>,
              SimInput<aba::ABAAlgorithm, aba::ABAField::kJointVelocity>,
              SimInput<aba::ABAAlgorithm, aba::ABAField::kJointSubspace>>,
          EncoderState,
          EncoderReading> {
 public:
  using Position =
      InputValueT<SimInput<aba::ABAAlgorithm, aba::ABAField::kJointPosition>>;
  using Velocity =
      InputValueT<SimInput<aba::ABAAlgorithm, aba::ABAField::kJointVelocity>>;
  using Subspace =
      InputValueT<SimInput<aba::ABAAlgorithm, aba::ABAField::kJointSubspace>>;

  static constexpr std::string_view kListName = "encoder";

  enum class Motion : std::uint8_t { kRotary, kLinear };

  struct Params {
    // In the joint's own frame; any nonzero length.
    domain::math::Vector3<float> axis = domain::math::Vector3<float>::UnitX();
    Motion motion = Motion::kRotary;
    bool multi_turn = true;
    domain::sensing::ErrorChain<float>::Params position_error;
    domain::sensing::ErrorChain<float>::Params velocity_error;
  };

  Encoder() = default;
  // Throws std::invalid_argument for a zero axis.
  Encoder(SensorOptions options, const Params& params)
      : Sensor(options), params_(params) {
    if (params_.axis.Norm() <= 0.0F) {
      throw std::invalid_argument("Encoder: the axis must be nonzero.");
    }
    params_.axis = params_.axis.Normalize();
  }

  // Settings: period, axis, linear (false: rotary), multi_turn, and two
  // error models, prefixed position_ and velocity_ (see
  // SensorConfig::ErrorModel).
  static Encoder FromConfig(SensorConfig& config) {
    const SensorOptions options = config.Options();
    Params params;
    params.axis = config.Vector("axis", params.axis);
    params.motion =
        config.Flag("linear", false) ? Motion::kLinear : Motion::kRotary;
    params.multi_turn = config.Flag("multi_turn", params.multi_turn);
    params.position_error = config.ErrorModel<float>("position_");
    params.velocity_error = config.ErrorModel<float>("velocity_");
    return Encoder(options, params);
  }

  void Read(
      const SensorTick& tick,
      const Position& q,
      const Velocity& qd,
      const Subspace& subspace,
      EncoderState* state,
      EncoderReading* out
  ) const {
    const auto rates = subspace * qd;
    const bool rotary = params_.motion == Motion::kRotary;
    const float position =
        rotary ? Angle(q, state) : q.Translation().Dot(params_.axis);
    const float velocity = rotary ? rates.Angular().Dot(params_.axis)
                                  : rates.Linear().Dot(params_.axis);

    domain::random::CounterRng rng(tick.stream, tick.tick);
    out->position = domain::sensing::ErrorChain<float>::Apply(
        params_.position_error, state->position_error, rng, tick.dt, position
    );
    out->velocity = domain::sensing::ErrorChain<float>::Apply(
        params_.velocity_error, state->velocity_error, rng, tick.dt, velocity
    );
  }

 private:
  // The rotation about the axis; unwrapped across samples when multi-turn.
  float Angle(const Position& q, EncoderState* state) const {
    const float angle = q.Rotation().AngleAbout(params_.axis);
    if (!params_.multi_turn) {
      return angle;
    }
    if (!state->started) {
      state->started = true;
      state->unwrapped = angle;
    } else {
      float step = angle - state->last_angle;
      constexpr float kPi = std::numbers::pi_v<float>;
      if (step > kPi) {
        step -= 2.0F * kPi;
      } else if (step < -kPi) {
        step += 2.0F * kPi;
      }
      state->unwrapped += step;
    }
    state->last_angle = angle;
    return state->unwrapped;
  }

  Params params_;
};

}  // namespace achilles::algorithms::sensing
