// Drag of a constant size against the motion, smoothed through zero rate so the
// step it would take at rest doesn't chatter an explicit integrator.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "domain/error/effect_contract.hpp"
#include "domain/math/vector3.hpp"
#include "domain/random/counter_rng.hpp"

namespace achilles::domain::error {

// Drag of a constant size against the motion, smoothed through zero rate so
// the step it would take at rest doesn't chatter an explicit integrator.
struct CoulombFriction {
  struct Params {
    float level = 0.0F;
    float smoothing = 2.0F;  // the rate over which it ramps up (tanh)
  };
  struct State {};

  static float Apply(
      const Params& params,
      State& /*state*/,
      Rng& /*rng*/,
      float /*dt*/,
      float rate,
      float value
  ) {
    if (params.level == 0.0F) {
      return value;
    }
    return value - (params.level * std::tanh(rate / params.smoothing));
  }
};

static_assert(RateEffectLike<CoulombFriction>);

}  // namespace achilles::domain::error
