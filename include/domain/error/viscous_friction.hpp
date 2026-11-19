// Drag proportional to the rate.

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

// Drag proportional to the rate.
struct ViscousFriction {
  struct Params {
    float coefficient = 0.0F;
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
    return value - (params.coefficient * rate);
  }
};

static_assert(RateEffectLike<ViscousFriction>);

}  // namespace achilles::domain::error
