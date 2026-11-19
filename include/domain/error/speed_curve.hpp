// The speed-effort curve.

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

// The speed-effort curve: all of `stall` at rest, falling linearly to none at
// `free_rate`, and more than `stall` against the motion. For a motor this is
// the bus voltage pushing against back-EMF. 0 stall is no curve.
struct SpeedCurve {
  struct Params {
    float stall = 0.0F;
    float free_rate = 0.0F;
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
    if (params.stall <= 0.0F) {
      return value;
    }
    const float fraction = rate / params.free_rate;
    return std::clamp(
        value,
        -params.stall * (1.0F + fraction),
        params.stall * (1.0F - fraction)
    );
  }
};

static_assert(RateEffectLike<SpeedCurve>);

}  // namespace achilles::domain::error
