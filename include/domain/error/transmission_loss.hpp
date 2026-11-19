// A transmission's loss.

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

// A transmission's loss: (1 - efficiency) of the effort when it drives the
// load, nothing when the load back-drives it.
struct TransmissionLoss {
  struct Params {
    float efficiency = 1.0F;
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
    return value * rate >= 0.0F ? value * params.efficiency : value;
  }
};

static_assert(RateEffectLike<TransmissionLoss>);

}  // namespace achilles::domain::error
