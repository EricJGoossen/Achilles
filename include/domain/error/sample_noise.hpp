// White noise given per sample.

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

// White noise given per sample: `sigma` is the standard deviation of each
// sample, whatever the rate -- a lidar's range noise, say, which a datasheet
// gives this way rather than as a density.
template <ErrorValueLike V>
struct SampleNoise {
  struct Params {
    float sigma = 0.0F;
  };
  struct State {};

  static V Apply(
      const Params& params,
      State& /*state*/,
      Rng& rng,
      float /*dt*/,
      const V& value
  ) {
    if (params.sigma == 0.0F) {
      return value;
    }
    return MapComponents(value, [&](float component, std::size_t) {
      return component + (params.sigma * rng.Normal());
    });
  }
};

}  // namespace achilles::domain::error
