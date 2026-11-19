// Finite resolution.

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

// Finite resolution: each component rounded to the nearest multiple of
// `resolution` (an ADC's LSB, an encoder's count). 0 is continuous.
template <ErrorValueLike V>
struct Quantize {
  struct Params {
    float resolution = 0.0F;
  };
  struct State {};

  static V Apply(
      const Params& params,
      State& /*state*/,
      Rng& /*rng*/,
      float /*dt*/,
      const V& value
  ) {
    if (params.resolution <= 0.0F) {
      return value;
    }
    return MapComponents(value, [&](float component, std::size_t) {
      return std::round(component / params.resolution) * params.resolution;
    });
  }
};

}  // namespace achilles::domain::error
