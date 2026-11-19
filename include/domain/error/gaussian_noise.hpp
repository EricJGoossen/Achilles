// White noise.

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

// White noise. `density` is the noise density in reading-units per
// sqrt(Hz) -- the datasheet figure -- so a sample's standard deviation is
// density / sqrt(dt): sampling faster sees more noise per sample, the same
// noise power overall.
template <ErrorValueLike V>
struct GaussianNoise {
  struct Params {
    float density = 0.0F;
  };
  struct State {};

  static V Apply(
      const Params& params, State& /*state*/, Rng& rng, float dt, const V& value
  ) {
    if (params.density == 0.0F || dt <= 0.0F) {
      return value;
    }
    const float sigma = params.density / std::sqrt(dt);
    return MapComponents(value, [&](float component, std::size_t) {
      return component + (sigma * rng.Normal());
    });
  }
};

}  // namespace achilles::domain::error
