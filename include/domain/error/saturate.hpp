// The measurement range.

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

// The measurement range: each component clamped to [-limit, limit]. 0 is
// unlimited.
template <ErrorValueLike V>
struct Saturate {
  struct Params {
    float limit = 0.0F;
  };
  struct State {};

  static V Apply(
      const Params& params,
      State& /*state*/,
      Rng& /*rng*/,
      float /*dt*/,
      const V& value
  ) {
    if (params.limit <= 0.0F) {
      return value;
    }
    return MapComponents(value, [&](float component, std::size_t) {
      return std::clamp(component, -params.limit, params.limit);
    });
  }
};

}  // namespace achilles::domain::error
