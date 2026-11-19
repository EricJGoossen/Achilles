// A per-axis gain error.

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

// A per-axis gain error: reads (1 + error) times the truth.
template <ErrorValueLike V>
struct ScaleError {
  struct Params {
    V error{};
  };
  struct State {};

  static V Apply(
      const Params& params,
      State& /*state*/,
      Rng& /*rng*/,
      float /*dt*/,
      const V& value
  ) {
    return MapComponents(value, [&](float component, std::size_t axis) {
      return component * (1.0F + Component(params.error, axis));
    });
  }
};

}  // namespace achilles::domain::error
