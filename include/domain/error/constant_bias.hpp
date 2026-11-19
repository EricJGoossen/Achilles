// A fixed offset (a gyro's turn-on bias, an encoder's zero offset).

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

// A fixed offset (a gyro's turn-on bias, an encoder's zero offset).
template <ErrorValueLike V>
struct ConstantBias {
  struct Params {
    V bias{};
  };
  struct State {};

  static V Apply(
      const Params& params,
      State& /*state*/,
      Rng& /*rng*/,
      float /*dt*/,
      const V& value
  ) {
    return value + params.bias;
  }
};

}  // namespace achilles::domain::error
