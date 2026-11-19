// A bias that wanders.

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

// A bias that wanders: integrated white noise (a gyro's rate random walk,
// an accelerometer's bias instability as a random walk). `density` is the
// walk's strength in reading-units per sqrt(second); each sample the bias
// takes a step of density * sqrt(dt) standard deviations.
template <ErrorValueLike V>
struct RandomWalk {
  struct Params {
    float density = 0.0F;
    V initial{};
  };
  struct State {
    V bias{};
    bool started = false;
  };

  static V Apply(
      const Params& params, State& state, Rng& rng, float dt, const V& value
  ) {
    if (!state.started) {
      state.bias = params.initial;
      state.started = true;
    }
    const float step = params.density * std::sqrt(std::max(dt, 0.0F));
    state.bias = MapComponents(state.bias, [&](float bias, std::size_t) {
      return bias + (step * rng.Normal());
    });
    return value + state.bias;
  }
};

}  // namespace achilles::domain::error
