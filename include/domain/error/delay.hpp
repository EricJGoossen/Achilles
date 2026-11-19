// Latency.

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

// Latency: reports the value from `samples` samples ago (until that many
// have been taken, the oldest one it has). At most MaxSamples - 1 -- the
// capacity is fixed so the state's size is, but the delay itself is
// runtime configuration.
template <ErrorValueLike V, std::size_t MaxSamples = 16>
struct Delay {
  static_assert(MaxSamples >= 1);

  struct Params {
    std::uint32_t samples = 0;
  };
  struct State {
    std::array<V, MaxSamples> ring{};
    std::uint32_t head = 0;
    std::uint32_t filled = 0;
  };

  static V Apply(
      const Params& params,
      State& state,
      Rng& /*rng*/,
      float /*dt*/,
      const V& value
  ) {
    state.ring[state.head] = value;
    state.filled =
        std::min<std::uint32_t>(state.filled + 1, std::uint32_t{MaxSamples});
    const std::uint32_t delay = std::min(params.samples, state.filled - 1);
    const std::uint32_t read =
        (state.head + std::uint32_t{MaxSamples} - delay) %
        std::uint32_t{MaxSamples};
    state.head = (state.head + 1) % std::uint32_t{MaxSamples};
    return state.ring[read];
  }
};

}  // namespace achilles::domain::error
