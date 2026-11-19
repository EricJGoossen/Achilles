// The standard error model.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "domain/error/constant_bias.hpp"
#include "domain/error/delay.hpp"
#include "domain/error/effect_contract.hpp"
#include "domain/error/gaussian_noise.hpp"
#include "domain/error/quantize.hpp"
#include "domain/error/random_walk.hpp"
#include "domain/error/sample_noise.hpp"
#include "domain/error/saturate.hpp"
#include "domain/error/scale_error.hpp"
#include "domain/math/vector3.hpp"
#include "domain/random/counter_rng.hpp"

namespace achilles::domain::error {

// The standard error model: the value goes through, in order, gain error,
// fixed bias, wandering bias, white noise of either kind, the range, finite
// resolution, and latency -- the terms a datasheet (and an Allan-variance plot)
// specifies. Every stage defaults to off, so a default ErrorModel is an ideal
// part; configure only the terms a part actually has.
template <ErrorValueLike V, std::size_t MaxDelaySamples = 16>
struct ErrorModel {
  using Scale = ScaleError<V>;
  using Bias = ConstantBias<V>;
  using Walk = RandomWalk<V>;
  using Noise = GaussianNoise<V>;
  using PerSampleNoise = SampleNoise<V>;
  using Range = Saturate<V>;
  using Resolution = Quantize<V>;
  using Latency = Delay<V, MaxDelaySamples>;

  struct Params {
    typename Scale::Params scale{};
    typename Bias::Params bias{};
    typename Walk::Params walk{};
    typename Noise::Params noise{};
    typename PerSampleNoise::Params sample_noise{};
    typename Range::Params range{};
    typename Resolution::Params resolution{};
    typename Latency::Params latency{};
  };

  struct State {
    typename Scale::State scale{};
    typename Bias::State bias{};
    typename Walk::State walk{};
    typename Noise::State noise{};
    typename PerSampleNoise::State sample_noise{};
    typename Range::State range{};
    typename Resolution::State resolution{};
    typename Latency::State latency{};
  };

  static V Apply(
      const Params& params, State& state, Rng& rng, float dt, const V& value
  ) {
    V reading = Scale::Apply(params.scale, state.scale, rng, dt, value);
    reading = Bias::Apply(params.bias, state.bias, rng, dt, reading);
    reading = Walk::Apply(params.walk, state.walk, rng, dt, reading);
    reading = Noise::Apply(params.noise, state.noise, rng, dt, reading);
    reading = PerSampleNoise::Apply(
        params.sample_noise, state.sample_noise, rng, dt, reading
    );
    reading = Range::Apply(params.range, state.range, rng, dt, reading);
    reading = Resolution::Apply(
        params.resolution, state.resolution, rng, dt, reading
    );
    return Latency::Apply(params.latency, state.latency, rng, dt, reading);
  }
};

static_assert(EffectLike<ErrorModel<float>, float>);

static_assert(EffectLike<ErrorModel<Vec3>, Vec3>);

}  // namespace achilles::domain::error
