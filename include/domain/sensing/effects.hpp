#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "domain/math/vector3.hpp"
#include "domain/random/counter_rng.hpp"

// The building blocks of a sensor's error model. Each effect is one way a
// real sensor's reading departs from the truth, as a stateless type with
//   - Params: its runtime configuration (a datasheet number or two), whose
//     default leaves the reading untouched -- so a sensor varies per
//     instance by data, never by type;
//   - State: what it remembers between samples (trivially copyable -- it
//     lives in the sim's memory, inside the sensor's own State);
//   - static V Apply(const Params&, State&, Rng&, float dt, const V& value).
// V is the reading's type -- float for a single-axis sensor, Vector3<float>
// for a three-axis one -- and every effect acts per component. Apply runs
// once per sample, with dt the time between samples.
namespace achilles::domain::sensing {

using Rng = random::CounterRng;
using Vec3 = math::Vector3<float>;

// The reading types effects act on.
template <typename V>
concept SensorValueLike = std::same_as<V, float> || std::same_as<V, Vec3>;

inline float Component(float value, std::size_t /*axis*/) { return value; }
inline float Component(const Vec3& value, std::size_t axis) {
  return value[axis];
}

// `fn(component, axis)` applied to each component of `value`.
template <typename Fn>
float MapComponents(float value, const Fn& fn) {
  return fn(value, std::size_t{0});
}

template <typename Fn>
Vec3 MapComponents(const Vec3& value, const Fn& fn) {
  return {
      fn(value.X(), std::size_t{0}),
      fn(value.Y(), std::size_t{1}),
      fn(value.Z(), std::size_t{2})
  };
}

template <typename E, typename V>
concept EffectLike =
    SensorValueLike<V> && std::is_trivially_copyable_v<typename E::State> &&
    std::is_default_constructible_v<typename E::State> &&
    std::is_default_constructible_v<typename E::Params> &&
    requires(
        const typename E::Params& params,
        typename E::State& state,
        Rng& rng,
        float dt,
        const V& value
    ) {
      { E::Apply(params, state, rng, dt, value) } -> std::same_as<V>;
    };

// A per-axis gain error: reads (1 + error) times the truth.
template <SensorValueLike V>
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

// A fixed offset (a gyro's turn-on bias, an encoder's zero offset).
template <SensorValueLike V>
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

// A bias that wanders: integrated white noise (a gyro's rate random walk,
// an accelerometer's bias instability as a random walk). `density` is the
// walk's strength in reading-units per sqrt(second); each sample the bias
// takes a step of density * sqrt(dt) standard deviations.
template <SensorValueLike V>
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

// White noise. `density` is the noise density in reading-units per
// sqrt(Hz) -- the datasheet figure -- so a sample's standard deviation is
// density / sqrt(dt): sampling faster sees more noise per sample, the same
// noise power overall.
template <SensorValueLike V>
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

// The measurement range: each component clamped to [-limit, limit]. 0 is
// unlimited.
template <SensorValueLike V>
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

// Finite resolution: each component rounded to the nearest multiple of
// `resolution` (an ADC's LSB, an encoder's count). 0 is continuous.
template <SensorValueLike V>
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

// Latency: reports the value from `samples` samples ago (until that many
// have been taken, the oldest one it has). At most MaxSamples - 1 -- the
// capacity is fixed so the state's size is, but the delay itself is
// runtime configuration.
template <SensorValueLike V, std::size_t MaxSamples = 16>
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

// The standard error model every library sensor kind uses: the truth goes
// through, in order, gain error, fixed bias, wandering bias, white noise,
// the measurement range, finite resolution, and latency -- the terms a
// sensor datasheet (and an Allan-variance plot) specifies. Every stage
// defaults to off, so a default ErrorChain is an ideal sensor; configure
// only the terms a part actually has.
template <SensorValueLike V, std::size_t MaxDelaySamples = 16>
struct ErrorChain {
  using Scale = ScaleError<V>;
  using Bias = ConstantBias<V>;
  using Walk = RandomWalk<V>;
  using Noise = GaussianNoise<V>;
  using Range = Saturate<V>;
  using Resolution = Quantize<V>;
  using Latency = Delay<V, MaxDelaySamples>;

  struct Params {
    typename Scale::Params scale{};
    typename Bias::Params bias{};
    typename Walk::Params walk{};
    typename Noise::Params noise{};
    typename Range::Params range{};
    typename Resolution::Params resolution{};
    typename Latency::Params latency{};
  };

  struct State {
    typename Scale::State scale{};
    typename Bias::State bias{};
    typename Walk::State walk{};
    typename Noise::State noise{};
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
    reading = Range::Apply(params.range, state.range, rng, dt, reading);
    reading = Resolution::Apply(
        params.resolution, state.resolution, rng, dt, reading
    );
    return Latency::Apply(params.latency, state.latency, rng, dt, reading);
  }
};

static_assert(EffectLike<ErrorChain<float>, float>);
static_assert(EffectLike<ErrorChain<Vec3>, Vec3>);

}  // namespace achilles::domain::sensing
