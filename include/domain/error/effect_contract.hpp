// What an error effect must provide, and the per-axis helpers effects share.

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

namespace achilles::domain::error {

using Rng = random::CounterRng;

using Vec3 = math::Vector3<float>;

// The value types effects act on.
template <typename V>
concept ErrorValueLike = std::same_as<V, float> || std::same_as<V, Vec3>;

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
    ErrorValueLike<V> && std::is_trivially_copyable_v<typename E::State> &&
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

// The effects only an actuator has: ways its effort departs from the ideal
// that depend on how fast it's moving. Shaped like the ones above, with the
// rate added:
//   - static float Apply(const Params&, State&, Rng&, float dt, float rate,
//                        float value).
// `rate` is along the actuator's axis: rad/s for a torque, m/s for a force.
// `value` is whatever the kind is limiting, in whatever unit it commands. None
// of these is specific to a motor: a linear slide has a force-speed curve and
// friction too.
template <typename E>
concept RateEffectLike = std::is_trivially_copyable_v<typename E::State> &&
                         std::is_default_constructible_v<typename E::State> &&
                         std::is_default_constructible_v<typename E::Params> &&
                         requires(
                             const typename E::Params& params,
                             typename E::State& state,
                             Rng& rng,
                             float dt,
                             float rate,
                             float value
                         ) {
                           {
                             E::Apply(params, state, rng, dt, rate, value)
                           } -> std::same_as<float>;
                         };

}  // namespace achilles::domain::error
