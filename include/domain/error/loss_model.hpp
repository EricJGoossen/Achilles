// What the mechanism takes out of an effort on its way to the load.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "domain/error/coulomb_friction.hpp"
#include "domain/error/effect_contract.hpp"
#include "domain/error/speed_curve.hpp"
#include "domain/error/transmission_loss.hpp"
#include "domain/error/viscous_friction.hpp"
#include "domain/math/vector3.hpp"
#include "domain/random/counter_rng.hpp"

namespace achilles::domain::error {

// What the mechanism takes out of an effort on its way to the load: friction,
// then the transmission's loss. Every stage defaults to off.
struct LossModel {
  struct Params {
    ViscousFriction::Params viscous{};
    CoulombFriction::Params coulomb{};
    TransmissionLoss::Params transmission{};
  };
  struct State {
    ViscousFriction::State viscous{};
    CoulombFriction::State coulomb{};
    TransmissionLoss::State transmission{};
  };

  static float Apply(
      const Params& params,
      State& state,
      Rng& rng,
      float dt,
      float rate,
      float value
  ) {
    float effort = ViscousFriction::Apply(
        params.viscous, state.viscous, rng, dt, rate, value
    );
    effort = CoulombFriction::Apply(
        params.coulomb, state.coulomb, rng, dt, rate, effort
    );
    return TransmissionLoss::Apply(
        params.transmission, state.transmission, rng, dt, rate, effort
    );
  }
};

static_assert(RateEffectLike<LossModel>);

}  // namespace achilles::domain::error
