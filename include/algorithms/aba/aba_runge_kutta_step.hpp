#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/math/matrix.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/topology/ordering_policy.hpp"

namespace achilles::algorithms::aba {

// Classical fourth-order Runge-Kutta, the RK4 option MuJoCo ships
// alongside Euler/implicit/implicitfast (see their XML reference's own
// option/integrator attribute). Their guidance is that RK4 is the one to
// reach for on *energy-conserving* systems -- "qualitatively better than
// the single-step methods ... even when the timestep is decreased by a
// factor of 4 (so the computational effort is identical)" -- which is
// exactly the regime this engine's passive pendulum scenes live in.
//
// Treats the whole sim as one ODE y' = f(y), y = (q, v), with
// f(q, v) = (v, a(q, v)) and a(q, v) the acceleration ABA already solves
// for. The four stages are the textbook ones:
//     k1 = f(y),            k2 = f(y + h/2 k1)
//     k3 = f(y + h/2 k2),   k4 = f(y + h k3)
//     y_{n+1} = y_n + h/6 (k1 + 2 k2 + 2 k3 + k4)
// so this costs four ABA evaluations per tick -- the same as
// ImplicitMidpointStep's four fixed-point iterations, but fourth-order
// accurate rather than second, which is the whole reason to prefer it:
// the error per step falls off as h^5 instead of h^3, so the same
// per-tick budget buys a much larger usable h.
//
// What it does *not* give you is a structural guarantee. RK4 is not
// symplectic: its energy error is merely *small*, not bounded, so it
// drifts secularly over long enough runs where a time-symmetric scheme
// like ImplicitMidpointStep oscillates in a band instead. Which one wins
// is therefore a question about time horizon and step size, not a
// settled ordering -- see this repo's own integrator benchmark for the
// measured answer on these scenes. MuJoCo's own docs add the matching
// caveat from the other side: "in the presence of large velocity-
// dependent forces, if the chosen single-step method integrates those
// forces implicitly, single-step methods can be significantly more stable
// than RK4", and a double pendulum is squarely a large-velocity-
// dependent-force system.
//
// The position stages compose on SE(3) (q * Exp(S v h)) rather than
// adding, the same way IntegratePositionOp does -- a pose is not a vector
// space. Strictly, that makes this a Lie-group RK rather than textbook
// RK4, and for a general joint the two differ at higher order. For every
// scene in this repo it makes no difference: each joint is a single-axis
// revolute, rotations about one fixed axis commute, and composing them is
// exact angle addition (see tests/examples_two_joint_arm.cpp's own note
// on the same property). A multi-axis joint would want a proper
// Munthe-Kaas treatment before trusting the fourth-order claim.
struct RungeKutta4Step {
  using ScalarTransform = domain::spatial::Transform<ScalarOperationT>;
  using ScalarVelocity = domain::spatial::SpatialVelocity<ScalarOperationT>;
  using ScalarAcceleration =
      domain::spatial::SpatialAcceleration<ScalarOperationT>;
  using Matrix6 = domain::math::Matrix6x6<ScalarOperationT>;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& sim_config,
      ScalarOperationT dt
  ) {
    const auto view = sim_state.template ViewFor<ABAView>();
    const auto& topology =
        sim_state.template TopologyFor<engine::topology::TopologicalOrdering>();
    const std::size_t n = topology.Size();

    std::vector<ScalarTransform> q0(n);
    std::vector<ScalarVelocity> v0(n);
    std::vector<Matrix6> subspace(n);
    for (std::size_t row = 0; row < n; ++row) {
      q0[row] =
          view.template Load<ABAField::kJointPosition, ScalarOperationT>(row);
      v0[row] =
          view.template Load<ABAField::kJointVelocity, ScalarOperationT>(row);
      subspace[row] =
          view.template Load<ABAField::kJointSubspace, ScalarOperationT>(row);
    }

    // kv[s] is stage s's own velocity derivative of position (i.e. the
    // velocity that stage was evaluated at); ka[s] its acceleration.
    std::array<std::vector<ScalarVelocity>, 4> kv;
    std::array<std::vector<ScalarAcceleration>, 4> ka;
    for (std::size_t s = 0; s < 4; ++s) {
      kv[s].resize(n);
      ka[s].resize(n);
    }

    // Writes stage state (q0 advanced by `twist` over `position_scale`*dt,
    // v0 advanced by `accel` over `velocity_scale`*dt) into the view, then
    // runs one ABA and records that stage's own (kv, ka).
    auto evaluate_stage = [&](std::size_t stage,
                              const std::vector<ScalarVelocity>* twist,
                              const std::vector<ScalarAcceleration>* accel,
                              ScalarOperationT scale) {
      if (twist != nullptr) {
        for (std::size_t row = 0; row < n; ++row) {
          ScalarVelocity stage_v =
              v0[row] + (*accel)[row].Integrate(dt * scale);
          ScalarVelocity lifted = subspace[row] * (*twist)[row];
          ScalarTransform stage_q =
              q0[row] * ScalarTransform::Exp(lifted * (dt * scale));
          view.template Store<ABAField::kJointPosition, ScalarOperationT>(
              row, stage_q
          );
          view.template Store<ABAField::kJointVelocity, ScalarOperationT>(
              row, stage_v
          );
          kv[stage][row] = stage_v;
        }
      } else {
        for (std::size_t row = 0; row < n; ++row) {
          kv[stage][row] = v0[row];
        }
      }
      ABAStep::Step(sim_state, sim_config, dt);
      for (std::size_t row = 0; row < n; ++row) {
        ka[stage][row] =
            view.template Load<ABAField::kJointAcceleration, ScalarOperationT>(
                row
            );
      }
    };

    evaluate_stage(0, nullptr, nullptr, 0.0F);
    evaluate_stage(1, &kv[0], &ka[0], 0.5F);
    evaluate_stage(2, &kv[1], &ka[1], 0.5F);
    evaluate_stage(3, &kv[2], &ka[2], 1.0F);

    for (std::size_t row = 0; row < n; ++row) {
      // v_{n+1} = v_n + h/6 (a1 + 2 a2 + 2 a3 + a4).
      ScalarVelocity velocity = v0[row] + ka[0][row].Integrate(dt / 6.0F) +
                                ka[1][row].Integrate(dt / 3.0F) +
                                ka[2][row].Integrate(dt / 3.0F) +
                                ka[3][row].Integrate(dt / 6.0F);

      // q_{n+1} = q_n * Exp(S * h/6 (v1 + 2 v2 + 2 v3 + v4)) -- one
      // exponential of the blended twist, not four composed ones.
      ScalarVelocity blended =
          (kv[0][row] + kv[1][row] * 2.0F + kv[2][row] * 2.0F + kv[3][row]) *
          (1.0F / 6.0F);
      ScalarVelocity lifted = subspace[row] * blended;
      ScalarTransform position = q0[row] * ScalarTransform::Exp(lifted * dt);
      // See Transform::NormalizeRotationInPlace's own comment -- this is
      // the cross-tick-accumulating commit, unlike the four per-stage
      // trial positions above (each of which is discarded within this
      // same tick, before it could ever compound).
      position.NormalizeRotationInPlace();

      view.template Store<ABAField::kJointPosition, ScalarOperationT>(
          row, position
      );
      view.template Store<ABAField::kJointVelocity, ScalarOperationT>(
          row, velocity
      );
    }
  }
};

}  // namespace achilles::algorithms::aba
