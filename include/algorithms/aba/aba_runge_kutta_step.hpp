#pragma once

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
#include "util/simd_ops.hpp"

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
//
// Every row is processed a whole batch group at a time (MathematicalT),
// the same groups ABAStep itself walks: a tick is four ABA passes plus a
// few cheap per-group sweeps over (q, qd), with per-group scratch that
// persists across ticks rather than being reallocated by every one.
struct RungeKutta4Step {
  // What a tick carries from stage to stage for one batch group: the
  // state it started from, and the weighted sums of each stage's own
  // velocity (k_q) and acceleration (k_v).
  struct GroupScratch {
    Transform q0;
    Velocity v0;
    Velocity kv_sum;
    Acceleration ka_sum;
  };

  template <engine::pass::SimContextLike SimStateT, typename InstrumentationT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& sim_config,
      const InstrumentationT& instrumentation,
      ScalarOperationT dt
  ) {
    constexpr std::size_t kLanes = util::LaneCountOf<MathematicalT>();
    const auto view = sim_state.template ViewFor<ABAView>();
    const auto& topology =
        sim_state.template TopologyFor<engine::topology::TopologicalOrdering>();
    // ABAStep walks exactly these groups (see its ForwardBatched) -- the
    // topology is padded out to whole groups.
    const std::size_t groups = topology.Size() / kLanes;

    thread_local std::vector<GroupScratch> scratch;
    scratch.resize(groups);

    auto load_q = [&](std::size_t g) {
      return view.template Load<ABAField::kJointPosition, MathematicalT>(g);
    };
    auto load_v = [&](std::size_t g) {
      return view.template Load<ABAField::kJointVelocity, MathematicalT>(g);
    };
    auto load_qdd = [&](std::size_t g) {
      return view.template Load<ABAField::kJointAcceleration, MathematicalT>(g);
    };
    auto load_s = [&](std::size_t g) {
      return view.template Load<ABAField::kJointSubspace, MathematicalT>(g);
    };

    // Writes the next stage's trial state, q0 * Exp(S kv h) and v0 + ka h,
    // with h = dt * scale.
    auto store_stage = [&](std::size_t g,
                           const Velocity& kv,
                           const Acceleration& ka,
                           ScalarOperationT scale) {
      const GroupScratch& s = scratch[g];
      const MathematicalT h(dt * scale);
      const Velocity lifted = load_s(g) * kv;
      view.template Store<ABAField::kJointPosition, MathematicalT>(
          g, s.q0 * Transform::Exp(lifted * h)
      );
      view.template Store<ABAField::kJointVelocity, MathematicalT>(
          g, s.v0 + ka.Integrate(h)
      );
    };

    // Stage 1 at (q0, v0): the state the view already holds.
    ABAStep::Step(sim_state, sim_config, instrumentation, dt);
    for (std::size_t g = 0; g < groups; ++g) {
      GroupScratch& s = scratch[g];
      s.q0 = load_q(g);
      s.v0 = load_v(g);
      const Acceleration ka = load_qdd(g);
      s.kv_sum = s.v0;
      s.ka_sum = ka;
      store_stage(g, s.v0, ka, 0.5F);
    }

    // Stages 2 and 3 (weight 2), each evaluated at the trial state the
    // previous one stored; stage 3 sets up stage 4 a full step out.
    const MathematicalT two(2.0F);
    for (ScalarOperationT next_scale : {0.5F, 1.0F}) {
      ABAStep::Step(sim_state, sim_config, instrumentation, dt);
      for (std::size_t g = 0; g < groups; ++g) {
        GroupScratch& s = scratch[g];
        const Velocity kv = load_v(g);
        const Acceleration ka = load_qdd(g);
        s.kv_sum += kv * two;
        s.ka_sum += ka * two;
        store_stage(g, kv, ka, next_scale);
      }
    }

    // Stage 4, then the commit:
    //   v_{n+1} = v_n + h/6 (a1 + 2 a2 + 2 a3 + a4)
    //   q_{n+1} = q_n * Exp(S * h/6 (v1 + 2 v2 + 2 v3 + v4))
    // -- one exponential of the blended twist, not four composed ones.
    ABAStep::Step(sim_state, sim_config, instrumentation, dt);
    const MathematicalT sixth_dt(dt / 6.0F);
    for (std::size_t g = 0; g < groups; ++g) {
      const GroupScratch& s = scratch[g];
      const Velocity kv_sum = s.kv_sum + load_v(g);
      const Acceleration ka_sum = s.ka_sum + load_qdd(g);
      Transform position =
          s.q0 * Transform::Exp((load_s(g) * kv_sum) * sixth_dt);
      // See Transform::NormalizeRotationInPlace's own comment -- this is
      // the cross-tick-accumulating commit, unlike the four per-stage
      // trial positions above (each of which is discarded within this
      // same tick, before it could ever compound).
      position.NormalizeRotationInPlace();
      view.template Store<ABAField::kJointPosition, MathematicalT>(g, position);
      view.template Store<ABAField::kJointVelocity, MathematicalT>(
          g, s.v0 + ka_sum.Integrate(sixth_dt)
      );
    }
  }
};

}  // namespace achilles::algorithms::aba
