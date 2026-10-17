#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/rk4/rk4_data.hpp"
#include "algorithms/rk4/rk4_ops.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/pass/view_step.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::rk4 {

// Classical fourth-order Runge-Kutta, the RK4 option MuJoCo ships alongside
// Euler/implicit/implicitfast (see their XML reference's own
// option/integrator attribute). Their guidance is that RK4 is the one to
// reach for on *energy-conserving* systems -- "qualitatively better than
// the single-step methods ... even when the timestep is decreased by a
// factor of 4 (so the computational effort is identical)" -- which is
// exactly the regime this engine's passive pendulum scenes live in.
//
// Treats the whole sim as one ODE y' = f(y), y = (q, qd), with
// f(q, qd) = (qd, qdd(q, qd)) and qdd the acceleration ABA solves for. The
// four stages are the textbook ones:
//     k1 = f(y),            k2 = f(y + h/2 k1)
//     k3 = f(y + h/2 k2),   k4 = f(y + h k3)
//     y_{n+1} = y_n + h/6 (k1 + 2 k2 + 2 k3 + k4)
// so this costs four ABA evaluations per tick -- the same as
// algorithms/midpoint's four fixed-point iterations, but fourth-order
// accurate rather than second: the error per step falls off as h^5 instead
// of h^3, so the same per-tick budget buys a much larger usable h.
//
// One tick is the step list algorithms::RungeKutta4 (registry.hpp):
// BeginStep, then per stage an ABA evaluation followed by AccumulateStep
// (fold that stage's (qd, qdd) into the running weighted sums) and, for
// stages 1-3, AdvanceStep (write the next stage's state where ABA reads
// it), then CommitStep to blend the sums into the final state. No stage's
// derivatives outlive the next stage.
//
// What it does *not* give you is a structural guarantee. RK4 is not
// symplectic: its energy error is merely *small*, not bounded, so it drifts
// secularly over long enough runs where a time-symmetric scheme like
// algorithms/midpoint oscillates in a band instead. Which one wins is a
// question about time horizon and step size, not a settled ordering.
// MuJoCo's own docs add the matching caveat from the other side: "in the
// presence of large velocity-dependent forces, if the chosen single-step
// method integrates those forces implicitly, single-step methods can be
// significantly more stable than RK4", and a double pendulum is squarely a
// large-velocity-dependent-force system.
//
// The position stages compose on SE(3) (q * Exp(S qd h)) rather than
// adding -- a pose is not a vector space. Strictly, that makes this a
// Lie-group RK rather than textbook RK4, and for a general joint the two
// differ at higher order. For every scene in this repo it makes no
// difference: each joint is a single-axis revolute, rotations about one
// fixed axis commute, and composing them is exact angle addition (see
// tests/examples_two_joint_arm.cpp's own note on the same property). A
// multi-axis joint would want a proper Munthe-Kaas treatment before
// trusting the fourth-order claim.
//
// Like vi/pi, each pass is independent per row, so it runs as a
// LinearTraversal over view.Size().
using ForwardBatched = engine::pass::LinearTraversal<
    engine::pass::Direction::kForward,
    util::LaneCountOf<MathematicalT>()>;

using RK4Algorithm = engine::Algorithm<RK4Field, RK4FieldTraits>;
static_assert(engine::AlgorithmLike<RK4Algorithm>);

// Snapshots (q_n, qd_n) and zeroes both sums.
struct BeginStep : engine::pass::ViewStep<BeginStep, RK4Algorithm> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    using engine::pass::Pass;
    return engine::pass::Ops<Pass<BeginStepOp, ForwardBatched>>{BeginStepOp()};
  }
};

// Adds Weight * (k_v, k_a) of the stage ABA just evaluated into the sums.
template <typename Weight>
struct AccumulateStep
    : engine::pass::ViewStep<AccumulateStep<Weight>, RK4Algorithm> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    using engine::pass::Pass;
    return engine::pass::Ops<Pass<AccumulateStageOp, ForwardBatched>>{
        AccumulateStageOp(util::kRatioValueV<ScalarOperationT, Weight>)
    };
  }
};

// Writes the next stage's state, advanced from (q_n, qd_n) by the previous
// stage's derivatives over DtFraction * dt.
template <typename DtFraction>
struct AdvanceStep
    : engine::pass::ViewStep<AdvanceStep<DtFraction>, RK4Algorithm> {
  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::pass::Pass;
    return engine::pass::Ops<Pass<AdvanceStageOp, ForwardBatched>>{
        AdvanceStageOp(dt * util::kRatioValueV<ScalarOperationT, DtFraction>)
    };
  }
};

// Blends the sums into (q_{n+1}, qd_{n+1}).
struct CommitStep : engine::pass::ViewStep<CommitStep, RK4Algorithm> {
  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::pass::Pass;
    return engine::pass::Ops<Pass<CommitOp, ForwardBatched>>{CommitOp(dt)};
  }
};

}  // namespace achilles::algorithms::rk4
