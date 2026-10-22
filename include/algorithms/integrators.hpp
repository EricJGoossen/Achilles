#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <ratio>
#include <string_view>

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/broad_phase/broad_phase_step.hpp"
#include "algorithms/contact_solver/contact_solver_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/midpoint/midpoint_step.hpp"
#include "algorithms/narrow_phase/narrow_phase_step.hpp"
#include "algorithms/pi/pi_step.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/rk4/rk4_step.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/verlet/verlet_step.hpp"
#include "algorithms/vi/vi_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms {

// Integrators: each is the ordered list of Steps one tick runs (see
// engine::StepLike). Every Step is a single pass of its own Algorithm's
// Ops; aba::ABAStep is the dynamics evaluation qdd = f(q, qd), and each
// integrator is just where it evaluates that and what it does with the
// result. ABA's fields are SharedAs-aliased with every integrator's, so
// each ABAStep reads whatever state the Step before it wrote.

// The collision pipeline: the broad phase's candidate pairs, then the
// narrow phase's contacts for them (algorithms/broad_phase,
// algorithms/narrow_phase). Both read only the world pose ABA last wrote,
// so it runs anywhere after an ABAStep and sees that evaluation's pose.
using Collision =
    util::TypeList<broad_phase::BroadPhaseStep, narrow_phase::NarrowPhaseStep>;

// mj_step's order: ABA solves for the unconstrained qdd at (q, qd), the
// collision pipeline finds this pose's contacts, the contact solver
// (algorithms/contact_solver) replaces qdd with the constrained qacc, VI
// integrates that into qd, and PI integrates the updated qd into q for the
// next tick's ABA to read.
using SemiImplicitEuler = util::ConcatT<
    util::TypeList<aba::ABAStep>,
    Collision,
    contact_solver::Solve,
    util::TypeList<vi::VIStep, pi::PIStep>>;

// See algorithms/verlet.
using VelocityVerlet = util::TypeList<
    aba::ABAStep,
    verlet::KickStep<std::ratio<1, 2>>,
    verlet::DriftStep,
    aba::ABAStep,
    verlet::KickStep<std::ratio<1, 2>>>;

// See algorithms/midpoint. Four fixed-point iterations per tick.
inline constexpr std::size_t kImplicitMidpointIterations = 4;
using ImplicitMidpoint = util::ConcatT<
    util::TypeList<midpoint::BeginStep>,
    util::RepeatListT<
        util::TypeList<midpoint::TrialStep, aba::ABAStep, midpoint::UpdateStep>,
        kImplicitMidpointIterations>,
    util::TypeList<midpoint::CommitStep>>;

// See algorithms/rk4. Stage weights 1, 2, 2, 1; stages 2-4 advance from the
// tick's start by dt/2, dt/2, dt.
using RungeKutta4 = util::TypeList<
    rk4::BeginStep,
    aba::ABAStep,
    rk4::AccumulateStep<std::ratio<1>>,
    rk4::AdvanceStep<std::ratio<1, 2>>,
    aba::ABAStep,
    rk4::AccumulateStep<std::ratio<2>>,
    rk4::AdvanceStep<std::ratio<1, 2>>,
    aba::ABAStep,
    rk4::AccumulateStep<std::ratio<2>>,
    rk4::AdvanceStep<std::ratio<1>>,
    aba::ABAStep,
    rk4::AccumulateStep<std::ratio<1>>,
    rk4::CommitStep>;

// See Integrator::kImplicitFast below.
using ImplicitFast = SemiImplicitEuler;

// Which integrator a tick runs. Every integrator's fields are hosted all
// the time (registry.hpp), so this can change between any two ticks. See
// each algorithm's own *_step.hpp for its derivation and tradeoffs:
//
// kSemiImplicitEuler -- ABA, collision and the contact solver, then VI's
// velocity kick, then PI's position update (algorithms/vi, algorithms/pi).
// One ABA evaluation per tick.
//
// kVelocityVerlet (algorithms/verlet) -- ABA -> half kick -> full drift
// -> ABA -> half kick. Two ABA evaluations; one order better than Euler
// for the position-dependent part of the dynamics, but ABA's velocity-
// dependent bias terms are evaluated at an extrapolated velocity.
//
// kImplicitMidpoint (algorithms/midpoint) -- the time-symmetric scheme:
// ABA at the average of the old and new state, converged by fixed-point
// iteration (four ABA evaluations).
//
// kRungeKutta4 (algorithms/rk4) -- classical RK4: four ABA evaluations,
// fourth-order accurate. MuJoCo points at RK4 for energy-conserving
// systems specifically.
//
// kImplicitFast is MuJoCo's own recommended default, and for *this*
// engine it is deliberately identical to kSemiImplicitEuler. That is not
// a stub: implicitfast's whole content is integrating the *non*-RNE
// velocity-dependent forces implicitly -- joint damping, actuator and
// fluid-drag terms -- while explicitly dropping the Coriolis/centripetal
// derivatives that MuJoCo's full `implicit` keeps. This engine models none
// of those forces (no damping coefficient, no actuators, no fluid), so
// implicitfast's D matrix is identically zero here and the scheme
// collapses to semi-implicit Euler exactly. It is named separately anyway
// so a benchmark across "every MuJoCo integrator" can report it honestly
// rather than quietly omitting it -- and so that the day joint damping is
// added, there is an obvious place for it to become real.
enum class Integrator : std::uint8_t {
  kSemiImplicitEuler,
  kVelocityVerlet,
  kImplicitMidpoint,
  kRungeKutta4,
  kImplicitFast,

  kCount
};

// What runs once, when a sim is built (interface::Simulation::Init), before
// its first tick -- MuJoCo's mj_setConst: ABA at the initial pose, then the
// contact solver's body_invweight0 from its factors.
using Setup = util::TypeList<aba::ABAStep, contact_solver::InvWeightStep>;

// The SimContext every hosted Algorithm lives in, and the shape of one
// integrator tick over it.
using RegisteredSimContext = engine::pass::SimContextForT<RegisteredAlgorithms>;
using IntegratorStepFn =
    void (*)(const RegisteredSimContext&, const SimConfig&, ScalarOperationT);

// One tick of StepList -- what an IntegratorStepFn points at.
template <typename StepList>
void StepIntegrator(
    const RegisteredSimContext& sim_context,
    const SimConfig& sim_config,
    ScalarOperationT dt
) {
  sim_context.template Step<StepList>(dt, sim_config);
}

// Every Integrator's name and tick, indexed by the enum. A caller resolves
// an Integrator to its IntegratorStepFn once (StepFnFor) and calls that per
// tick, so choosing an integrator costs nothing on the step path.
struct IntegratorEntry {
  Integrator integrator;
  std::string_view name;
  IntegratorStepFn step;
};

inline constexpr std::
    array<IntegratorEntry, static_cast<std::size_t>(Integrator::kCount)>
        kIntegrators = {{
            {Integrator::kSemiImplicitEuler,
             "euler",
             &StepIntegrator<SemiImplicitEuler>},
            {Integrator::kVelocityVerlet,
             "verlet",
             &StepIntegrator<VelocityVerlet>},
            {Integrator::kImplicitMidpoint,
             "midpoint",
             &StepIntegrator<ImplicitMidpoint>},
            {Integrator::kRungeKutta4, "rk4", &StepIntegrator<RungeKutta4>},
            {Integrator::kImplicitFast,
             "implicitfast",
             &StepIntegrator<ImplicitFast>},
        }};

namespace detail {
consteval bool IntegratorsIndexedByEnum() {
  for (std::size_t i = 0; i < kIntegrators.size(); ++i) {
    if (static_cast<std::size_t>(kIntegrators[i].integrator) != i) {
      return false;
    }
  }
  return true;
}
}  // namespace detail
static_assert(
    detail::IntegratorsIndexedByEnum(),
    "kIntegrators must list every Integrator, in enum order"
);

constexpr const IntegratorEntry& EntryFor(Integrator integrator) {
  return kIntegrators[static_cast<std::size_t>(integrator)];
}

constexpr IntegratorStepFn StepFnFor(Integrator integrator) {
  return EntryFor(integrator).step;
}

constexpr std::string_view NameOf(Integrator integrator) {
  return EntryFor(integrator).name;
}

// The Integrator called `name` (kIntegrators' names), or std::nullopt.
constexpr std::optional<Integrator> IntegratorFromName(std::string_view name) {
  for (const IntegratorEntry& entry : kIntegrators) {
    if (entry.name == name) {
      return entry.integrator;
    }
  }
  return std::nullopt;
}

}  // namespace achilles::algorithms
