// The integrator table: every tick list, instantiated here once.
// Doc: docs/code/algorithms_integrators.md

#include "algorithms/integrators.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <string_view>

namespace achilles::algorithms {

constexpr std::
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

namespace {
consteval bool IntegratorsIndexedByEnum() {
  for (std::size_t i = 0; i < kIntegrators.size(); ++i) {
    if (static_cast<std::size_t>(kIntegrators[i].integrator) != i) {
      return false;
    }
  }
  return true;
}
}  // namespace
static_assert(
    IntegratorsIndexedByEnum(),
    "kIntegrators must list every Integrator, in enum order"
);

const IntegratorEntry& EntryFor(Integrator integrator) {
  return kIntegrators[static_cast<std::size_t>(integrator)];
}

IntegratorStepFn StepFnFor(Integrator integrator) {
  return EntryFor(integrator).step;
}

std::string_view NameOf(Integrator integrator) {
  return EntryFor(integrator).name;
}

std::optional<Integrator> IntegratorFromName(std::string_view name) {
  for (const IntegratorEntry& entry : kIntegrators) {
    if (entry.name == name) {
      return entry.integrator;
    }
  }
  return std::nullopt;
}

}  // namespace achilles::algorithms
