#pragma once

#include <cstddef>

#include "engine/algorithm_contract.hpp"
#include "engine/field_contract.hpp"
#include "engine/pass/sim_context.hpp"

namespace achilles::engine::pass {

// A Step that runs StepList again while Continue says so, at most
// Limit::Get(config) times: the engine's one data-dependent loop, and the
// "repeat until a global flag clears" an iterative solver's early exit
// needs (issue #22).
//
// The decision is taken between repetitions, never inside a pass: Continue
// reads one whole-sim value (a GlobalIndexing field some Op of StepList
// wrote), the same for every lane, so every pass StepList runs stays
// branch-free and batched. It is the host-side loop a GPU port runs as its
// launch loop -- MJX's while_loop over a scalar condition -- and costs one
// load per repetition here.
//
//   Continue: `template <SimContextLike S> static bool Get(const S&)`
//   Limit:    `static std::size_t Get(const Config&)`
//
// FieldEnum is the Algorithm StepList's Steps run over (SimContext::Step
// checks that it's hosted); StepList may name Steps of any hosted Algorithm.
template <
    typename StepList,
    typename Continue,
    typename Limit,
    FieldEnumLike Enum>
struct RepeatWhileStep {
  using FieldEnum = Enum;

  template <SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& config,
      algorithms::ScalarOperationT dt
  ) {
    const std::size_t limit = Limit::Get(config);
    for (std::size_t i = 0; i < limit && Continue::Get(sim_state); ++i) {
      sim_state.template Step<StepList>(dt, config);
    }
  }
};

}  // namespace achilles::engine::pass
