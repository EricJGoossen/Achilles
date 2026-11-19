// A step list repeated while a whole-sim flag says to, for iterative
// solvers that stop early once converged.
// Doc: docs/code/engine_pass_repeat_while_step.md

#pragma once

#include <cstddef>

#include "engine/algorithm_contract.hpp"
#include "engine/pass/sim_context.hpp"

namespace achilles::engine::pass {

// A Step that runs the step list Body up to MaxRepeats times, stopping as
// soon as AlgorithmT's global field Flag reads zero. The flag is read
// between repeats, never inside a pass, so every pass stays branch-free;
// what varies is how many times the list runs. Body's last Step writes the
// flag. With the flag never clearing, this is RepeatListT<Body,
// MaxRepeats>.
//
// Example:
//   using Solve = util::ConcatT<
//       util::TypeList<SetupStep>,
//       util::TypeList<RepeatWhileStep<Iteration, SolverAlgorithm,
//                                      SolverField::kContinue, 20>>>;
template <
    typename Body,
    AlgorithmLike AlgorithmT,
    typename AlgorithmT::Enum Flag,
    std::size_t MaxRepeats>
struct RepeatWhileStep {
  using FieldEnum = typename AlgorithmT::Enum;

  // Runs Body until the flag clears or MaxRepeats runs have happened.
  //
  // sim_state: the context every Step in Body runs over.
  // config: the whole-sim config, passed through to Body.
  // dt: the tick's timestep, passed through to Body.
  template <SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& config,
      algorithms::ScalarOperationT dt
  ) {
    const auto view = sim_state.template ViewFor<AlgorithmT>();
    using FlagT = typename decltype(view)::template Scalar<Flag>;
    for (std::size_t i = 0; i < MaxRepeats; ++i) {
      if (view.template Load<Flag, FlagT>(0).Value() == FlagT{0}) {
        return;
      }
      sim_state.template Step<Body>(dt, config);
    }
  }
};

}  // namespace achilles::engine::pass
