#pragma once

#include "algorithms/conventions.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"

namespace achilles::engine::pass {

// Optional base for the common Step shape (see engine::StepLike): one
// engine::pass::Step over every row of AlgorithmT's View, running the Ops
// Derived builds. Derived supplies only
//     static auto MakeOps(const Config& config, ScalarOperationT dt)
// returning an Ops<Pass<...>...> -- each Op constructed with whatever it
// needs from config/dt -- and inherits FieldEnum and Step from here:
//
//   struct VIStep : engine::pass::ViewStep<VIStep, VIAlgorithm> {
//     static auto MakeOps(const SimConfig&, ScalarOperationT dt) {
//       return engine::pass::Ops<Pass<IntegrateVelocityOp, Linear>>{
//           IntegrateVelocityOp(dt)};
//     }
//   };
//
// The size handed to every Pass's traversal is view.Size(), so this fits
// passes whose traversal walks rows by count (LinearTraversal). A Step that
// needs anything else -- a JointTopology-driven TreeTraversal, several
// Views, more than one engine::pass::Step call -- writes its own static
// Step instead (see algorithms/aba/aba_step.hpp).
template <typename Derived, AlgorithmLike AlgorithmT>
struct ViewStep {
  using FieldEnum = typename AlgorithmT::Enum;

  template <SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& config,
      algorithms::ScalarOperationT dt
  ) {
    const auto view = sim_state.template ViewFor<AlgorithmT>();
    engine::pass::Step(Derived::MakeOps(config, dt), view, view.Size());
  }
};

}  // namespace achilles::engine::pass
