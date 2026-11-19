// A base for steps that run their ops over every row of their algorithm's
// view.

#pragma once

#include <concepts>

#include "engine/algorithm.hpp"
#include "engine/context/sim_context.hpp"
#include "engine/op_invoker/algorithm_step.hpp"

namespace achilles::algorithms::base {

// The common step shape (see engine::spec::StepLike): one
// engine::op_invoker::Step over every row of AlgorithmT's view, running the
// ops Derived builds. Derived supplies only
//     static auto MakeOps(const Config& config, ScalarOperationT dt)
// returning an Ops<OpWalk<...>...>, each op constructed with whatever it
// needs from config and dt, and inherits FieldEnum and Step from here:
//
//   struct VIStep : base::EveryRowStep<VIStep, VIAlgorithm> {
//     static auto MakeOps(const SimConfig&, ScalarOperationT dt) {
//       return engine::op_invoker::Ops<OpWalk<IntegrateVelocityOp, Linear>>{
//           IntegrateVelocityOp(dt)};
//     }
//   };
//
// Every walk's traversal is handed view.Size(), so this fits walks whose
// traversal walks rows by count (LinearTraversal). A step that needs anything
// else -- a tree traversal, several views, more than one
// engine::op_invoker::Step call -- writes its own static Step instead (see
// algorithms/aba/aba_step.hpp).
template <typename Derived, engine::AlgorithmLike AlgorithmT>
struct EveryRowStep {
  using FieldEnum = typename AlgorithmT::Enum;

  template <
      engine::context::SimContextLike SimStateT,
      typename Config,
      std::floating_point DtT>
  static void Step(const SimStateT& sim_state, const Config& config, DtT dt) {
    const auto view = sim_state.template ViewFor<AlgorithmT>();
    engine::op_invoker::Step(Derived::MakeOps(config, dt), view, view.Size());
  }
};

}  // namespace achilles::algorithms::base
