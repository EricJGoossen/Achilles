#pragma once

#include <array>

#include "algorithms/clock/clock_data.hpp"
#include "algorithms/conventions.hpp"
#include "domain/clock/sim_clock.hpp"
#include "engine/algorithm.hpp"
#include "engine/context/sim_context.hpp"
#include "engine/op_invoker/algorithm_step.hpp"
#include "engine/spec/arg_data.hpp"
#include "engine/spec/op_contract.hpp"
#include "engine/spec/step_contract.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "policy/ordering/linear_ordering.hpp"
#include "policy/ordering/list_ordering.hpp"
#include "policy/ordering/pair_ordering.hpp"
#include "policy/ordering/topological_ordering.hpp"
#include "policy/traversal/base_traversal.hpp"
#include "policy/traversal/global_traversal.hpp"
#include "policy/traversal/pair_traversal.hpp"
#include "policy/traversal/row_traversal.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::clock {

using ClockAlgorithm = engine::Algorithm<ClockField, ClockFieldTraits>;
static_assert(engine::AlgorithmLike<ClockAlgorithm>);

struct AdvanceClockOp {
  using FieldEnum = ClockField;
  using ArgData = engine::spec::ArgData<FieldEnum>;
  using ArgIndex = engine::spec::ArgIndex;

  explicit AdvanceClockOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 0> kInputs = {};
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{ClockField::kClock, ArgIndex::kTarget},
  };

  void operator()(domain::clock::SimClock* clock) const {
    domain::clock::Advance(clock, dt_);
  }

 private:
  ScalarOperationT dt_;
};
static_assert(engine::spec::OpLike<AdvanceClockOp>);

// Ends a tick: the state has reached the time the tick took it to. Last in
// every integrator's list (integrators.hpp), so everything in the tick reads
// the time it started from, and a tick that throws doesn't count.
struct TickStep {
  using FieldEnum = ClockField;

  template <engine::context::SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& /*sim_config*/,
      ScalarOperationT dt
  ) {
    engine::op_invoker::Step(
        engine::op_invoker::Ops<engine::op_invoker::OpWalk<
            AdvanceClockOp,
            policy::traversal::GlobalTraversal>>{AdvanceClockOp(dt)},
        sim_state.template ViewFor<ClockView>(),
        sim_state.template TopologyFor<policy::ordering::TopologicalOrdering>()
    );
  }
};

}  // namespace achilles::algorithms::clock
