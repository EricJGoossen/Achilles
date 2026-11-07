#pragma once

#include <array>

#include "algorithms/clock/clock_data.hpp"
#include "algorithms/conventions.hpp"
#include "domain/sim_clock.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/op_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/orderings.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::clock {

using ClockAlgorithm = engine::Algorithm<ClockField, ClockFieldTraits>;
static_assert(engine::AlgorithmLike<ClockAlgorithm>);

struct AdvanceClockOp {
  using FieldEnum = ClockField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  explicit AdvanceClockOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 0> kInputs = {};
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{ClockField::kClock, ArgIndex::kTarget},
  };

  void operator()(domain::SimClock* clock) const { clock->Advance(dt_); }

 private:
  ScalarOperationT dt_;
};
static_assert(engine::OpLike<AdvanceClockOp>);

// Ends a tick: the state has reached the time the tick took it to. Last in
// every integrator's list (integrators.hpp), so everything in the tick reads
// the time it started from, and a tick that throws doesn't count.
struct TickStep {
  using FieldEnum = ClockField;

  template <engine::pass::SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& /*sim_config*/,
      ScalarOperationT dt
  ) {
    engine::pass::Step(
        engine::pass::Ops<
            engine::pass::Pass<AdvanceClockOp, engine::pass::GlobalTraversal>>{
            AdvanceClockOp(dt)
        },
        sim_state.template ViewFor<ClockView>(),
        sim_state.template TopologyFor<engine::topology::TopologicalOrdering>()
    );
  }
};

}  // namespace achilles::algorithms::clock
