#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/sensing/sensing_data.hpp"
#include "algorithms/sensing/sensing_ops.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/field_contract.hpp"
#include "engine/op_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/view/view_contract.hpp"

namespace achilles::algorithms::sensing {

template <SensorLike S>
using SensingAlgorithm =
    engine::Algorithm<SensingField<S>, SensingFields<S>::template Traits>;

// Runs every instance on the sim's current state, then publishes the
// readings: the flip comes after the pass, so what the user reads once the
// tick returns is this tick's reading (the mirror of an input buffer like
// actuator torque, flipped before the passes that read it).
//
// Also where a kind's generated algorithm is checked against the same
// contracts every hand-written one asserts at namespace scope -- once per
// kind, the first time it's named.
template <SensorLike S>
struct ReadStep {
  using FieldEnum = SensingField<S>;

  static_assert(engine::FieldTraitsLike<
                FieldEnum,
                SensingFields<S>::template Traits>);
  static_assert(engine::view::ViewLike<SensingView<S>, FieldEnum>);
  static_assert(engine::AlgorithmLike<SensingAlgorithm<S>>);
  static_assert(engine::OpLike<ReadOp<S>>);
  static_assert(engine::OpArgsMatchView<ReadOp<S>, SensingView<S>>);
  static_assert(engine::OpLike<InitOp<S>>);
  static_assert(engine::OpArgsMatchView<InitOp<S>, SensingView<S>>);

  template <engine::pass::SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& /*sim_config*/,
      ScalarOperationT dt
  ) {
    engine::pass::Step(
        engine::pass::Ops<
            engine::pass::Pass<ReadOp<S>, engine::pass::LinkedTraversal>>{
            ReadOp<S>(dt)
        },
        sim_state.template ViewFor<SensingView<S>>(),
        sim_state.template TopologyFor<SensingOrdering<S>>()
    );
    sim_state.template FlipParity<OutputGroup<S>>();
  }
};

// Once, at Init: resets every instance's clock and seeds its locals.
template <SensorLike S>
struct InitStep {
  using FieldEnum = SensingField<S>;

  template <engine::pass::SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& /*sim_config*/,
      ScalarOperationT /*dt*/
  ) {
    engine::pass::Step(
        engine::pass::Ops<engine::pass::Pass<
            InitOp<S>,
            engine::pass::ForwardLinearTraversal>>{InitOp<S>{}},
        sim_state.template ViewFor<SensingView<S>>(),
        sim_state.template TopologyFor<SensingOrdering<S>>()
    );
  }
};

}  // namespace achilles::algorithms::sensing
