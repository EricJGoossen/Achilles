#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/sensing/sensing_data.hpp"
#include "algorithms/sensing/sensing_ops.hpp"
#include "algorithms/sensing/sensor_kinds.hpp"
#include "algorithms/shared_slots.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::sensing {

using SensingAlgorithm = engine::Algorithm<SensingField, SensingFieldTraits>;
static_assert(engine::AlgorithmLike<SensingAlgorithm>);

// Runs every sensor on the sim's current state -- each pass one per kind,
// over its own segment (engine::pass::SegmentOps): for a scanning kind, a
// gate deciding who samples, then its Observe against every body; then
// every kind's Read. Then it publishes the readings: the flip comes after
// the passes, so what the user reads once the tick returns is this tick's
// reading (the mirror of an input buffer like actuator torque, flipped
// before the passes that read it). Given the bodies' layout second, for
// the scanning passes to pair against.
struct ReadStep {
  using FieldEnum = SensingField;

  template <engine::pass::SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& /*sim_config*/,
      ScalarOperationT dt
  ) {
    engine::pass::Step(
        engine::pass::JoinOps(
            engine::pass::SegmentOps<
                GateOp,
                SensorKinds,
                engine::pass::SelfSegmentTraversal,
                ScansScene>(dt),
            engine::pass::SegmentOps<
                ObserveOp,
                SensorKinds,
                engine::pass::LinkedSegmentScanTraversal,
                ScansScene>(dt),
            engine::pass::SegmentOps<
                ReadOp,
                SensorKinds,
                engine::pass::LinkedSegmentTraversal>(dt)
        ),
        sim_state.template ViewFor<SensingView>(),
        sim_state.template TopologyFor<SensingOrdering>(),
        sim_state.template LayoutFor<engine::topology::TopologicalOrdering>()
    );
    sim_state.template FlipParity<SensingOutputSlot>();
  }
};

// Once, at Init: resets every sensor's clock and seeds its state.
struct InitStep {
  using FieldEnum = SensingField;

  template <engine::pass::SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& /*sim_config*/,
      ScalarOperationT /*dt*/
  ) {
    engine::pass::Step(
        engine::pass::
            SegmentOps<InitOp, SensorKinds, engine::pass::SelfSegmentTraversal>(
            ),
        sim_state.template ViewFor<SensingView>(),
        sim_state.template TopologyFor<SensingOrdering>()
    );
  }
};

// What the integrators run (integrators.hpp): each tick's sensing, and the
// once-at-Init setup.
using Sensing = util::TypeList<ReadStep>;
using SensingSetup = util::TypeList<InitStep>;

}  // namespace achilles::algorithms::sensing
