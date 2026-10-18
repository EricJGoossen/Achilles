#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/narrow_phase/narrow_phase_data.hpp"
#include "algorithms/narrow_phase/narrow_phase_ops.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/ordering_policy.hpp"

namespace achilles::algorithms::narrow_phase {

// Runs the narrow phase for a single archetype: one pass over this Step's
// own NarrowPhaseView, looked up via ViewFor<NarrowPhaseView>() the same way
// BroadPhaseStep looks up its own (see broad_phase_step.hpp).
//
// ComputeSweptBoxContactOp over PairBatched -- the same PairTraversal, with
// the same Stride and the same TopologicalOrdering topology, the broad
// phase's own pair test ran over, so each call lines up with exactly the
// kBoundsOverlap word that test wrote for it. Every lane is computed and
// kBoundsOverlap masks the result; the traversal's own PairStructure isn't
// read again, since every candidate bit already has it ANDed in.
//
// The one shape type pair today has the one Pass. Another type pair would
// add its own contact Op here, writing the same contact fields (see
// narrow_phase_data.hpp).
//
// `sim_config` and `dt` are both unused: the narrow phase is a pure function
// of this tick's pose, like the broad phase.
struct NarrowPhaseStep {
  using FieldEnum = NarrowPhaseField;

  using PairBatched =
      engine::pass::PairTraversal<engine::pass::Direction::kForward, kLanes>;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& sim_config,
      ScalarOperationT dt
  ) {
    (void)sim_config;
    (void)dt;
    using engine::pass::Pass;

    const auto view = sim_state.template ViewFor<NarrowPhaseView>();
    const NarrowPhaseTopology& topology =
        sim_state.template TopologyFor<engine::topology::TopologicalOrdering>();

    engine::pass::Step(
        engine::pass::Ops<Pass<ComputeSweptBoxContactOp, PairBatched>>{
            ComputeSweptBoxContactOp{}
        },
        view,
        topology
    );
  }
};

using NarrowPhaseAlgorithm =
    engine::Algorithm<NarrowPhaseField, NarrowPhaseFieldTraits>;
static_assert(engine::AlgorithmLike<NarrowPhaseAlgorithm>);

}  // namespace achilles::algorithms::narrow_phase
