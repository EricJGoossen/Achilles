#pragma once

#include "aba_data.hpp"
#include "aba_ops.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_contract.hpp"

namespace achilles::algorithms::aba {

// Runs one full ABA step (velocity, inertia, acceleration passes) for a
// single archetype. `sim_state` supplies both this Step's own ABAView
// (looked up via ViewFor<ABAView>(), the same way it names its own View type
// as engine::pass::Step's Op arguments below) and the JointTopology for
// joint order/parentage (looked up by ABA's own TopologicalOrdering policy);
// `sim_config` supplies the base row's seed state --
// `world_base_transform`/`base_velocity` (PropagateVelocityOp::Initialize)
// and `base_acceleration` (PropagateAccelerationOp::Initialize, typically
// -gravity for the common fixed-base case, or Acceleration::Zero() for a
// floating base with gravity applied as an explicit force elsewhere). Each
// pass seeds only the base-row state it itself reads back from a parent
// index (see the comment on aba_ops.hpp for the full list); there's no
// separate seed pass to keep in sync with them. `dt` is unused -- ABA
// solves for acceleration, it doesn't integrate -- but still declared, the
// same (SimContext, Config, dt) shape every Step::Step takes (see
// engine/pass/sim_step.hpp).
//
// A struct (rather than a bare free function) so it can be named as
// Algorithm<ABAField, ABAFieldTraits, ABAStep>'s Step.
//
// SimStateT is a template parameter, not engine::pass::SimContext<...>
// named directly: SimContext is templated on the full Algorithms... pack of
// whichever sim hosts this Algorithm, which ABAStep itself has no reason to
// know -- it only ever calls sim_state.ViewFor<ABAView>()/
// TopologyFor<Policy>(), constrained by engine::pass::SimContextLike
// (engine/pass/sim_context.hpp) purely to check "is this actually a
// SimContext<...>", not "does it structurally have ABA's own View/Policy" --
// see SimContextLike's own comment on why the latter can't be probed
// generically now that both ViewFor and TopologyFor are std::get on real
// per-sim tuples. A SimStateT that doesn't actually carry ABA's View/
// Topology still just fails to compile inside Step's own body, same as any
// other mistyped member access; SimContextLike only catches the coarser
// mistake of the wrong object entirely. SimConfig, by contrast, is one
// concrete, whole-simulation-wide type (algorithms/sim_config.hpp) -- not
// templated, since every Step in a given sim already agrees on its shape
// (engine::pass itself stays generic over Config; ABA just always asks for
// this one).
//
// All three passes here walk the tree, but nothing about engine::pass::Step
// (the Op-level one, run inside Step below) requires that -- a future pass
// could name engine::LinearTraversal instead and still take `topology` (via
// its Size()).
struct ABAStep {
  using FieldEnum = ABAField;

  // Every ABA Op is called with TargetScalar = MathematicalT =
  // xsimd::batch<float> (see aba_data.hpp), never a scalar -- so the
  // traversal driving them must walk batch-GROUPS of the real,
  // TopologicalOrdering-built topology, not its raw rows one at a time
  // (see engine::pass::TreeTraversal's own comment on Stride for why a raw
  // row and a batch group aren't the same index space once more than one
  // group exists). Stride = util::LaneCountOf<MathematicalT>() is the one
  // piece of information that was missing before Stride existed: without
  // it, a topology with more than one lane's worth of real+padding rows
  // silently ran the traversal's own loop bound (and every parent lookup)
  // in the wrong units.
  using ForwardBatched = engine::pass::TreeTraversal<
      engine::pass::Direction::kForward,
      util::LaneCountOf<MathematicalT>()>;
  using BackwardBatched = engine::pass::TreeTraversal<
      engine::pass::Direction::kBackward,
      util::LaneCountOf<MathematicalT>()>;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state, const SimConfig& sim_config, float dt
  ) {
    (void)dt;
    using engine::pass::Pass;

    const ABAView view = sim_state.template ViewFor<ABAView>();
    const ABATopology& topology =
        sim_state.template TopologyFor<engine::topology::TopologicalOrdering>();

    engine::pass::Step(
        engine::pass::Ops<
            Pass<PropagateVelocityOp, ForwardBatched>,
            Pass<PropagateInertiaOp, BackwardBatched>,
            Pass<PropagateAccelerationOp, ForwardBatched>>{
            PropagateVelocityOp(
                sim_config.world_base_transform, sim_config.base_velocity
            ),
            PropagateInertiaOp{},
            PropagateAccelerationOp(sim_config.base_acceleration)
        },
        view,
        topology
    );
  }
};

using ABAAlgorithm = engine::Algorithm<ABAField, ABAFieldTraits, ABAStep>;
static_assert(engine::AlgorithmLike<ABAAlgorithm>);

}  // namespace achilles::algorithms::aba
