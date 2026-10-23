#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/joint_limit/joint_limit_data.hpp"
#include "algorithms/joint_limit/joint_limit_ops.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/ordering_policy.hpp"

namespace achilles::algorithms::joint_limit {

// Runs the joint-limit detector for a single archetype: one pass of
// DetectJointLimitOp over this Step's own JointLimitView. Each joint's rows
// depend only on its own q, so a LinearTraversal, like the broad phase's
// bounds pass, over the same TopologicalOrdering topology.
//
// `sim_config` and `dt` are both unused: like the rest of the collision
// pipeline, it's a pure function of this tick's pose.
struct JointLimitStep {
  using FieldEnum = JointLimitField;

  using LinearBatched =
      engine::pass::LinearTraversal<engine::pass::Direction::kForward, kLanes>;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& sim_config,
      ScalarOperationT dt
  ) {
    (void)sim_config;
    (void)dt;
    using engine::pass::Pass;

    const auto view = sim_state.template ViewFor<JointLimitView>();
    const JointLimitTopology& topology =
        sim_state.template TopologyFor<engine::topology::TopologicalOrdering>();

    engine::pass::Step(
        engine::pass::Ops<Pass<DetectJointLimitOp, LinearBatched>>{
            DetectJointLimitOp{}
        },
        view,
        topology
    );
  }
};

using JointLimitAlgorithm =
    engine::Algorithm<JointLimitField, JointLimitFieldTraits>;
static_assert(engine::AlgorithmLike<JointLimitAlgorithm>);

}  // namespace achilles::algorithms::joint_limit
