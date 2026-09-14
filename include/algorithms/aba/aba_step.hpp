#pragma once

#include "aba_data.hpp"
#include "aba_ops.hpp"
#include "algorithms/conventions.hpp"
#include "engine/algorithm_step.hpp"
#include "engine/traversals.hpp"
#include "engine/view/planar_view.hpp"
#include "engine/view/view_contract.hpp"

namespace achilles::algorithms::aba {

// Runs one full ABA step (velocity, inertia, acceleration passes) for a
// single archetype over `view`, using `topology` for joint order/parentage.
// `x_world_base`/`v_base` seed the base row's world transform/velocity
// (PropagateVelocityOp::Initialize); `a_base` seeds its spatial
// acceleration (PropagateAccelerationOp::Initialize) -- pass
// `a_base = -gravity` for the common fixed-base case, or Acceleration::
// Zero() for a floating base with gravity applied as an explicit force
// elsewhere. Each pass seeds only the base-row state it itself reads back
// from a parent index (see the comment on aba_ops.hpp for the full list);
// there's no separate seed pass to keep in sync with them.
//
// All three passes here walk the tree, but nothing about engine::Step
// requires that -- a future pass could name engine::LinearTraversal
// instead and still take `topology` (via its Size()).
void Step(
    ABAView view,
    const ABATopology& topology,
    const Transform& x_world_base,
    const Velocity& v_base,
    const Acceleration& a_base
) {
  using engine::BackwardTreeTraversal;
  using engine::ForwardTreeTraversal;
  using engine::Pass;

  engine::Step(
      engine::Ops<
          Pass<PropagateVelocityOp, ForwardTreeTraversal>,
          Pass<PropagateInertiaOp, BackwardTreeTraversal>,
          Pass<PropagateAccelerationOp, ForwardTreeTraversal>>{
          PropagateVelocityOp(x_world_base, v_base),
          PropagateInertiaOp{},
          PropagateAccelerationOp(a_base)
      },
      view,
      topology
  );
}

}  // namespace achilles::algorithms::aba
