#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/vi/vi_data.hpp"
#include "algorithms/vi/vi_ops.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/traversals.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::vi {

struct ABAStep {
  using FieldEnum = VIField;

  using ForwardBatched = engine::pass::LinearTraversal<
      engine::pass::Direction::kForward,
      util::LaneCountOf<MathematicalT>()>;

  template <typename SimStateT>
  static void Step(VIView view, const SimStateT&, const SimConfig&, float dt) {
    using engine::pass::Pass;

    engine::pass::Step(
        engine::pass::Ops<Pass<IntegrateVelocityOp, ForwardBatched>>{
            IntegrateVelocityOp(dt)
        },
        view
    );
  }
};

}  // namespace achilles::algorithms::vi