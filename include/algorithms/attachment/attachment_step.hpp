#pragma once

#include "algorithms/attachment/attachment_data.hpp"
#include "algorithms/attachment/attachment_ops.hpp"
#include "algorithms/attachment_kinds/attachment_kinds.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::attachment {

using AttachmentAlgorithm =
    engine::Algorithm<AttachmentField, AttachmentFieldTraits>;
static_assert(engine::AlgorithmLike<AttachmentAlgorithm>);

// Runs every actuator on the state the tick starts from, each kind over its
// own segment. It writes the torque half SwapTorqueStep is about to hand ABA,
// so it comes right before that: what a caller commanded since the last tick
// reaches this one. The joint state it reads is the integrator's result, fresh;
// anything ABA derives from it (a world pose, a body's velocity) is still the
// last evaluation's. What it reports is published with the sensors' readings
// (ReadStep).
struct DriveStep {
  using FieldEnum = AttachmentField;

  template <engine::pass::SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& /*sim_config*/,
      ScalarOperationT dt
  ) {
    engine::pass::Step(
        engine::pass::SegmentOps<
            DriveOp,
            AttachmentKinds,
            engine::pass::LinkedSegmentTraversal,
            IsActuator>(dt),
        sim_state.template ViewFor<AttachmentView>(),
        sim_state.template TopologyFor<AttachmentOrdering>()
    );
  }
};

// Runs every sensor on the sim's current state -- each pass one per kind, over
// its own segment (engine::pass::SegmentOps): for a scanning kind, its Observe
// against every body; then every kind's Read. Then it publishes the outputs:
// the flip comes after the passes, so what the user reads once the tick returns
// is this tick's reading (the mirror of an input buffer like actuator torque,
// flipped before the passes that read it). Given the bodies' layout second, for
// the scanning passes to pair against.
struct ReadStep {
  using FieldEnum = AttachmentField;

  template <engine::pass::SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& /*sim_config*/,
      ScalarOperationT dt
  ) {
    engine::pass::Step(
        engine::pass::JoinOps(
            engine::pass::SegmentOps<
                ObserveOp,
                AttachmentKinds,
                engine::pass::LinkedSegmentScanTraversal,
                ScansScene>(dt),
            engine::pass::SegmentOps<
                ReadOp,
                AttachmentKinds,
                engine::pass::LinkedSegmentTraversal,
                IsSensor>(dt)
        ),
        sim_state.template ViewFor<AttachmentView>(),
        sim_state.template TopologyFor<AttachmentOrdering>(),
        sim_state.template LayoutFor<engine::topology::TopologicalOrdering>()
    );
    sim_state.template FlipParity<AttachmentOutputSlot>();
  }
};

// Once, at Init: starts every attachment (InitOp), then publishes the outputs
// that fills. Runs after the physics' setup, so a kind filling its output from
// its inputs sees the initial pose.
struct InitStep {
  using FieldEnum = AttachmentField;

  template <engine::pass::SimContextLike SimStateT, typename Config>
  static void Step(
      const SimStateT& sim_state,
      const Config& /*sim_config*/,
      ScalarOperationT /*dt*/
  ) {
    engine::pass::Step(
        engine::pass::SegmentOps<
            InitOp,
            AttachmentKinds,
            engine::pass::LinkedSegmentTraversal>(),
        sim_state.template ViewFor<AttachmentView>(),
        sim_state.template TopologyFor<AttachmentOrdering>()
    );
    sim_state.template FlipParity<AttachmentOutputSlot>();
  }
};

// What the integrators run (integrators.hpp): each tick's actuators and
// sensors, and the once-at-Init setup.
using Actuation = util::TypeList<DriveStep>;
using Sensing = util::TypeList<ReadStep>;
using AttachmentSetup = util::TypeList<InitStep>;

}  // namespace achilles::algorithms::attachment
