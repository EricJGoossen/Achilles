// What a step must declare: StepLike.

#pragma once

#include <concepts>
#include <cstddef>

#include "engine/spec/field_contract.hpp"

namespace achilles::engine::spec {

// A Step: one named stage of a tick, stepped from a list by
// engine::context::SimContext::Step<StepList> (see algorithms/registry.hpp for
// the real lists). Its contract:
//   - FieldEnum names the Algorithm whose View it runs over, and that
//     Algorithm must be hosted by the SimContext stepping it.
//   - A static Step(sim_state, sim_config, dt) does the stage's work through
//     Views (and JointTopologies) it looks up from sim_state.
// By convention a Step is one engine::op_invoker::Step(Ops<...>{...}, view,
// ...) call -- the Ops are the math, the Passes the order they run in -- and
// achilles::algorithms::base::EveryRowStep (algorithms/base/every_row_step.hpp)
// implements that shape for the common whole-View case. Nothing requires it: a
// Step that needs something else (a topology-driven traversal, several Views)
// writes its own Step body. Which stages run, in what order, how many times,
// with what compile-time weights, belongs to the step list, not to a Step.
// Step(...) itself is a template on SimStateT, so only FieldEnum can be
// checked here; the rest is checked where the list is stepped.
template <typename T>
concept StepLike =
    requires { typename T::FieldEnum; } && FieldEnumLike<typename T::FieldEnum>;

}  // namespace achilles::engine::spec
