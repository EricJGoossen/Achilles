#pragma once

#include <concepts>
#include <cstddef>

#include "engine/field_contract.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::engine {

// A Step: one named stage of a tick, stepped from a list by
// engine::pass::SimContext::Step<StepList> (see algorithms/registry.hpp for
// the real lists). Its contract:
//   - FieldEnum names the Algorithm whose View it runs over, and that
//     Algorithm must be hosted by the SimContext stepping it.
//   - A static Step(sim_state, sim_config, dt) does the stage's work through
//     Views (and JointTopologies) it looks up from sim_state.
// By convention a Step is one engine::pass::Step(Ops<...>{...}, view, ...)
// call -- the Ops are the math, the Passes the order they run in -- and
// engine::pass::ViewStep (engine/pass/view_step.hpp) implements that shape
// for the common whole-View case. Nothing requires it: a Step that needs
// something else (a topology-driven traversal, several Views) writes its
// own Step body. Which stages run, in what order, how many times, with
// what compile-time weights, belongs to the step list, not to a Step.
// Step(...) itself is a template on SimStateT, so only FieldEnum can be
// checked here; the rest is checked where the list is stepped.
template <typename T>
concept StepLike =
    requires { typename T::FieldEnum; } && FieldEnumLike<typename T::FieldEnum>;

// Ties one FieldEnum to its Traits and resolves the View type ViewFactory
// would build from them -- everything SimAllocator needs to host it.
// Hosting is all an Algorithm is for; what gets stepped each tick is a
// list of Steps (see StepLike above), named separately.
//
// EnumT and TraitsT are recovered later via partial specialization (see
// AlgorithmVisitor in sim_allocator.hpp) rather than stored as member
// aliases, since Traits is a template, not a value the type system can hand
// back out of an alias template.
template <FieldEnumLike EnumT, template <EnumT> class TraitsT>
struct Algorithm {
  using Enum = EnumT;
  using View = view::ViewFactory<EnumT, TraitsT>;
};

template <typename AlgorithmT>
concept AlgorithmLike = requires {
  typename AlgorithmT::Enum;
  typename AlgorithmT::View;
  requires FieldEnumLike<typename AlgorithmT::Enum>;
  requires view::ViewLike<typename AlgorithmT::View, typename AlgorithmT::Enum>;
};
}  // namespace achilles::engine