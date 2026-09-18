#pragma once

#include <concepts>
#include <cstddef>

#include "engine/field_contract.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::engine {

// Sentinel Step for an Algorithm with no per-tick computation of its own --
// e.g. a field pool other algorithms only read/write via SharedAs, with
// nothing to step on its own account, or a test Algorithm that exists
// purely to exercise SimAllocator's memory-layout machinery. engine::pass::
// Step (engine/pass/sim_step.hpp) skips any Algorithm whose Step names this
// type, rather than every caller having to omit it from an Algorithms...
// pack by hand.
struct NoStep {};

// Ties one FieldEnum to its Traits and resolves the View type ViewFactory
// would build from them, plus (optionally) the type that knows how to step
// it -- StepT, a struct exposing a static Step(View, ExtraArgs...) that
// runs one full tick over a View of this shape (see algorithms/aba/
// aba_step.hpp's ABAStep for the real example). Defaults to NoStep: most
// Algorithms in a test or a shared-memory-only role never get stepped, so
// naming a real StepT is opt-in, not mandatory the way Traits is.
//
// EnumT and TraitsT are recovered later via partial specialization (see
// AlgorithmVisitor in sim_allocator.hpp) rather than stored as member
// aliases, since Traits is a template, not a value the type system can hand
// back out of an alias template.
template <
    FieldEnumLike EnumT,
    template <EnumT>
    class TraitsT,
    typename StepT = NoStep>
struct Algorithm {
  using Enum = EnumT;
  using View = view::ViewFactory<EnumT, TraitsT>;
  using Step = StepT;
};

template <typename AlgorithmT>
concept AlgorithmLike = requires {
  typename AlgorithmT::Enum;
  typename AlgorithmT::View;
  typename AlgorithmT::Step;
  requires FieldEnumLike<typename AlgorithmT::Enum>;
  requires view::ViewLike<typename AlgorithmT::View, typename AlgorithmT::Enum>;
};
}  // namespace achilles::engine