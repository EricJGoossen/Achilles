// An algorithm: a field enum and its traits, and the view they resolve to.

#pragma once

#include <concepts>
#include <cstddef>

#include "engine/spec/field_binding.hpp"
#include "engine/spec/field_contract.hpp"
#include "engine/spec/step_contract.hpp"
#include "engine/spec/view_contract.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::engine {

// Ties one FieldEnum to its Traits and resolves the View type ViewFactory
// would build from them -- everything SimAllocator needs to host it.
// Hosting is all an Algorithm is for; what gets stepped each tick is a
// list of Steps (see StepLike above), named separately.
//
// EnumT and TraitsT are recovered later via partial specialization (see
// AlgorithmVisitor in sim_allocator.hpp) rather than stored as member
// aliases, since Traits is a template, not a value the type system can hand
// back out of an alias template.
template <spec::FieldEnumLike EnumT, template <EnumT> class TraitsT>
struct Algorithm {
  using Enum = EnumT;
  using View = view::ViewFactory<EnumT, TraitsT>;
};

template <typename AlgorithmT>
concept AlgorithmLike = requires {
  typename AlgorithmT::Enum;
  typename AlgorithmT::View;
  requires spec::FieldEnumLike<typename AlgorithmT::Enum>;
  requires spec::ViewLike<typename AlgorithmT::View, typename AlgorithmT::Enum>;
};

}  // namespace achilles::engine
