#pragma once

#include <cstddef>
#include <utility>

#include "engine/field_contract.hpp"
#include "engine/memory/binding.hpp"
#include "engine/view/view.hpp"

namespace achilles::engine::view {

// Builds one FieldBinding<Policy, Assembler> per field: Assembler from
// Traits<F>::Assembler and Policy from Traits<F>::Layout, both exactly as
// Traits<F> declares them, and instantiates View<EnumT, Bindings...> from
// them. View itself is never a parameter here: it's the one view class
// that exists, and which Layout policy each field declares is already what
// determines its shape -- there's nothing left for a caller to choose
// between. This is the only place a field's Traits specialization gets
// translated into what View stores per field, so binding a field to a
// different Layout policy is a one-line change to that field's own
// Traits<F> -- nothing here or in View changes further.
template <typename EnumT, template <EnumT> class Traits, size_t... Is>
auto MakeViewFromTraits(std::index_sequence<Is...>)
    -> View<
        EnumT,
        FieldBinding<
            memory::FieldLayoutT<EnumT, Traits, static_cast<EnumT>(Is)>,
            typename Traits<static_cast<EnumT>(Is)>::Assembler>...>;

template <typename EnumT, template <EnumT> class Traits>
  requires FieldTraitsLike<EnumT, Traits>
using ViewFactory = decltype(MakeViewFromTraits<EnumT, Traits>(
    std::make_index_sequence<static_cast<size_t>(EnumT::kCount)>{}
));

}  // namespace achilles::engine::view
