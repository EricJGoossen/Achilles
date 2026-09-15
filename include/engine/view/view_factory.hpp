#pragma once

#include <cstddef>

#include "engine/field_contract.hpp"

namespace achilles::engine::view {

template <
    template <typename, typename...>
    class View,
    typename EnumT,
    template <EnumT>
    class Traits,
    size_t... Is>
auto MakeViewFromTraits(std::index_sequence<Is...>)
    -> View<EnumT, typename Traits<static_cast<EnumT>(Is)>::Assembler...>;

template <
    template <typename, typename...>
    class View,
    typename EnumT,
    template <EnumT>
    class Traits>
  requires FieldTraitsLike<EnumT, Traits>
using ViewFactory = decltype(MakeViewFromTraits<View, EnumT, Traits>(
    std::make_index_sequence<static_cast<size_t>(EnumT::kCount)>{}
));

}  // namespace achilles::engine::view