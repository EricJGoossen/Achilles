#pragma once

#include <concepts>
#include <cstddef>
#include <type_traits>
#include <utility>

#include "assembler.hpp"

namespace achilles::engine {

// A well-formed field enum for PlanarView/ViewFactory: a scoped enum
// whose last enumerator is kCount, giving the number of real fields
// (0 .. kCount-1) and sizing the index-sequence machinery below. Exists
// so forgetting kCount (or mistyping it) fails with one clear diagnostic
// naming the enum, instead of a cryptic substitution failure deep inside
// PlanarView's static_assert or a ViewFactory Assembler instantiation.
template <typename EnumT>
concept FieldEnumLike = std::is_enum_v<EnumT> && requires {
  // Scoped enums (the intended EnumT) aren't implicitly convertible to
  // std::size_t -- every call site static_casts kCount explicitly, so
  // that's the check that matters here.
  static_cast<std::size_t>(EnumT::kCount);
};

// Traits<F>::Assembler exists and is AssemblerLike, for one field F.
template <typename EnumT, template <EnumT> class Traits, EnumT F>
concept HasFieldAssembler = requires { typename Traits<F>::Assembler; } &&
                            AssemblerLike<typename Traits<F>::Assembler>;

template <typename EnumT, template <EnumT> class Traits, size_t... Is>
consteval bool AllFieldsHaveAssemblers(std::index_sequence<Is...>) {
  return (HasFieldAssembler<EnumT, Traits, static_cast<EnumT>(Is)> && ...);
}

// A well-formed field-traits template: EnumT is a well-formed field enum
// (see FieldEnumLike), and Traits<F> supplies an AssemblerLike Assembler
// for every field F in [0, EnumT::kCount). The FieldEnumLike check comes
// first specifically so an EnumT missing kCount fails there, by name,
// rather than inside the index_sequence below -- && short-circuits, so
// EnumT::kCount is never substituted unless FieldEnumLike<EnumT> already
// held.
template <typename EnumT, template <EnumT> class Traits>
concept FieldTraitsLike =
    FieldEnumLike<EnumT> &&
    AllFieldsHaveAssemblers<EnumT, Traits>(
        std::make_index_sequence<static_cast<size_t>(EnumT::kCount)>{}
    );

}  // namespace achilles::engine
