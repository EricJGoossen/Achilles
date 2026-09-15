#pragma once

#include <concepts>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <xsimd/xsimd.hpp>

#include "engine/field_contract.hpp"

namespace achilles::engine::view {

// A view supports field F at shape T (T is F's own scalar type for one
// plain instance, or an xsimd::batch of it for several lanes at once) if
// it exposes a matching Value<F, T> and a FieldCursor<F> whose Load<T>/
// Store<T> can gather/scatter by index. This is the exact per-shape
// surface OpInvoker relies on.
//
// Load/Store are checked through a *const* FieldCursor&, not a mutable one:
// OpInvoker::Invoke is itself const, so it reaches its cursors as const
// members -- a FieldCursor whose Load/Store aren't const-qualified would
// compile here but fail inside the real OpInvoker.
template <typename View, typename EnumT, EnumT F, typename T>
concept ViewFieldLikeForT =
    requires(View& view, size_t i, typename View::template Value<F, T> v) {
      typename View::template Value<F, T>;
      typename View::template FieldCursor<F>;
      {
        view.template Field<F>()
      } -> std::same_as<typename View::template FieldCursor<F>>;
      {
        std::declval<const typename View::template FieldCursor<F>&>()
            .template Load<T>(i)
      } -> std::same_as<typename View::template Value<F, T>>;
      {
        std::declval<const typename View::template FieldCursor<F>&>()
            .template Store<T>(i, v)
      } -> std::same_as<void>;
    };

// A view supports field F if it exposes F's own scalar type and supports F
// at both that scalar shape and the matching batched shape.
template <typename View, typename EnumT, EnumT F>
concept ViewFieldLike =
    requires { typename View::template Scalar<F>; } &&
    ViewFieldLikeForT<View, EnumT, F, typename View::template Scalar<F>> &&
    ViewFieldLikeForT<
        View,
        EnumT,
        F,
        xsimd::batch<typename View::template Scalar<F>>>;

template <typename View, typename EnumT, size_t... Is>
consteval bool AllFieldsViewLike(std::index_sequence<Is...>) {
  return (ViewFieldLike<View, EnumT, static_cast<EnumT>(Is)> && ...);
}

// Satisfied when View supports every field of EnumT (0 up to
// EnumT::kCount), per ViewFieldLike. This is the full surface OpInvoker
// needs from a view. FieldEnumLike is checked first so a missing/mistyped
// kCount fails there, by name, rather than inside the index_sequence
// below.
template <typename View, typename EnumT>
concept ViewLike =
    FieldEnumLike<EnumT> &&
    AllFieldsViewLike<View, EnumT>(
        std::make_index_sequence<static_cast<size_t>(EnumT::kCount)>{}
    );

}  // namespace achilles::engine::view