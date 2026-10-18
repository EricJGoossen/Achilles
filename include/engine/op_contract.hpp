#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <type_traits>
#include <utility>

#include "engine/field_contract.hpp"
#include "engine/view/view_contract.hpp"
#include "util/tmp.hpp"

namespace achilles::engine {

// Which of the indices a traversal hands an Op an argument is read from (or
// written to) -- a position, not a flag: OpInvoker packs every index a
// traversal hands it into one tuple, in this order, and an argument picks
// its own by std::get<ArgIndex>, never by a branch (see
// engine/pass/op_invoker.hpp). kTarget is always the row/group the
// traversal is visiting. kSource is the second index, whatever that
// traversal makes it -- kParent names the same slot under a TreeTraversal,
// which is the only place it means "parent"; a LinearTraversal hands the
// target again, and a PairTraversal hands the broadcast source row
// (engine/pass/traversals.hpp). kStructure is a third slot only some
// traversals fill: not an index at all, but a fact the traversal derived
// from its own topology about this one call (a PairTraversal's
// PairStructure) -- read through kTraversalInputs, never through a field.
enum class ArgIndex : uint8_t {
  kTarget = 0,
  kSource = 1,
  kParent = kSource,
  kStructure = 2,
};

// One argument to gather (or scatter) for an Op: `field` selects which
// field to read from (or write to) the View, and `index` selects which of
// the traversal's indices to do so at. A field whose own Indexing policy
// isn't per-row (e.g. topology::PairGroupIndexing) may resolve from every index
// regardless; see topology/indexing_policy.hpp.
template <FieldEnumLike EnumT>
struct ArgData {
  EnumT field;
  ArgIndex index;
};

// An Op optionally takes values straight from its traversal rather than
// from any field: `static constexpr std::array<ArgIndex, K>
// kTraversalInputs` names which slots of the traversal's index tuple to
// hand over as-is, as operator()'s parameters right after kInputs' (and
// before kOutputs'), each declared `const T&` for whatever type that slot
// holds. It's how structural facts only the traversal knows -- which lanes
// of a pair call are distinct, non-parent/child pairs, see
// pass::PairStructure -- reach an Op without being stored per row first.
// Naming a slot the traversal doesn't fill fails to compile at the call.
template <typename T>
inline constexpr size_t kTraversalInputCountV = 0;

template <typename T>
  requires requires {
    {
      T::kTraversalInputs
    } -> std::same_as<const std::array<ArgIndex, T::kTraversalInputs.size()>&>;
  }
inline constexpr size_t kTraversalInputCountV<T> = T::kTraversalInputs.size();

// An Op: operator() takes one `const P&` per kInputs entry (read-only),
// then one `const T&` per kTraversalInputs entry (if any), then one `P*`
// per kOutputs entry (read/write -- loaded before the call and stored after
// it, see OpInvokerBase in engine/pass/op_invoker.hpp).
template <typename T>
concept OpLike = requires(T a) {
  requires std::same_as<util::ReturnTypeOfT<decltype(&T::operator())>, void>;

  {
    T::kInputs
  } -> std::same_as<
        const std::array<ArgData<typename T::FieldEnum>, T::kInputs.size()>&>;
  {
    T::kOutputs
  } -> std::same_as<
        const std::array<ArgData<typename T::FieldEnum>, T::kOutputs.size()>&>;

  // Nested requirement: arbitrary bool constant-expression check
  requires util::kArityOfV<decltype(&T::operator())> ==
               T::kInputs.size() + kTraversalInputCountV<T> +
                   T::kOutputs.size();
};

// An Op optionally carries a second, narrower contract: Initialize, called
// once at the reserved base row (see TraversalLike::ApplyToBase) before a
// pass's real per-target Apply runs, to seed whatever that pass reads from
// its own parent index on the first (root) iteration -- the world
// transform/velocity a root joint's PropagateVelocityOp reads as
// x_world_parent/v_parent, the zeroed articulated-inertia accumulator
// PropagateInertiaOp accumulates into, and so on (see
// algorithms/aba/aba_ops.hpp for the real examples). Not every Op needs
// this -- OpHasInit, not OpLike, gates it -- so op_invoker.hpp can build
// Initialize-specific cursors only for the Ops that actually declare
// kInitInputs/kInitOutputs/Initialize, the same way OpLike's shape is
// checked without requiring every Op to also be OpHasInit.
template <typename T>
concept OpHasInit = requires(T a) {
  requires std::same_as<util::ReturnTypeOfT<decltype(&T::Initialize)>, void>;

  {
    T::kInitInputs
  }
  -> std::same_as<
      const std::array<ArgData<typename T::FieldEnum>, T::kInitInputs.size()>&>;
  {
    T::kInitOutputs
  } -> std::same_as<const std::array<
        ArgData<typename T::FieldEnum>,
        T::kInitOutputs.size()>&>;

  requires util::kArityOfV<decltype(&T::Initialize)> ==
               T::kInitInputs.size() + T::kInitOutputs.size();
};

// operator()'s I-th input is expected to be `const P&` where P is some
// domain type that names its own scalar via `P::ScalarType` -- that's the T
// an Op wants field kInputs[I].field read as. Checked with `if constexpr`/
// `requires` throughout (rather than asserting directly) so a malformed
// operator() -- wrong reference-ness, or a parameter type with no
// ScalarType -- makes this consteval function return false instead of
// hard-erroring, keeping OpArgsMatchView usable as an ordinary bool.
template <typename Op, typename View, size_t I>
consteval bool InputArgMatchesView() {
  using Declared =
      std::tuple_element_t<I, util::ArgsOfT<decltype(&Op::operator())>>;
  if constexpr (!std::is_lvalue_reference_v<Declared> ||
                !std::is_const_v<std::remove_reference_t<Declared>>) {
    return false;
  } else {
    using Plain = std::remove_cvref_t<Declared>;
    if constexpr (!requires { typename Plain::ScalarType; }) {
      return false;
    } else {
      return std::is_same_v<
          Declared,
          const typename View::template Value<
              Op::kInputs[I].field,
              typename Plain::ScalarType>&>;
    }
  }
}

// Same idea for the J-th output, which is expected to be `P*` -- after
// every kInputs and kTraversalInputs parameter.
template <typename Op, typename View, size_t J>
consteval bool OutputArgMatchesView() {
  using Declared = std::tuple_element_t<
      Op::kInputs.size() + kTraversalInputCountV<Op> + J,
      util::ArgsOfT<decltype(&Op::operator())>>;
  if constexpr (!std::is_pointer_v<Declared>) {
    return false;
  } else {
    using Plain = std::remove_pointer_t<Declared>;
    if constexpr (!requires { typename Plain::ScalarType; }) {
      return false;
    } else {
      return std::is_same_v<
          Declared,
          typename View::template Value<
              Op::kOutputs[J].field,
              typename Plain::ScalarType>*>;
    }
  }
}

template <typename Op, typename View, size_t... InIs, size_t... OutIs>
consteval bool
ArgTypesMatchViewImpl(std::index_sequence<InIs...>, std::index_sequence<OutIs...>) {
  return (InputArgMatchesView<Op, View, InIs>() && ...) &&
         (OutputArgMatchesView<Op, View, OutIs>() && ...);
}

// Same two checks again, against Initialize/kInitInputs/kInitOutputs
// instead of operator()/kInputs/kOutputs -- kept as their own functions
// (rather than generalizing InputArgMatchesView/OutputArgMatchesView to
// take "which member" and "which arrays" as extra parameters) to match
// how OpInvokerBase/OpInitInvoker already keep the two paths separate in
// op_invoker.hpp.
template <typename Op, typename View, size_t I>
consteval bool InitInputArgMatchesView() {
  using Declared =
      std::tuple_element_t<I, util::ArgsOfT<decltype(&Op::Initialize)>>;
  if constexpr (!std::is_lvalue_reference_v<Declared> ||
                !std::is_const_v<std::remove_reference_t<Declared>>) {
    return false;
  } else {
    using Plain = std::remove_cvref_t<Declared>;
    if constexpr (!requires { typename Plain::ScalarType; }) {
      return false;
    } else {
      return std::is_same_v<
          Declared,
          const typename View::template Value<
              Op::kInitInputs[I].field,
              typename Plain::ScalarType>&>;
    }
  }
}

template <typename Op, typename View, size_t J>
consteval bool InitOutputArgMatchesView() {
  using Declared = std::tuple_element_t<
      Op::kInitInputs.size() + J,
      util::ArgsOfT<decltype(&Op::Initialize)>>;
  if constexpr (!std::is_pointer_v<Declared>) {
    return false;
  } else {
    using Plain = std::remove_pointer_t<Declared>;
    if constexpr (!requires { typename Plain::ScalarType; }) {
      return false;
    } else {
      return std::is_same_v<
          Declared,
          typename View::template Value<
              Op::kInitOutputs[J].field,
              typename Plain::ScalarType>*>;
    }
  }
}

template <typename Op, typename View, size_t... InIs, size_t... OutIs>
consteval bool
InitArgTypesMatchViewImpl(std::index_sequence<InIs...>, std::index_sequence<OutIs...>) {
  return (InitInputArgMatchesView<Op, View, InIs>() && ...) &&
         (InitOutputArgMatchesView<Op, View, OutIs>() && ...);
}

// Only meaningful for an Op that actually has Init -- an Op without one
// trivially passes, the same way an absent kInitInputs/kInitOutputs
// simply has nothing to check.
template <typename Op, typename View>
consteval bool InitArgsMatchViewIfPresent() {
  if constexpr (OpHasInit<Op>) {
    return InitArgTypesMatchViewImpl<Op, View>(
        std::make_index_sequence<Op::kInitInputs.size()>{},
        std::make_index_sequence<Op::kInitOutputs.size()>{}
    );
  } else {
    return true;
  }
}

// An Op is well-paired with a View when operator()'s parameter types, in
// declaration order, match what kInputs/kOutputs says they should be: a
// `const P&` for each kInputs entry and a `P*` for each kOutputs entry,
// where P is exactly what reading (or writing) that field as P's own
// ScalarType produces from the View. Nothing about OpLike or the ArgData
// arrays otherwise enforces this -- reordering kInputs (or kOutputs)
// without reordering operator()'s parameters the same way compiles fine
// and silently pairs the wrong field with the wrong parameter, and nothing
// stops a field's Assembler from producing a type that merely happens to
// have the right size but is semantically the wrong thing. Checking here
// turns both mistakes into a compile error at the point an Op is paired
// with a concrete View. Because P supplies its own ScalarType, this works
// unchanged whether an Op mixes scalar and batched fields or uses only one
// -- there is no separate "batched" version of this check.
template <typename Op, typename View>
concept OpArgsMatchView =
    OpLike<Op> && view::ViewLike<View, typename Op::FieldEnum> &&
    ArgTypesMatchViewImpl<Op, View>(
        std::make_index_sequence<Op::kInputs.size()>{},
        std::make_index_sequence<Op::kOutputs.size()>{}
    ) &&
    InitArgsMatchViewIfPresent<Op, View>();

}  // namespace achilles::engine