#pragma once

#include <bit>
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <xsimd/xsimd.hpp>

#include "util/simd_ops.hpp"
#include "util/tmp.hpp"

namespace achilles::engine {

// Marker: N consecutive children.
// RepeatTypes<N> -- N copies of the enclosing Assembler's own T.
// RepeatTypes<N, ChildType> -- N copies of an explicit ChildType.
template <size_t N, typename ChildType = void>
struct RepeatTypes {};

// Marker: wraps a template-template parameter so it can appear in an
// Assembler's Children pack. Resolved to TT<ResolveT> during
// flattening, where ResolveT is the *immediately enclosing*
// Assembler's own scalar type T -- i.e. the wrapped assembler is
// instantiated with its parent's T, never specified at the use site.
template <template <typename> class TT>
struct Templated {};

namespace assembler_detail {

using util::Concat;
using util::ConcatT;
using util::RepeatList;
using util::RepeatListT;
using util::TypeList;

// -- Flattening (expands RepeatTypes in place) --

// ResolveT is the enclosing Assembler's own scalar type T, used to
// resolve a bare RepeatTypes<N>. Ordinary children ignore it.
template <typename ResolveT, typename Child>
struct FlattenOne {
  using Type = TypeList<Child>;
};

template <typename ResolveT, template <typename> class TT>
struct FlattenOne<ResolveT, Templated<TT>> {
  using Type = typename FlattenOne<ResolveT, TT<ResolveT>>::Type;
};

template <typename ResolveT, size_t N, typename ChildType>
struct FlattenOne<ResolveT, RepeatTypes<N, ChildType>> {
  using ElemType =
      std::conditional_t<std::is_void_v<ChildType>, ResolveT, ChildType>;
  using Type = RepeatListT<typename FlattenOne<ResolveT, ElemType>::Type, N>;
};

template <typename ResolveT, typename... Ts>
struct FlattenPack {
  using Type = ConcatT<typename FlattenOne<ResolveT, Ts>::Type...>;
};

template <typename ResolveT, typename... Ts>
using FlattenPackT = typename FlattenPack<ResolveT, Ts...>::Type;

using util::ToTuple;
using util::ToTupleT;

// -- Leaf/scalar/offset helpers (operate on the flattened list) --

template <typename T>
inline constexpr bool kIsLeaf = std::is_arithmetic_v<T>;

template <typename Child>
constexpr size_t ChildNumFields() {
  if constexpr (kIsLeaf<Child>) {
    return 1;
  } else {
    return Child::kNumFields;
  }
}

// Tag-dispatched so we never instantiate `Child::ScalarType` for a
// leaf (int::ScalarType would be ill-formed even under conditional_t).
template <typename Child, bool = kIsLeaf<Child>>
struct ChildScalarImpl;

template <typename Child>
struct ChildScalarImpl<Child, true> {
  using Type = Child;
};

template <typename Child>
struct ChildScalarImpl<Child, false> {
  using Type = typename Child::ScalarType;
};

template <typename Child>
using ChildScalarT = typename ChildScalarImpl<Child>::Type;

// Tag-dispatched for the same reason as ChildScalarImpl above:
// `Child::template Read<TargetScalar>` would be ill-formed for a leaf
// Child (float has no Read member), and conditional_t substitutes both
// of its type arguments regardless of which one it ends up selecting --
// it doesn't short-circuit the way `if constexpr` does in a function
// body.
template <typename TargetScalar, typename Child, bool = kIsLeaf<Child>>
struct ExpectedElemImpl;

template <typename TargetScalar, typename Child>
struct ExpectedElemImpl<TargetScalar, Child, true> {
  using Type = TargetScalar;
};

template <typename TargetScalar, typename Child>
struct ExpectedElemImpl<TargetScalar, Child, false> {
  using Type = decltype(Child::template Read<TargetScalar>(
      std::declval<const std::byte*>(), size_t{0}
  ));
};

template <typename TargetScalar, typename Child>
using ExpectedElemT = typename ExpectedElemImpl<TargetScalar, Child>::Type;

template <typename Tuple, size_t Index>
struct FieldOffset {
  static constexpr size_t kValue =
      FieldOffset<Tuple, Index - 1>::kValue +
      ChildNumFields<std::tuple_element_t<Index - 1, Tuple>>();
};

template <typename Tuple>
struct FieldOffset<Tuple, 0> {
  static constexpr size_t kValue = 0;
};

template <typename Tuple, size_t... Is>
constexpr size_t SumFields(std::index_sequence<Is...>) {
  return (ChildNumFields<std::tuple_element_t<Is, Tuple>>() + ...);
}

template <typename T, typename Tuple, size_t... Is>
constexpr bool AllMatchScalar(std::index_sequence<Is...>) {
  return (
      std::is_same_v<T, ChildScalarT<std::tuple_element_t<Is, Tuple>>> && ...
  );
}

}  // namespace assembler_detail

template <template <typename> class Target, typename T, typename... Children>
class Assembler {
  using ChildTuple = assembler_detail::ToTupleT<
      assembler_detail::FlattenPackT<T, Children...>>;

  static constexpr size_t kNumChildren = std::tuple_size_v<ChildTuple>;
  static_assert(kNumChildren > 0, "Assembler needs at least one child.");

  using ChildIndices = std::make_index_sequence<kNumChildren>;

 public:
  using ScalarType = T;

  static constexpr size_t kNumFields =
      assembler_detail::SumFields<ChildTuple>(ChildIndices{});

  // TargetScalar picks the shape to read/write as: this Assembler's own
  // ScalarType for one plain instance, or an xsimd::batch of it to gather
  // several lanes' worth at once. Any other TargetScalar is rejected --
  // the same underlying planar float storage backs both, so the two are
  // just different strides over it, never different data.
  template <typename TargetScalar>
  static auto Read(const std::byte* field0, size_t field_stride_bytes) {
    static_assert(
        std::is_same_v<TargetScalar, ScalarType> ||
            std::is_same_v<TargetScalar, xsimd::batch<ScalarType>>,
        "Read<TargetScalar> requires TargetScalar to be this "
        "Assembler's ScalarType or an xsimd::batch of it."
    );
    return ReadImpl<TargetScalar>(field0, field_stride_bytes, ChildIndices{});
  }

  template <typename TargetScalar, typename ValueT>
  static void Write(
      std::byte* field0, size_t field_stride_bytes, const ValueT& value
  ) {
    static_assert(
        std::is_same_v<TargetScalar, ScalarType> ||
            std::is_same_v<TargetScalar, xsimd::batch<ScalarType>>,
        "Write<TargetScalar> requires TargetScalar to be this "
        "Assembler's ScalarType or an xsimd::batch of it."
    );
    WriteImpl<TargetScalar>(
        field0, field_stride_bytes, value.ToTuple(), ChildIndices{}
    );
  }

 private:
  // Declared, never defined: named only inside decltype() in
  // ExpectedTupleT below, same trick OpInvokerBase uses for its own
  // cursor-tuple types. Has to come before ExpectedTupleT textually --
  // unlike a member function body, an alias's definition isn't a
  // complete-class context, so it can't forward-reference a member
  // declared later.
  template <typename TargetScalar, size_t... Is>
  static auto ExpectedTupleTypeImpl(std::index_sequence<Is...>)
      -> std::tuple<assembler_detail::ExpectedElemT<
          TargetScalar,
          std::tuple_element_t<Is, ChildTuple>>...>;

 public:
  // The exact tuple type Write<TargetScalar> needs `value.ToTuple()` to
  // return: one element per child, each either TargetScalar itself (a
  // leaf) or whatever that child's own Read<TargetScalar> produces (a
  // nested Assembler) -- WriteChild hands that element straight to
  // SimdStore or to the child's own Write, respectively. AssemblableLike
  // below checks a domain type's real ToTuple() against this instead of
  // merely checking that *a* call to Write type-checks -- see there for
  // why that distinction matters.
  template <typename TargetScalar>
  using ExpectedTupleT =
      decltype(ExpectedTupleTypeImpl<TargetScalar>(ChildIndices{}));

 private:
  static_assert(
      assembler_detail::AllMatchScalar<T, ChildTuple>(ChildIndices{}),
      "All leaves in an Assembler tree must match its scalar type T."
  );

  template <size_t Index>
  static constexpr size_t FieldOffset() {
    return assembler_detail::FieldOffset<ChildTuple, Index>::kValue;
  }

  template <typename Child, typename TargetScalar>
  static auto ReadChild(const std::byte* field0, size_t field_stride_bytes) {
    if constexpr (assembler_detail::kIsLeaf<Child>) {
      return util::SimdLoad<TargetScalar>(
          std::bit_cast<const Child*>(field0), 0
      );
    } else {
      return Child::template Read<TargetScalar>(field0, field_stride_bytes);
    }
  }

  template <typename Child, typename TargetScalar, typename ValueT>
  static void WriteChild(
      std::byte* field0, size_t field_stride_bytes, const ValueT& value
  ) {
    if constexpr (assembler_detail::kIsLeaf<Child>) {
      util::SimdStore(std::bit_cast<Child*>(field0), 0, value);
    } else {
      Child::template Write<TargetScalar>(field0, field_stride_bytes, value);
    }
  }

  template <typename TargetScalar, size_t... Is>
  static Target<TargetScalar>
  ReadImpl(const std::byte* field0, size_t field_stride_bytes, std::index_sequence<Is...>) {
    return Target<TargetScalar>(
        ReadChild<std::tuple_element_t<Is, ChildTuple>, TargetScalar>(
            field0 + FieldOffset<Is>() * field_stride_bytes, field_stride_bytes
        )...
    );
  }

  template <typename TargetScalar, typename Tuple, size_t... Is>
  static void
  WriteImpl(std::byte* field0, size_t field_stride_bytes, const Tuple& parts, std::index_sequence<Is...>) {
    (WriteChild<std::tuple_element_t<Is, ChildTuple>, TargetScalar>(
         field0 + FieldOffset<Is>() * field_stride_bytes,
         field_stride_bytes,
         std::get<Is>(parts)
     ),
     ...);
  }
};

// A domain type is assemblable by A when its own ToTuple() -- for both the
// scalar and batched TargetScalar -- returns exactly A::ExpectedTupleT:
// one element per child, matching what WriteChild recurses through. This
// is deliberately a separate check from "does A::Write<T>(...) type-check
// as a call", because it isn't enough on its own: Write's return type is
// declared `void`, so checking that a call to it resolves never needs to
// instantiate Write's *body* -- and value.ToTuple() is read only inside
// that body, several calls deep through WriteImpl/WriteChild. A ToTuple()
// with the wrong shape (wrong arity, or wrapping/unwrapping a member
// incorrectly) compiled clean under the old check and only failed the
// first time something actually called Write for real, which could be a
// completely different translation unit. Checking the shape directly here
// catches that at the type's own static_assert instead.
template <typename A>
concept AssemblableLike = requires(
    decltype(A::template Read<typename A::ScalarType>(
        std::declval<const std::byte*>(), size_t{0}
    )) value,
    decltype(A::template Read<xsimd::batch<typename A::ScalarType>>(
        std::declval<const std::byte*>(), size_t{0}
    )) batch_value
) {
  {
    value.ToTuple()
  }
  -> std::same_as<typename A::template ExpectedTupleT<typename A::ScalarType>>;
  {
    batch_value.ToTuple()
  } -> std::same_as<typename A::template ExpectedTupleT<
        xsimd::batch<typename A::ScalarType>>>;
};

// A well-formed Assembler is empty (PlanarView never instantiates one; a
// non-static data member would silently do nothing), exposes its scalar
// type and field count, its Read/Write pair round-trips (Write must accept
// whatever Read returns, for both the scalar and batched forms), and the
// domain type it reads/writes is AssemblableLike -- see there for why
// that's not implied by the round-trip check. This is the exact surface
// PlanarView relies on when it treats a template parameter as "an
// Assembler".
template <typename A>
concept AssemblerLike = std::is_empty_v<A> && requires {
  typename A::ScalarType;
  requires std::is_arithmetic_v<typename A::ScalarType>;
  { A::kNumFields } -> std::convertible_to<size_t>;
  requires A::kNumFields > 0;
} && requires(const std::byte* read_ptr, std::byte* write_ptr, size_t stride) {
  { A::template Read<typename A::ScalarType>(read_ptr, stride) };
  { A::template Read<xsimd::batch<typename A::ScalarType>>(read_ptr, stride) };
  {
    A::template Write<typename A::ScalarType>(
        write_ptr,
        stride,
        A::template Read<typename A::ScalarType>(read_ptr, stride)
    )
  } -> std::same_as<void>;
  {
    A::template Write<xsimd::batch<typename A::ScalarType>>(
        write_ptr,
        stride,
        A::template Read<xsimd::batch<typename A::ScalarType>>(read_ptr, stride)
    )
  } -> std::same_as<void>;
} && AssemblableLike<A>;

}  // namespace achilles::engine