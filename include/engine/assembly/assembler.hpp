// Converts domain types to and from flat scalar leaves.
// Doc: docs/code/engine_assembly_assembler.md

#pragma once

#include <bit>
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <xsimd/xsimd.hpp>

#include "engine/assembly/assembler_contract.hpp"
#include "engine/assembly/leaf_assembler.hpp"
#include "util/simd_ops.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::assembly {

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

namespace detail {

using util::Concat;
using util::ConcatT;
using util::RepeatList;
using util::RepeatListT;
using util::TypeList;

// -- Flattening (expands RepeatTypes in place) --

// A child as an assembler: an arithmetic child is its LeafAssembler, a
// nested Assembler is itself -- so after flattening, every child is
// handled the same way.
template <typename Child>
struct ChildAssembler {
  using Type = Child;
};
template <typename Child>
  requires std::is_arithmetic_v<Child>
struct ChildAssembler<Child> {
  using Type = LeafAssembler<Child>;
};

// ResolveT is the enclosing Assembler's own scalar type T, used to
// resolve a bare RepeatTypes<N>. Ordinary children ignore it.
template <typename ResolveT, typename Child>
struct FlattenOne {
  using Type = TypeList<typename ChildAssembler<Child>::Type>;
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

// -- Scalar/offset helpers (operate on the flattened list, every element
// an assembler) --

template <typename TargetScalar, typename Child>
using ExpectedElemT = typename Child::template Value<TargetScalar>;

template <typename Tuple, size_t Index>
struct FieldOffset {
  static constexpr size_t kValue =
      FieldOffset<Tuple, Index - 1>::kValue +
      std::tuple_element_t<Index - 1, Tuple>::kNumFields;
};

template <typename Tuple>
struct FieldOffset<Tuple, 0> {
  static constexpr size_t kValue = 0;
};

template <typename Tuple, size_t... Is>
constexpr size_t SumFields(std::index_sequence<Is...>) {
  return (std::tuple_element_t<Is, Tuple>::kNumFields + ...);
}

template <typename T, typename Tuple, size_t... Is>
constexpr bool AllMatchScalar(std::index_sequence<Is...>) {
  return (
      std::is_same_v<T, typename std::tuple_element_t<Is, Tuple>::ScalarType> &&
      ...
  );
}

}  // namespace detail

template <template <typename> class Target, typename T, typename... Children>
class Assembler {
  using ChildTuple = detail::ToTupleT<detail::FlattenPackT<T, Children...>>;

  static constexpr size_t kNumChildren = std::tuple_size_v<ChildTuple>;
  static_assert(kNumChildren > 0, "Assembler needs at least one child.");

  using ChildIndices = std::make_index_sequence<kNumChildren>;

 public:
  using ScalarType = T;

  static constexpr size_t kNumFields =
      detail::SumFields<ChildTuple>(ChildIndices{});

  // What Read/Broadcast/Zero produce at TargetScalar: Target built at it.
  // Only this Assembler's ScalarType or an xsimd::batch of it -- the same
  // underlying storage backs both, so the two are just different reads of
  // it, never different data. Declared here, so anything that needs the
  // type (a View, the concepts below) names it without instantiating Read.
  template <typename TargetScalar>
    requires(std::same_as<TargetScalar, T> ||
             std::same_as<TargetScalar, xsimd::batch<T>>)
  using Value = Target<TargetScalar>;

  // TargetScalar picks the shape to read/write as (see Value): one plain
  // instance, or a batch gathering several lanes' worth at once. Layout
  // (policy/layout/) is how the leaves sit in storage:
  // this class only knows which leaves a Target has, and asks Layout where
  // each one is and how to load it -- all resolved at compile time.
  template <typename TargetScalar, typename Layout>
  static Value<TargetScalar> Read(
      const std::byte* field0, size_t field_stride_bytes
  ) {
    return ReadImpl<TargetScalar, Layout, false>(
        field0, field_stride_bytes, ChildIndices{}
    );
  }

  // Read's broadcasting counterpart: reads the one instance at field0 and
  // hands every lane of TargetScalar that same value, leaf by leaf (see
  // util::SimdBroadcast) -- what a View hands an Op for an argument read
  // through a policy::indexing::BroadcastIndex (policy/indexing/).
  // Same TargetScalar restriction and same result type as Read, so an Op
  // never sees any difference between a gathered and a broadcast argument.
  template <typename TargetScalar, typename Layout>
  static Value<TargetScalar> Broadcast(
      const std::byte* field0, size_t field_stride_bytes
  ) {
    return ReadImpl<TargetScalar, Layout, true>(
        field0, field_stride_bytes, ChildIndices{}
    );
  }

  template <typename TargetScalar, typename Layout, typename ValueT>
  static void Write(
      std::byte* field0, size_t field_stride_bytes, const ValueT& value
  ) {
    static_assert(
        std::is_same_v<TargetScalar, ScalarType> ||
            std::is_same_v<TargetScalar, xsimd::batch<ScalarType>>,
        "Write<TargetScalar> requires TargetScalar to be this "
        "Assembler's ScalarType or an xsimd::batch of it."
    );
    WriteImpl<TargetScalar, Layout>(
        field0, field_stride_bytes, value.ToTuple(), ChildIndices{}
    );
  }

  // A value with every leaf TargetScalar(0) -- what an accumulating output
  // (one stored through Accumulate below) starts from, so an Op adding
  // onto it adds onto nothing.
  template <typename TargetScalar>
  static Value<TargetScalar> Zero() {
    return ZeroImpl<TargetScalar>(ChildIndices{});
  }

  // Write's reducing counterpart: adds the sum of every lane of `value`
  // into the one instance at field0, leaf by leaf (see
  // util::SimdAccumulate) -- what a View does with an output stored
  // through a policy::indexing::BroadcastIndex (policy/indexing/).
  template <typename TargetScalar, typename Layout, typename ValueT>
  static void Accumulate(
      std::byte* field0, size_t field_stride_bytes, const ValueT& value
  ) {
    static_assert(
        std::is_same_v<TargetScalar, ScalarType> ||
            std::is_same_v<TargetScalar, xsimd::batch<ScalarType>>,
        "Accumulate<TargetScalar> requires TargetScalar to be this "
        "Assembler's ScalarType or an xsimd::batch of it."
    );
    AccumulateImpl<TargetScalar, Layout>(
        field0, field_stride_bytes, value.ToTuple(), ChildIndices{}
    );
  }

  // What a padding row holds: Target's own PaddingSeed (see SeedableLike
  // below -- only the type knows what's inert for it).
  static Target<ScalarType> PaddingSeed()
    requires SeedableLike<Target<ScalarType>>
  {
    return Target<ScalarType>::PaddingSeed();
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
      -> std::tuple<detail::ExpectedElemT<
          TargetScalar,
          std::tuple_element_t<Is, ChildTuple>>...>;

 public:
  // The exact tuple type Write<TargetScalar> needs `value.ToTuple()` to
  // return: one element per child, each that child's own Value at
  // TargetScalar (TargetScalar itself for a leaf) -- WriteImpl hands each
  // element straight to that child's own Write. AssemblableLike
  // below checks a domain type's real ToTuple() against this instead of
  // merely checking that *a* call to Write type-checks -- see there for
  // why that distinction matters.
  template <typename TargetScalar>
  using ExpectedTupleT =
      decltype(ExpectedTupleTypeImpl<TargetScalar>(ChildIndices{}));

 private:
  static_assert(
      detail::AllMatchScalar<T, ChildTuple>(ChildIndices{}),
      "All leaves in an Assembler tree must match its scalar type T."
  );

  template <size_t Index>
  static constexpr size_t FieldOffset() {
    return detail::FieldOffset<ChildTuple, Index>::kValue;
  }

  // A child's gather or splat read -- every child is an assembler (a leaf
  // or a nested Assembler), so the only choice is which of its reads.
  template <typename Child, typename TargetScalar, typename Layout, bool Splat>
    requires(!Splat)
  static auto ReadChild(const std::byte* field0, size_t field_stride_bytes) {
    return Child::template Read<TargetScalar, Layout>(
        field0, field_stride_bytes
    );
  }
  template <typename Child, typename TargetScalar, typename Layout, bool Splat>
    requires(Splat)
  static auto ReadChild(const std::byte* field0, size_t field_stride_bytes) {
    return Child::template Broadcast<TargetScalar, Layout>(
        field0, field_stride_bytes
    );
  }

  // A Target with its own Zero() builds it that way: a type whose
  // constructor validates its parts (Inertia's physical-validity assert)
  // rejects all-zero parts, but its own Zero() is exactly every leaf zero,
  // built without that check. Anything else is built from zero children.
  template <typename TargetScalar, size_t... Is>
    requires requires { Target<TargetScalar>::Zero(); }
  static Target<TargetScalar> ZeroImpl(std::index_sequence<Is...>) {
    return Target<TargetScalar>::Zero();
  }
  template <typename TargetScalar, size_t... Is>
  static Target<TargetScalar> ZeroImpl(std::index_sequence<Is...>) {
    return Target<TargetScalar>(
        std::tuple_element_t<Is, ChildTuple>::template Zero<TargetScalar>()...
    );
  }

  template <
      typename TargetScalar,
      typename Layout,
      typename Tuple,
      size_t... Is>
  static void
  AccumulateImpl(std::byte* field0, size_t field_stride_bytes, const Tuple& parts, std::index_sequence<Is...>) {
    (std::tuple_element_t<Is, ChildTuple>::
         template Accumulate<TargetScalar, Layout>(
             Layout::LeafAt(field0, FieldOffset<Is>(), field_stride_bytes),
             field_stride_bytes,
             std::get<Is>(parts)
         ),
     ...);
  }

  template <typename TargetScalar, typename Layout, bool Splat, size_t... Is>
  static Target<TargetScalar>
  ReadImpl(const std::byte* field0, size_t field_stride_bytes, std::index_sequence<Is...>) {
    return Target<TargetScalar>(ReadChild<
                                std::tuple_element_t<Is, ChildTuple>,
                                TargetScalar,
                                Layout,
                                Splat>(
        Layout::LeafAt(field0, FieldOffset<Is>(), field_stride_bytes),
        field_stride_bytes
    )...);
  }

  template <
      typename TargetScalar,
      typename Layout,
      typename Tuple,
      size_t... Is>
  static void
  WriteImpl(std::byte* field0, size_t field_stride_bytes, const Tuple& parts, std::index_sequence<Is...>) {
    (std::tuple_element_t<Is, ChildTuple>::template Write<TargetScalar, Layout>(
         Layout::LeafAt(field0, FieldOffset<Is>(), field_stride_bytes),
         field_stride_bytes,
         std::get<Is>(parts)
     ),
     ...);
  }
};

}  // namespace achilles::engine::assembly
