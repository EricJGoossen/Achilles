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

// One leaf: a record, read and written whole through the Layout (which
// decides where it sits and how to load it). A scalar is just the simplest
// record. Every Assembler tree bottoms out in these -- an arithmetic child
// listed in an Assembler's Children (float, RepeatTypes<3>, ...) is a
// LeafAssembler<float> -- and one can also be a whole field on its own:
// LeafAssembler<LidarConfig> is a field holding one LidarConfig per row,
// which an Op takes directly as `const LidarConfig&` / `LidarConfig*`.
//
// With several record types, each row holds one of them: Rs' k-th type in
// segment k of the field's ordering (see topology::SegmentedLayout, the
// layout such a field uses), so an Op written for segment k takes
// `const R_k&` / `R_k*`. ScalarType is the first record.
//
// A single arithmetic record reads as itself or as a batch of itself, and
// can be zeroed and accumulated into; any other record only reads as itself
// -- one instance at a time, no lanes.
template <typename... Rs>
  requires(
      sizeof...(Rs) > 0 && ((std::is_trivially_copyable_v<Rs> &&
                             std::is_default_constructible_v<Rs>) &&
                            ...)
  )
struct LeafAssembler {
  using Records = util::TypeList<Rs...>;
  using ScalarType = std::tuple_element_t<0, std::tuple<Rs...>>;
  static constexpr size_t kNumFields = 1;

 private:
  static constexpr bool kScalar =
      sizeof...(Rs) == 1 && std::is_arithmetic_v<ScalarType>;

  // The record a read or write as TargetScalar touches: itself, or a
  // batch's own scalar.
  template <typename TargetScalar>
  using LeafOf = std::conditional_t<
      util::kIsOneOfV<TargetScalar, Rs...>,
      TargetScalar,
      ScalarType>;

 public:
  template <typename TargetScalar>
    requires(util::kIsOneOfV<TargetScalar, Rs...> ||
             (kScalar && std::same_as<TargetScalar, xsimd::batch<ScalarType>>))
  using Value = TargetScalar;

  template <typename TargetScalar, typename Layout>
  static Value<TargetScalar> Read(
      const std::byte* field0, size_t /*field_stride_bytes*/
  ) {
    return Layout::template LoadLeaf<TargetScalar, LeafOf<TargetScalar>>(field0
    );
  }

  template <typename TargetScalar, typename Layout>
  static Value<TargetScalar> Broadcast(
      const std::byte* field0, size_t /*field_stride_bytes*/
  ) {
    return Layout::template BroadcastLeaf<TargetScalar, LeafOf<TargetScalar>>(
        field0
    );
  }

  template <typename TargetScalar, typename Layout>
  static void Write(
      std::byte* field0,
      size_t /*field_stride_bytes*/,
      const Value<TargetScalar>& value
  ) {
    Layout::template StoreLeaf<LeafOf<TargetScalar>>(field0, value);
  }

  template <typename TargetScalar>
    requires kScalar
  static Value<TargetScalar> Zero() {
    return TargetScalar(0);
  }

  template <typename TargetScalar, typename Layout>
    requires kScalar
  static void Accumulate(
      std::byte* field0,
      size_t /*field_stride_bytes*/,
      const Value<TargetScalar>& value
  ) {
    Layout::template AccumulateLeaf<ScalarType>(field0, value);
  }

  // What a padding row holds when this leaf is a whole field: a
  // value-initialized record. (Inside an Assembler tree the tree's own
  // Target decides -- see Assembler::PaddingSeed. A segmented field's
  // padding rows have no storage, so its seed is never written.)
  static ScalarType PaddingSeed() { return ScalarType{}; }
};

namespace assembler_detail {

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

}  // namespace assembler_detail

// A domain type's own statement of what's safe to write into a SIMD lane that
// no real Op will ever read meaningfully -- a padding row or the sink row.
// Lives on the type itself because only the type knows: Vector3::Zero() is
// inert, but Quaternion::Zero() is degenerate (normalizes to NaN), so the
// right answer is Identity() -- Quaternion is the only thing that should get
// to make that call, not a per-field enum guessing at it from outside.
template <typename Target>
concept SeedableLike = requires {
  { Target::PaddingSeed() } -> std::same_as<Target>;
};

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
  // (engine/topology/layout_policy.hpp) is how the leaves sit in storage:
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
  // through a topology::BroadcastIndex (engine/topology/indexing_policy.hpp).
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
  // through a topology::BroadcastIndex (engine/topology/indexing_policy.hpp).
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
      -> std::tuple<assembler_detail::ExpectedElemT<
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
      assembler_detail::AllMatchScalar<T, ChildTuple>(ChildIndices{}),
      "All leaves in an Assembler tree must match its scalar type T."
  );

  template <size_t Index>
  static constexpr size_t FieldOffset() {
    return assembler_detail::FieldOffset<ChildTuple, Index>::kValue;
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

// A domain type is assemblable by A when its own ToTuple() -- for both the
// scalar and batched TargetScalar -- returns exactly A::ExpectedTupleT:
// one element per child, matching what WriteImpl recurses through. This
// is deliberately a separate check from "does A::Write<T>(...) type-check
// as a call", because it isn't enough on its own: Write's return type is
// declared `void`, so checking that a call to it resolves never needs to
// instantiate Write's *body* -- and value.ToTuple() is read only inside
// that body, several calls deep through WriteImpl. A ToTuple()
// with the wrong shape (wrong arity, or wrapping/unwrapping a member
// incorrectly) compiled clean under the old check and only failed the
// first time something actually called Write for real, which could be a
// completely different translation unit. Checking the shape directly here
// catches that at the type's own static_assert instead.
template <typename A>
concept AssemblableLike = requires(
    typename A::template Value<typename A::ScalarType> value,
    typename A::template Value<xsimd::batch<typename A::ScalarType>> batch_value
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

// What a field can be read/written as: an Assembler's ScalarType or a
// batch of it -- which, for a LeafAssembler<R> field, is the whole R.
// Which of those a given field accepts, its Assembler's Value enforces.
template <typename T>
concept FieldShapeLike = std::is_trivially_copyable_v<T>;

// The value A reads one instance as: what a View hands an Op for a plain
// (unbatched) read of a field A assembles.
template <typename A>
using AssembledT = typename A::template Value<typename A::ScalarType>;

// What every field's Assembler must declare, whatever memory it's read
// from: empty (View never instantiates one; a non-static data member would
// silently do nothing), a scalar type and a field count, the Value it reads
// one instance as, and the value a padding row holds. Enough for a field
// that's one whole trivially-copyable record (LeafAssembler<R>), which has
// no lanes to batch and no children to tuple up. Whether
// it actually reads and writes through a given Layout is AssemblesThrough,
// checked where a field pairs the two.
template <typename A>
concept AssemblerLike = std::is_empty_v<A> && requires {
  typename A::ScalarType;
  { A::kNumFields } -> std::convertible_to<size_t>;
  requires A::kNumFields > 0;
  typename AssembledT<A>;
  { A::PaddingSeed() } -> std::same_as<AssembledT<A>>;
};

// A builds its value from children (an Assembler tree) rather than being
// a single leaf.
template <typename A>
concept DecomposesLike =
    requires { typename A::template ExpectedTupleT<typename A::ScalarType>; };

// An Assembler whose values can also be read lane-wide: AssemblerLike with
// a batched Value -- which only an arithmetic scalar has -- and, if it
// builds its value from children, a domain type that's AssemblableLike at
// both shapes (see there for why that shape check is what catches a
// malformed ToTuple() early). This is the surface a batched Op's View
// relies on.
template <typename A>
concept BatchAssemblerLike = AssemblerLike<A> && requires {
  typename A::template Value<xsimd::batch<typename A::ScalarType>>;
} && (!DecomposesLike<A> || AssemblableLike<A>);

// A reads and writes one instance through Layout: Read gives back its
// Value, and Write takes it. The pairing a field binding makes (see
// view::FieldBinding), and so where it's checked.
template <typename A, typename Layout>
concept AssemblesThrough =
    AssemblerLike<A> &&
    requires(const std::byte* read_ptr, std::byte* write_ptr, size_t stride) {
      {
        A::template Read<typename A::ScalarType, Layout>(read_ptr, stride)
      } -> std::same_as<AssembledT<A>>;
      {
        A::template Write<typename A::ScalarType, Layout>(
            write_ptr, stride, A::PaddingSeed()
        )
      } -> std::same_as<void>;
    };

}  // namespace achilles::engine
