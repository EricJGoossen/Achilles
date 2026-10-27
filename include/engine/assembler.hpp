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

// Stands in for a real Layout (engine/topology/layout_policy.hpp) wherever
// only the *type* an Assembler reads is wanted -- decltype, the concepts
// below: what a Read returns never depends on where its leaves sit. Same
// surface as a Layout's leaf access; declared, never defined.
struct LayoutProbe {
  static const std::byte* LeafAt(const std::byte* element, size_t, size_t);
  static std::byte* LeafAt(std::byte* element, size_t, size_t);
  template <typename TargetScalar, typename Leaf>
  static TargetScalar LoadLeaf(const std::byte* leaf);
  template <typename TargetScalar, typename Leaf>
  static TargetScalar BroadcastLeaf(const std::byte* leaf);
  template <typename Leaf, typename ValueT>
  static void StoreLeaf(std::byte* leaf, const ValueT& value);
  template <typename Leaf, typename ValueT>
  static void AccumulateLeaf(std::byte* leaf, const ValueT& value);
};

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
  using Type = decltype(Child::template Read<TargetScalar, LayoutProbe>(
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

  // TargetScalar picks the shape to read/write as: this Assembler's own
  // ScalarType for one plain instance, or an xsimd::batch of it to gather
  // several lanes' worth at once. Any other TargetScalar is rejected --
  // the same underlying storage backs both, so the two are just different
  // reads of it, never different data. Layout (engine/topology/
  // layout_policy.hpp) is how the leaves sit in that storage: this class
  // only knows which leaves a Target has, and asks Layout where each one
  // is and how to load it -- all resolved at compile time.
  template <typename TargetScalar, typename Layout>
  static auto Read(const std::byte* field0, size_t field_stride_bytes) {
    static_assert(
        std::is_same_v<TargetScalar, ScalarType> ||
            std::is_same_v<TargetScalar, xsimd::batch<ScalarType>>,
        "Read<TargetScalar> requires TargetScalar to be this "
        "Assembler's ScalarType or an xsimd::batch of it."
    );
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
  static auto Broadcast(const std::byte* field0, size_t field_stride_bytes) {
    static_assert(
        std::is_same_v<TargetScalar, ScalarType> ||
            std::is_same_v<TargetScalar, xsimd::batch<ScalarType>>,
        "Broadcast<TargetScalar> requires TargetScalar to be this "
        "Assembler's ScalarType or an xsimd::batch of it."
    );
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
  static auto Zero() {
    static_assert(
        std::is_same_v<TargetScalar, ScalarType> ||
            std::is_same_v<TargetScalar, xsimd::batch<ScalarType>>,
        "Zero<TargetScalar> requires TargetScalar to be this "
        "Assembler's ScalarType or an xsimd::batch of it."
    );
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

  // One overload per (leaf or nested Assembler) x (gather or splat) case,
  // picked by its requires-clause: a leaf is loaded through Layout, a
  // nested Assembler recurses into its own Read/Broadcast.
  template <typename Child, typename TargetScalar, typename Layout, bool Splat>
    requires(assembler_detail::kIsLeaf<Child> && !Splat)
  static auto
  ReadChild(const std::byte* field0, size_t /*field_stride_bytes*/) {
    return Layout::template LoadLeaf<TargetScalar, Child>(field0);
  }
  template <typename Child, typename TargetScalar, typename Layout, bool Splat>
    requires(assembler_detail::kIsLeaf<Child> && Splat)
  static auto
  ReadChild(const std::byte* field0, size_t /*field_stride_bytes*/) {
    return Layout::template BroadcastLeaf<TargetScalar, Child>(field0);
  }
  template <typename Child, typename TargetScalar, typename Layout, bool Splat>
    requires(!assembler_detail::kIsLeaf<Child> && !Splat)
  static auto ReadChild(const std::byte* field0, size_t field_stride_bytes) {
    return Child::template Read<TargetScalar, Layout>(
        field0, field_stride_bytes
    );
  }
  template <typename Child, typename TargetScalar, typename Layout, bool Splat>
    requires(!assembler_detail::kIsLeaf<Child> && Splat)
  static auto ReadChild(const std::byte* field0, size_t field_stride_bytes) {
    return Child::template Broadcast<TargetScalar, Layout>(
        field0, field_stride_bytes
    );
  }

  template <
      typename Child,
      typename TargetScalar,
      typename Layout,
      typename ValueT>
    requires(assembler_detail::kIsLeaf<Child>)
  static void WriteChild(
      std::byte* field0, size_t /*field_stride_bytes*/, const ValueT& value
  ) {
    Layout::template StoreLeaf<Child>(field0, value);
  }
  template <
      typename Child,
      typename TargetScalar,
      typename Layout,
      typename ValueT>
    requires(!assembler_detail::kIsLeaf<Child>)
  static void WriteChild(
      std::byte* field0, size_t field_stride_bytes, const ValueT& value
  ) {
    Child::template Write<TargetScalar, Layout>(
        field0, field_stride_bytes, value
    );
  }

  template <typename Child, typename TargetScalar>
    requires(assembler_detail::kIsLeaf<Child>)
  static auto ZeroChild() {
    return TargetScalar(0);
  }
  template <typename Child, typename TargetScalar>
    requires(!assembler_detail::kIsLeaf<Child>)
  static auto ZeroChild() {
    return Child::template Zero<TargetScalar>();
  }

  template <
      typename Child,
      typename TargetScalar,
      typename Layout,
      typename ValueT>
    requires(assembler_detail::kIsLeaf<Child>)
  static void AccumulateChild(
      std::byte* field0, size_t /*field_stride_bytes*/, const ValueT& value
  ) {
    Layout::template AccumulateLeaf<Child>(field0, value);
  }
  template <
      typename Child,
      typename TargetScalar,
      typename Layout,
      typename ValueT>
    requires(!assembler_detail::kIsLeaf<Child>)
  static void AccumulateChild(
      std::byte* field0, size_t field_stride_bytes, const ValueT& value
  ) {
    Child::template Accumulate<TargetScalar, Layout>(
        field0, field_stride_bytes, value
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
        ZeroChild<std::tuple_element_t<Is, ChildTuple>, TargetScalar>()...
    );
  }

  template <
      typename TargetScalar,
      typename Layout,
      typename Tuple,
      size_t... Is>
  static void
  AccumulateImpl(std::byte* field0, size_t field_stride_bytes, const Tuple& parts, std::index_sequence<Is...>) {
    (AccumulateChild<
         std::tuple_element_t<Is, ChildTuple>,
         TargetScalar,
         Layout>(
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
    (WriteChild<std::tuple_element_t<Is, ChildTuple>, TargetScalar, Layout>(
         Layout::LeafAt(field0, FieldOffset<Is>(), field_stride_bytes),
         field_stride_bytes,
         std::get<Is>(parts)
     ),
     ...);
  }
};

// Assembles a whole trivially-copyable record R -- a sensor's configuration,
// its persistent noise state, ... -- as a field value: one leaf, the record
// itself, rather than a tree of arithmetic leaves. Where that leaf sits is
// the Layout's business like any other (under topology::PlanarLayout: one
// contiguous array of R). R is its own ScalarType, so an Op takes the
// record directly (`const R&` / `R*`) and a field holding one is read one
// instance at a time: there are no lanes to batch, so it's AssemblerLike
// but never BatchAssemblerLike.
template <typename R>
  requires(std::is_trivially_copyable_v<R> && std::is_default_constructible_v<R>)
struct RecordAssembler {
  using ScalarType = R;
  static constexpr size_t kNumFields = 1;

  template <std::same_as<R> TargetScalar, typename Layout>
  static R Read(const std::byte* field0, size_t /*field_stride_bytes*/) {
    return Layout::template LoadLeaf<R, R>(field0);
  }

  template <std::same_as<R> TargetScalar, typename Layout>
  static void Write(
      std::byte* field0, size_t /*field_stride_bytes*/, const R& record
  ) {
    Layout::template StoreLeaf<R>(field0, record);
  }

  // What a padding row holds: a value-initialized record.
  static R PaddingSeed() { return R{}; }
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
    decltype(A::template Read<
             typename A::ScalarType,
             assembler_detail::LayoutProbe>(
        std::declval<const std::byte*>(), size_t{0}
    )) value,
    decltype(A::template Read<
             xsimd::batch<typename A::ScalarType>,
             assembler_detail::LayoutProbe>(
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

// What a field can be read/written as: an Assembler's ScalarType or a
// batch of it -- which, for a RecordAssembler field, is the whole record.
// Which of those a given field accepts, its Assembler's Read/Write enforce.
template <typename T>
concept FieldShapeLike = std::is_trivially_copyable_v<T>;

// The value A reads one instance as: what a View hands an Op for a plain
// (unbatched) read of a field A assembles.
template <typename A>
using AssembledT = decltype(A::template Read<
                            typename A::ScalarType,
                            assembler_detail::LayoutProbe>(
    std::declval<const std::byte*>(), size_t{0}
));

// What every field's Assembler must offer: empty (View never instantiates
// one; a non-static data member would silently do nothing), a scalar type
// and a field count, a Read/Write pair at that scalar that round-trips
// through any Layout, and the value a padding row holds. That's all
// SimAllocator and a View reading one instance at a time rely on -- enough
// for a field whose one leaf is a whole trivially-copyable record (see
// RecordAssembler above), which has no lanes to
// batch and no leaves to tuple up.
template <typename A>
concept AssemblerLike = std::is_empty_v<A> && requires {
  typename A::ScalarType;
  { A::kNumFields } -> std::convertible_to<size_t>;
  requires A::kNumFields > 0;
} && requires(const std::byte* read_ptr, std::byte* write_ptr, size_t stride) {
  {
    A::template Write<typename A::ScalarType, assembler_detail::LayoutProbe>(
        write_ptr,
        stride,
        A::template Read<typename A::ScalarType, assembler_detail::LayoutProbe>(
            read_ptr, stride
        )
    )
  } -> std::same_as<void>;
  { A::PaddingSeed() } -> std::same_as<AssembledT<A>>;
};

// An Assembler whose values can also be read lane-wide: AssemblerLike over
// an arithmetic scalar, plus the batched Read/Write pair (Write must accept
// whatever Read returns), and the domain type it reads/writes is
// AssemblableLike -- see there for why that's not implied by the round-
// trip check. This is the surface a batched Op's View relies on.
template <typename A>
concept BatchAssemblerLike =
    AssemblerLike<A> && std::is_arithmetic_v<typename A::ScalarType> &&
    requires(const std::byte* read_ptr, std::byte* write_ptr, size_t stride) {
      {
        A::template Write<
            xsimd::batch<typename A::ScalarType>,
            assembler_detail::LayoutProbe>(
            write_ptr,
            stride,
            A::template Read<
                xsimd::batch<typename A::ScalarType>,
                assembler_detail::LayoutProbe>(read_ptr, stride)
        )
      } -> std::same_as<void>;
    } && AssemblableLike<A>;

}  // namespace achilles::engine
