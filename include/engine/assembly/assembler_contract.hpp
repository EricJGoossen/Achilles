// What an assembler, and the domain types it assembles, must provide.

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

namespace achilles::engine::assembly {

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

}  // namespace achilles::engine::assembly
