#pragma once

#include <concepts>
#include <string_view>

#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"

namespace achilles::engine::memory {

// Per-field sharing / ordering / indexing / layout intent, read out of the
// same Traits<F> the Assembler already lives in. SharedAs, Ordering and
// Indexing are detected optionally -- exactly the way op_contract detects an
// Op's optional Initialize (OpHasInit) -- but Layout is mandatory: every field
// must say explicitly how it is laid out. Adding an algorithm stays "new
// enum + new Traits + new Ops":
//
//   struct FixedJointTransformSlot;  // tag, lives in a shared header
//   template <> struct SomeFieldTraits<Field::kFixedJointTransform> {
//     using Assembler = spatial::TransformAssembler<ScalarOperationT>;
//     using SharedAs  = FixedJointTransformSlot;                     // opt
//     using Ordering  = topology::TopologicalOrdering;               // opt
//     using Indexing  = topology::RowIndexing;                       // opt
//     using Layout    = topology::PlanarLayout;                // mandatory
//   };
//
// No SharedAs -> private to this field. No Ordering -> topology::
// LinearOrdering (every instance stands alone -- see ordering_policy.hpp).
// No Indexing -> topology::RowIndexing (see indexing_policy.hpp).
// Layout has no default -- omitting it is a compile error. None of these
// require touching engine/.
//
// Ordering is a policy TYPE, not a value (see topology::OrderingPolicyLike):
// the kind itself -- topology::LinearOrdering, topology::TopologicalOrdering
// -- owns the algorithm that sequences and pads a field's instances, the
// same way Layout's policy type owns a field's byte layout. A new ordering
// kind is a new policy struct, not a new branch in a shared builder.

// Distinct link-time address per tag, no RTTI needed -- the shared-slot
// registry key SimAllocator dedups on.
template <typename Tag>
inline constexpr char kSlotStorage = 0;

template <typename Tag>
constexpr const void* SlotId() noexcept {
  return &kSlotStorage<Tag>;
}

template <typename EnumT, template <EnumT> class Traits, EnumT F>
concept FieldIsShared = requires { typename Traits<F>::SharedAs; };

template <typename EnumT, template <EnumT> class Traits, EnumT F>
concept FieldHasOrdering = requires { typename Traits<F>::Ordering; };

template <typename EnumT, template <EnumT> class Traits, EnumT F>
struct ResolvedFieldOrdering {
  using Type = topology::LinearOrdering;
};

template <typename EnumT, template <EnumT> class Traits, EnumT F>
  requires FieldHasOrdering<EnumT, Traits, F>
struct ResolvedFieldOrdering<EnumT, Traits, F> {
  using Type = typename Traits<F>::Ordering;
  static_assert(
      topology::OrderingPolicyLike<Type>,
      "Traits<F>::Ordering, if declared, must satisfy OrderingPolicyLike."
  );
};

template <typename EnumT, template <EnumT> class Traits, EnumT F>
using FieldOrderingT = typename ResolvedFieldOrdering<EnumT, Traits, F>::Type;

// Optional, like Ordering: no Indexing -> topology::RowIndexing (one value
// per row, every field's shape before index spaces existed). A field that
// holds one value per pair of rows (e.g. a broad phase's overlap result)
// names topology::PairGroupIndexing instead -- see indexing_policy.hpp.
template <typename EnumT, template <EnumT> class Traits, EnumT F>
concept FieldHasIndexing = requires { typename Traits<F>::Indexing; };

template <typename EnumT, template <EnumT> class Traits, EnumT F>
struct ResolvedFieldIndexing {
  using Type = topology::RowIndexing;
};

template <typename EnumT, template <EnumT> class Traits, EnumT F>
  requires FieldHasIndexing<EnumT, Traits, F>
struct ResolvedFieldIndexing<EnumT, Traits, F> {
  using Type = typename Traits<F>::Indexing;
  static_assert(
      topology::IndexingPolicyLike<Type>,
      "Traits<F>::Indexing, if declared, must satisfy IndexingPolicyLike."
  );
};

template <typename EnumT, template <EnumT> class Traits, EnumT F>
using FieldIndexingT = typename ResolvedFieldIndexing<EnumT, Traits, F>::Type;

template <typename EnumT, template <EnumT> class Traits, EnumT F>
concept FieldHasLayout = requires { typename Traits<F>::Layout; };

// No fallback: every field must name its own Layout explicitly. The primary
// template exists only to hold the static_assert that fires with a clear
// message when Traits<F>::Layout is missing, rather than the caller hitting
// a raw "no member named Type" error out of FieldLayoutT below.
template <typename EnumT, template <EnumT> class Traits, EnumT F>
struct ResolvedFieldLayout {
  static_assert(
      FieldHasLayout<EnumT, Traits, F>,
      "Traits<F>::Layout is mandatory -- every field must declare an "
      "explicit layout policy (e.g. topology::PlanarLayout)."
  );
};

template <typename EnumT, template <EnumT> class Traits, EnumT F>
  requires FieldHasLayout<EnumT, Traits, F>
struct ResolvedFieldLayout<EnumT, Traits, F> {
  using Type = typename Traits<F>::Layout;
  static_assert(
      topology::LayoutPolicyLike<Type>,
      "Traits<F>::Layout must satisfy LayoutPolicyLike."
  );
};

template <typename EnumT, template <EnumT> class Traits, EnumT F>
using FieldLayoutT = typename ResolvedFieldLayout<EnumT, Traits, F>::Type;

// The name a field is addressed by in user-authored archetype data (see
// domain/archetype.hpp's ArchetypeField and SimAllocator's
// Populate pass) -- optional, like SharedAs/Ordering: an algorithm-internal
// scratch or output field (e.g. ABA's kJointAcceleration) is never
// user-supplied and simply declares no kName, so SimAllocator's Populate
// pass skips it entirely rather than looking for archetype data that will
// never exist.
template <typename EnumT, template <EnumT> class Traits, EnumT F>
concept FieldHasName = requires {
  { Traits<F>::kName } -> std::convertible_to<std::string_view>;
};

}  // namespace achilles::engine::memory
