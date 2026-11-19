// Reads each field's traits: sharing, ordering, indexing, layout, buffering.
// Doc: docs/code/engine_binding.md

#pragma once

#include <concepts>
#include <string_view>

#include "policy/buffering/base_buffering.hpp"
#include "policy/buffering/double_buffering.hpp"
#include "policy/buffering/single_buffering.hpp"
#include "policy/indexing/base_indexing.hpp"
#include "policy/indexing/global_indexing.hpp"
#include "policy/indexing/group_indexing.hpp"
#include "policy/indexing/row_indexing.hpp"
#include "policy/layout/base_layout.hpp"
#include "policy/layout/planar_layout.hpp"
#include "policy/layout/segmented_layout.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "policy/ordering/linear_ordering.hpp"
#include "policy/ordering/list_ordering.hpp"
#include "policy/ordering/pair_ordering.hpp"
#include "policy/ordering/topological_ordering.hpp"
#include "util/tmp.hpp"

namespace achilles::engine {

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
//     using Ordering  = policy::ordering::TopologicalOrdering;               //
//     opt using Indexing  = policy::indexing::RowIndexing; // opt using
//     Buffering = policy::buffering::DoubleBuffering<Group, Side>;         //
//     opt using Layout    = policy::layout::PlanarLayout;                //
//     mandatory
//   };
//
// No SharedAs -> private to this field. No Ordering -> topology::
// LinearOrdering (every instance stands alone -- see orderings.hpp).
// No Indexing -> policy::indexing::RowIndexing (see indexing_policy.hpp).
// No Buffering -> SingleBuffering (see buffering_policy.hpp).
// Layout has no default -- omitting it is a compile error. None of these
// require touching engine/.
//
// Ordering is a policy TYPE, not a value (see
// policy::ordering::OrderingPolicyLike): the kind itself --
// policy::ordering::LinearOrdering, policy::ordering::TopologicalOrdering
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

namespace detail {

// Each optional policy a Traits<F> may declare, as the util::DetectedOrT
// probe for it (util/tmp.hpp).
template <typename T>
using DeclaredOrdering = typename T::Ordering;
template <typename T>
using DeclaredIndexing = typename T::Indexing;
template <typename T>
using DeclaredBuffering = typename T::Buffering;

// Identity aliases whose constraint rejects a declared policy that doesn't
// satisfy its concept, naming that concept in the error.
template <policy::ordering::OrderingPolicyLike P>
using CheckedOrdering = P;
template <policy::indexing::IndexingPolicyLike P>
using CheckedIndexing = P;
template <policy::buffering::BufferingPolicyLike P>
using CheckedBuffering = P;

}  // namespace detail

template <typename EnumT, template <EnumT> class Traits, EnumT F>
using FieldOrderingT = detail::CheckedOrdering<util::DetectedOrT<
    policy::ordering::LinearOrdering,
    detail::DeclaredOrdering,
    Traits<F>>>;

// Optional, like Ordering: no Indexing -> policy::indexing::RowIndexing (one
// value per row, every field's shape before index spaces existed). A field that
// holds one value per lane group (e.g. a broad phase's overlap bits) names
// policy::indexing::GroupIndexing instead -- see indexing_policy.hpp.
template <typename EnumT, template <EnumT> class Traits, EnumT F>
using FieldIndexingT = detail::CheckedIndexing<util::DetectedOrT<
    policy::indexing::RowIndexing,
    detail::DeclaredIndexing,
    Traits<F>>>;

// Optional, like Indexing: no Buffering -> SingleBuffering (one block, read in
// place). A field one side writes while another reads the previous value
// names DoubleBuffering<Group, Side> instead -- see buffering_policy.hpp.
template <typename EnumT, template <EnumT> class Traits, EnumT F>
using FieldBufferingT = detail::CheckedBuffering<util::DetectedOrT<
    policy::buffering::SingleBuffering,
    detail::DeclaredBuffering,
    Traits<F>>>;

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
      "explicit layout policy (e.g. policy::layout::PlanarLayout)."
  );
};

template <typename EnumT, template <EnumT> class Traits, EnumT F>
  requires FieldHasLayout<EnumT, Traits, F>
struct ResolvedFieldLayout<EnumT, Traits, F> {
  using Type = typename Traits<F>::Layout;
  static_assert(
      policy::layout::LayoutPolicyLike<Type>,
      "Traits<F>::Layout must satisfy LayoutPolicyLike."
  );
};

template <typename EnumT, template <EnumT> class Traits, EnumT F>
using FieldLayoutT = typename ResolvedFieldLayout<EnumT, Traits, F>::Type;

// The name a field is addressed by in user-authored scene data (see
// domain/archetype/archetype.hpp's domain::JointField and SimAllocator's
// Populate pass) -- optional, like SharedAs/Ordering: an algorithm-internal
// scratch or output field (e.g. ABA's kJointAcceleration) is never
// user-supplied and simply declares no kName, so SimAllocator's Populate
// pass skips it entirely rather than looking for scene data that will
// never exist.
template <typename EnumT, template <EnumT> class Traits, EnumT F>
concept FieldHasName = requires {
  { Traits<F>::kName } -> std::convertible_to<std::string_view>;
};

}  // namespace achilles::engine
