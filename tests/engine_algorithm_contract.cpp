#include <gtest/gtest.h>

#include "engine/algorithm_contract.hpp"
#include "support/toy_field.hpp"

using achilles::engine::Algorithm;
using achilles::engine::AlgorithmLike;
using achilles::engine::StepLike;
using achilles::test_support::ToyField;
using achilles::test_support::ToyFieldTraits;
using achilles::test_support::ToyView;

// Every check in this file is compile-time-only (a concept, or the identity
// of a type alias) -- there's no runtime behavior to exercise, so each TEST
// body is empty and the real assertions are the static_asserts above it.
// Kept as TESTs anyway (rather than bare file-scope static_asserts) so a
// failure names which specific check broke in the test-runner output, and
// so this file has runnable tests for gtest_discover_tests to find, per
// TESTING.md.

namespace {

// Exactly what AlgorithmLike requires and nothing more -- built from real
// ToyField/ToyView (see support/toy_field.hpp), not an invented Enum/View
// pair, so this only ever proves AlgorithmLike's own two-member shape,
// never anything about a real FieldEnumLike/ViewLike's own contract (those
// have their own concept archetypes -- see engine_field_contract.cpp and
// engine_view_view_contract.cpp).
struct AlgorithmArchetype {
  using Enum = ToyField;
  using View = ToyView;
};
static_assert(AlgorithmLike<AlgorithmArchetype>);

struct NotAnAlgorithm {};
static_assert(
    !AlgorithmLike<NotAnAlgorithm>,
    "A type with neither Enum nor View must be rejected."
);

// Has View but no Enum at all.
struct MissingEnum {
  using View = ToyView;
};
static_assert(!AlgorithmLike<MissingEnum>);

// Has Enum but no View at all.
struct MissingView {
  using Enum = ToyField;
};
static_assert(!AlgorithmLike<MissingView>);

// Enum exists but isn't FieldEnumLike (no kCount) -- proves AlgorithmLike
// actually checks FieldEnumLike<Enum>, not just that the member exists.
struct NotAFieldEnum {};
struct WrongEnumType {
  using Enum = NotAFieldEnum;
  using View = ToyView;
};
static_assert(!AlgorithmLike<WrongEnumType>);

// View exists but doesn't satisfy ViewLike<Enum> -- proves AlgorithmLike
// actually checks view::ViewLike<View, Enum>, not just that the member
// exists.
struct NotAView {};
struct WrongViewType {
  using Enum = ToyField;
  using View = NotAView;
};
static_assert(!AlgorithmLike<WrongViewType>);

}  // namespace

TEST(AlgorithmLikeConcept, AcceptsAndRejects) { SUCCEED(); }

namespace {

// The real Algorithm<EnumT, TraitsT> template itself must satisfy the same
// concept its own archetype above does.
static_assert(AlgorithmLike<Algorithm<ToyField, ToyFieldTraits>>);

}  // namespace

TEST(AlgorithmTemplate, SatisfiesAlgorithmLike) { SUCCEED(); }

namespace {

// Exactly what StepLike requires and nothing more: a FieldEnum naming the
// Algorithm the Step runs over. Step(...) itself is a template on the
// SimContext type, so StepLike can't probe it (see StepLike's own
// comment) -- the archetype doesn't declare one, which is what proves the
// concept doesn't silently require it.
struct StepArchetype {
  using FieldEnum = ToyField;
};
static_assert(StepLike<StepArchetype>);

struct NotAStep {};
static_assert(!StepLike<NotAStep>, "A Step must name its FieldEnum.");

// FieldEnum exists but isn't FieldEnumLike -- proves StepLike checks
// FieldEnumLike<FieldEnum>, not just that the member exists.
struct WrongFieldEnumStep {
  using FieldEnum = int;
};
static_assert(!StepLike<WrongFieldEnumStep>);

}  // namespace

TEST(StepLikeConcept, AcceptsAndRejects) { SUCCEED(); }
