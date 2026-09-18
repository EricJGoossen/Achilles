#include <gtest/gtest.h>

#include <type_traits>

#include "engine/algorithm_contract.hpp"
#include "support/toy_field.hpp"

using achilles::engine::Algorithm;
using achilles::engine::AlgorithmLike;
using achilles::engine::NoStep;
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
// pair, so this only ever proves AlgorithmLike's own three-member shape,
// never anything about a real FieldEnumLike/ViewLike's own contract (those
// have their own concept archetypes -- see engine_field_contract.cpp and
// engine_view_contract.cpp).
struct AlgorithmArchetype {
  using Enum = ToyField;
  using View = ToyView;
  using Step = NoStep;
};
static_assert(AlgorithmLike<AlgorithmArchetype>);

struct NotAnAlgorithm {};
static_assert(
    !AlgorithmLike<NotAnAlgorithm>,
    "A type with none of Enum/View/Step must be rejected."
);

// Has View/Step but no Enum at all.
struct MissingEnum {
  using View = ToyView;
  using Step = NoStep;
};
static_assert(!AlgorithmLike<MissingEnum>);

// Has Enum/Step but no View at all.
struct MissingView {
  using Enum = ToyField;
  using Step = NoStep;
};
static_assert(!AlgorithmLike<MissingView>);

// Has Enum/View but no Step at all -- Step has no shape requirement of its
// own (see AlgorithmLike), but the member must still exist.
struct MissingStep {
  using Enum = ToyField;
  using View = ToyView;
};
static_assert(!AlgorithmLike<MissingStep>);

// Enum exists but isn't FieldEnumLike (no kCount) -- proves AlgorithmLike
// actually checks FieldEnumLike<Enum>, not just that the member exists.
struct NotAFieldEnum {};
struct WrongEnumType {
  using Enum = NotAFieldEnum;
  using View = ToyView;
  using Step = NoStep;
};
static_assert(!AlgorithmLike<WrongEnumType>);

// View exists but doesn't satisfy ViewLike<Enum> -- proves AlgorithmLike
// actually checks view::ViewLike<View, Enum>, not just that the member
// exists.
struct NotAView {};
struct WrongViewType {
  using Enum = ToyField;
  using View = NotAView;
  using Step = NoStep;
};
static_assert(!AlgorithmLike<WrongViewType>);

}  // namespace

TEST(AlgorithmLikeConcept, AcceptsAndRejects) { SUCCEED(); }

namespace {

// The real Algorithm<EnumT, TraitsT, StepT> template itself must satisfy
// the same concept its own archetype above does.
static_assert(AlgorithmLike<Algorithm<ToyField, ToyFieldTraits>>);

struct SomeStep {};
static_assert(AlgorithmLike<Algorithm<ToyField, ToyFieldTraits, SomeStep>>);

// StepT defaults to NoStep (engine::pass::Step, sim_step.hpp, skips any
// Algorithm whose Step names this type) when a caller doesn't name one --
// the default is part of Algorithm's own contract, not just an
// implementation detail this test happens to observe.
static_assert(
    std::is_same_v<Algorithm<ToyField, ToyFieldTraits>::Step, NoStep>
);
static_assert(std::is_same_v<
              Algorithm<ToyField, ToyFieldTraits, SomeStep>::Step,
              SomeStep>);

}  // namespace

TEST(AlgorithmTemplate, SatisfiesAlgorithmLikeAndDefaultsStepToNoStep) {
  SUCCEED();
}
