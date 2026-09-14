#include <gtest/gtest.h>

#include <cstddef>

#include "algorithms/aba/aba_data.hpp"
#include "domain/math/vector3.hpp"
#include "engine/field_contract.hpp"
#include "support/toy_field.hpp"

using achilles::engine::FieldEnumLike;
using achilles::engine::FieldTraitsLike;
using achilles::engine::HasFieldAssembler;
using achilles::test_support::ToyField;
using achilles::test_support::ToyFieldTraits;  // NOLINT

// Every check in this file is compile-time-only (a concept or a consteval
// function) -- there's no runtime behavior to exercise, so each TEST body
// is empty and the real assertions are the static_asserts above it. Kept
// as TESTs anyway (rather than bare file-scope static_asserts) so a
// failure names which specific check broke in the test-runner output, and
// so this file has runnable tests for gtest_discover_tests to find, per
// TESTING.md.

namespace {

enum class MissingKCount { kFoo, kBar };
static_assert(
    !FieldEnumLike<MissingKCount>,
    "An enum with no kCount enumerator at all must be rejected."
);

static_assert(FieldEnumLike<ToyField>);
static_assert(FieldEnumLike<achilles::algorithms::aba::ABAField>);

}  // namespace

TEST(FieldEnumLikeConcept, AcceptsAndRejects) { SUCCEED(); }

namespace {

// Traits missing an Assembler for one field (here, ToyField::kVelocity)
// fails HasFieldAssembler for exactly that field -- and therefore fails
// FieldTraitsLike overall, since AllFieldsHaveAssemblers requires every
// field in [0, kCount) to have one.
template <ToyField F>
struct IncompleteTraits;
template <>
struct IncompleteTraits<ToyField::kPosition> {
  using Assembler = achilles::domain::math::Vector3Assembler<float>;
};
// No specialization for ToyField::kVelocity.

static_assert(HasFieldAssembler<ToyField, ToyFieldTraits, ToyField::kPosition>);
static_assert(HasFieldAssembler<ToyField, ToyFieldTraits, ToyField::kVelocity>);
static_assert(HasFieldAssembler<
              ToyField,
              IncompleteTraits,
              ToyField::kPosition>);
static_assert(
    !HasFieldAssembler<ToyField, IncompleteTraits, ToyField::kVelocity>,
    "A field with no Traits<F> specialization at all must fail "
    "HasFieldAssembler, not fail to compile."
);

}  // namespace

TEST(HasFieldAssemblerConcept, DetectsAMissingFieldSpecifically) { SUCCEED(); }

namespace {

// Traits where every field has *a* Traits<F>::Assembler member, but one
// of them isn't itself AssemblerLike (it's missing ScalarType/kNumFields/
// Read/Write entirely) -- HasFieldAssembler checks AssemblerLike<Assembler>
// too, not just that the typedef exists.
struct NotAnAssembler {};

template <ToyField F>
struct WrongAssemblerTypeTraits;
template <>
struct WrongAssemblerTypeTraits<ToyField::kPosition> {
  using Assembler = achilles::domain::math::Vector3Assembler<float>;
};
template <>
struct WrongAssemblerTypeTraits<ToyField::kVelocity> {
  using Assembler = NotAnAssembler;
};

static_assert(!HasFieldAssembler<
              ToyField,
              WrongAssemblerTypeTraits,
              ToyField::kVelocity>);
static_assert(!FieldTraitsLike<ToyField, WrongAssemblerTypeTraits>);
static_assert(!FieldTraitsLike<ToyField, IncompleteTraits>);

static_assert(FieldTraitsLike<ToyField, ToyFieldTraits>);
static_assert(FieldTraitsLike<
              achilles::algorithms::aba::ABAField,
              achilles::algorithms::aba::ABAFieldTraits>);

}  // namespace

TEST(FieldTraitsLikeConcept, RequiresEveryFieldToHaveARealAssembler) {
  SUCCEED();
}
