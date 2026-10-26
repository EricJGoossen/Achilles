#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>
#include <type_traits>

#include "domain/math/vector3.hpp"
#include "engine/memory/binding.hpp"
#include "engine/memory/buffering_policy.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"

using achilles::domain::math::Vector3Assembler;
using achilles::engine::memory::DoubleBuffered;
using achilles::engine::memory::FieldBufferingT;
using achilles::engine::memory::FieldHasName;
using achilles::engine::memory::FieldIndexingT;
using achilles::engine::memory::FieldOrderingT;
using achilles::engine::memory::SingleBuffered;
using achilles::engine::topology::GlobalIndexing;
using achilles::engine::topology::LinearOrdering;
using achilles::engine::topology::PlanarLayout;
using achilles::engine::topology::RowIndexing;
using achilles::engine::topology::TopologicalOrdering;

namespace {

// A minimal enum exercising every combination binding.hpp's optional traits
// (Ordering, Indexing, Buffering, kName) support: kDefault declares none of
// the policies (each must fall back to its default), kTopological and
// kGlobalBuffered declare them explicitly, and kDefault is the one field
// with a kName.
enum class BindingTestField : std::uint8_t {
  kDefault,
  kTopological,
  kGlobalBuffered,
  kCount
};

template <BindingTestField F>
struct BindingTestTraits;

template <>
struct BindingTestTraits<BindingTestField::kDefault> {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
  static constexpr std::string_view kName = "default_field";
};

template <>
struct BindingTestTraits<BindingTestField::kTopological> {
  using Assembler = Vector3Assembler<float>;
  using Ordering = TopologicalOrdering;
  using Layout = PlanarLayout;
};

template <>
struct BindingTestTraits<BindingTestField::kGlobalBuffered> {
  using Assembler = Vector3Assembler<float>;
  using Indexing = GlobalIndexing;
  using Buffering = DoubleBuffered;
  using Layout = PlanarLayout;
};

// Declares a Buffering that isn't a buffering policy at all. In its own
// enum so nothing else here ever resolves it.
enum class BadBufferingField : std::uint8_t { kBad, kCount };

template <BadBufferingField F>
struct BadBufferingTraits;

template <>
struct BadBufferingTraits<BadBufferingField::kBad> {
  using Assembler = Vector3Assembler<float>;
  using Buffering = int;
  using Layout = PlanarLayout;
};

// Whether FieldBufferingT resolves at all -- a concept so a rejected policy
// reads as `false` instead of a hard error.
template <typename EnumT, template <EnumT> class Traits, EnumT F>
concept BufferingResolves =
    requires { typename FieldBufferingT<EnumT, Traits, F>; };

}  // namespace

TEST(ResolvedFieldOrdering, DefaultsToLinearOrderingWhenUndeclared) {
  EXPECT_TRUE((std::is_same_v<
               FieldOrderingT<
                   BindingTestField,
                   BindingTestTraits,
                   BindingTestField::kDefault>,
               LinearOrdering>));
}

TEST(ResolvedFieldOrdering, UsesExplicitlyDeclaredOrdering) {
  EXPECT_TRUE((std::is_same_v<
               FieldOrderingT<
                   BindingTestField,
                   BindingTestTraits,
                   BindingTestField::kTopological>,
               TopologicalOrdering>));
}

TEST(ResolvedFieldIndexing, DefaultsToRowIndexingWhenUndeclared) {
  EXPECT_TRUE((std::is_same_v<
               FieldIndexingT<
                   BindingTestField,
                   BindingTestTraits,
                   BindingTestField::kDefault>,
               RowIndexing>));
}

TEST(ResolvedFieldIndexing, UsesExplicitlyDeclaredIndexing) {
  EXPECT_TRUE((std::is_same_v<
               FieldIndexingT<
                   BindingTestField,
                   BindingTestTraits,
                   BindingTestField::kGlobalBuffered>,
               GlobalIndexing>));
}

TEST(ResolvedFieldBuffering, DefaultsToSingleBufferedWhenUndeclared) {
  EXPECT_TRUE((std::is_same_v<
               FieldBufferingT<
                   BindingTestField,
                   BindingTestTraits,
                   BindingTestField::kDefault>,
               SingleBuffered>));
}

TEST(ResolvedFieldBuffering, UsesExplicitlyDeclaredBuffering) {
  EXPECT_TRUE((std::is_same_v<
               FieldBufferingT<
                   BindingTestField,
                   BindingTestTraits,
                   BindingTestField::kGlobalBuffered>,
               DoubleBuffered>));
}

// A declared Buffering that isn't a BufferingPolicyLike must be rejected,
// not silently treated as undeclared (which would quietly single-buffer a
// field its author meant to double-buffer).
TEST(ResolvedFieldBuffering, RejectsADeclaredTypeThatIsNotABufferingPolicy) {
  EXPECT_FALSE((BufferingResolves<
                BadBufferingField,
                BadBufferingTraits,
                BadBufferingField::kBad>));
  EXPECT_TRUE((BufferingResolves<
               BindingTestField,
               BindingTestTraits,
               BindingTestField::kGlobalBuffered>));
}

TEST(FieldHasNameConcept, TrueOnlyForFieldsDeclaringKName) {
  EXPECT_TRUE((FieldHasName<
               BindingTestField,
               BindingTestTraits,
               BindingTestField::kDefault>));
  EXPECT_FALSE((FieldHasName<
                BindingTestField,
                BindingTestTraits,
                BindingTestField::kTopological>));
}
