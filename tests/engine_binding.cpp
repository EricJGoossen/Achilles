#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>
#include <type_traits>

#include "domain/math/vector3.hpp"
#include "engine/binding.hpp"
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

using achilles::domain::math::Vector3Assembler;
using achilles::engine::FieldBufferingT;
using achilles::engine::FieldHasName;
using achilles::engine::FieldIndexingT;
using achilles::engine::FieldOrderingT;
using achilles::policy::buffering::BufferSide;
using achilles::policy::buffering::DoubleBuffering;
using achilles::policy::buffering::SingleBuffering;
using achilles::policy::indexing::GlobalIndexing;
using achilles::policy::indexing::RowIndexing;
using achilles::policy::layout::PlanarLayout;
using achilles::policy::ordering::LinearOrdering;
using achilles::policy::ordering::TopologicalOrdering;

namespace {

struct BindingTestGroup;

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
  using Buffering = DoubleBuffering<BindingTestGroup, BufferSide::kRead>;
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
               SingleBuffering>));
}

TEST(ResolvedFieldBuffering, UsesExplicitlyDeclaredBuffering) {
  EXPECT_TRUE((std::is_same_v<
               FieldBufferingT<
                   BindingTestField,
                   BindingTestTraits,
                   BindingTestField::kGlobalBuffered>,
               DoubleBuffering<BindingTestGroup, BufferSide::kRead>>));
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
