#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>
#include <type_traits>

#include "domain/math/vector3.hpp"
#include "engine/memory/binding.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"

using achilles::domain::math::Vector3Assembler;
using achilles::engine::memory::FieldHasName;
using achilles::engine::memory::FieldOrderingT;
using achilles::engine::topology::LinearOrdering;
using achilles::engine::topology::PlanarLayout;
using achilles::engine::topology::TopologicalOrdering;

namespace {

// A minimal three-field enum exercising every combination binding.hpp's
// optional traits (Ordering, kName) support: no Ordering declared (must
// default to LinearOrdering), an explicit TopologicalOrdering, and a field
// with a kName vs. one without.
enum class BindingTestField : std::uint8_t { kDefault, kTopological, kCount };

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
