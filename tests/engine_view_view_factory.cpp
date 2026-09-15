#include <type_traits>

#include <gtest/gtest.h>

#include "algorithms/aba/aba_data.hpp"
#include "domain/math/vector3.hpp"
#include "engine/view/planar_view.hpp"
#include "engine/view/view_factory.hpp"
#include "support/toy_field.hpp"

using achilles::domain::math::Vector3Assembler;
using achilles::engine::view::PlanarView;
using achilles::engine::view::ViewFactory;
using achilles::test_support::ToyField;
using achilles::test_support::ToyFieldTraits;
using achilles::test_support::ToyView;

// ViewFactory is pure compile-time type computation (MakeViewFromTraits
// is declared, never defined -- it only exists to be named inside
// decltype()) -- there's no runtime behavior, so this is entirely
// static_asserts with an empty TEST body per file, same as
// engine_field_contract.cpp/engine_view_contract.cpp.

namespace {

// ViewFactory<PlanarView, ToyField, ToyFieldTraits> must be exactly
// PlanarView<ToyField, Vector3Assembler<float>, Vector3Assembler<float>>
// -- the concrete type support/toy_field.hpp's own ToyView alias already
// names, checked here as an independent assertion rather than trusting
// that alias's definition.
static_assert(std::is_same_v<
              ViewFactory<PlanarView, ToyField, ToyFieldTraits>,
              PlanarView<ToyField, Vector3Assembler<float>, Vector3Assembler<float>>>);
static_assert(std::is_same_v<ViewFactory<PlanarView, ToyField, ToyFieldTraits>, ToyView>);

// Same check against the real, already-shipping ABA case: ViewFactory's
// output must be exactly ABAView (algorithms/aba/aba_data.hpp).
static_assert(std::is_same_v<
              ViewFactory<
                  PlanarView,
                  achilles::algorithms::aba::ABAField,
                  achilles::algorithms::aba::ABAFieldTraits>,
              achilles::algorithms::aba::ABAView>);

}  // namespace

TEST(ViewFactory, ProducesTheExactExpectedViewType) { SUCCEED(); }
