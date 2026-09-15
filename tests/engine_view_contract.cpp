#include <gtest/gtest.h>

#include "algorithms/aba/aba_data.hpp"
#include "engine/view/view_contract.hpp"
#include "support/toy_field.hpp"

using achilles::engine::view::ViewLike;
using achilles::test_support::ToyField;
using achilles::test_support::ToyView;

// Every check in this file is compile-time-only (concepts, no runtime
// component) -- see engine_field_contract.cpp for why these are still
// TESTs (empty bodies, real assertion is the static_assert above) rather
// than bare file-scope static_asserts.

namespace {

// ToyView (support/toy_field.hpp) is a real ViewFactory<PlanarView, ...>
// instantiation, already checked in that header; ABAView is the
// real, already-shipping production view. Both confirmed ViewLike here
// too so a regression in either shows up as a failure in this file
// specifically.
static_assert(ViewLike<ToyView, ToyField>);
static_assert(ViewLike<
              achilles::algorithms::aba::ABAView,
              achilles::algorithms::aba::ABAField>);

// A type with none of the required nested types/methods must fail
// ViewLike outright, not fail to compile -- concepts are meant to be
// checkable, not just satisfiable.
template <typename EnumT>
struct NotAView {};
static_assert(!ViewLike<NotAView<ToyField>, ToyField>);

}  // namespace

TEST(ViewLikeConcept, AcceptsRealViewsRejectsNonViews) { SUCCEED(); }
