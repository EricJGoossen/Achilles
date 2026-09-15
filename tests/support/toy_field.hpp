#pragma once

#include "domain/math/vector3.hpp"
#include "engine/field_contract.hpp"
#include "engine/view/planar_view.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::test_support {

// A minimal, two-field FieldEnum + Traits for exercising the engine's generic
// machinery (field_contract, view_contract, PlanarView, op_contract,
// op_invoker, algorithm_step, traversals) against something smaller and
// more legible than the real ABAField (see algorithms/aba/aba_data.hpp,
// which several of these test files also use directly, as the "does this
// hold for the real, already-shipping case too" check). Built from a real
// production Assembler (Vector3Assembler<float>), not an invented one.
enum class ToyField {
  kPosition,
  kVelocity,
  kCount,
};

template <ToyField F>
struct ToyFieldTraits;

template <>
struct ToyFieldTraits<ToyField::kPosition> {
  using Assembler = achilles::domain::math::Vector3Assembler<float>;
};
template <>
struct ToyFieldTraits<ToyField::kVelocity> {
  using Assembler = achilles::domain::math::Vector3Assembler<float>;
};

static_assert(achilles::engine::FieldTraitsLike<ToyField, ToyFieldTraits>);

using ToyView = achilles::engine::view::
    ViewFactory<achilles::engine::view::PlanarView, ToyField, ToyFieldTraits>;
static_assert(achilles::engine::view::ViewLike<ToyView, ToyField>);

}  // namespace achilles::test_support
