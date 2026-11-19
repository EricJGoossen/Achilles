#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "domain/archetype/archetype.hpp"
#include "domain/archetype/archetype_set.hpp"
#include "domain/math/vector3.hpp"
#include "engine/algorithm.hpp"
#include "engine/allocation/sim_allocator.hpp"
#include "engine/spec/field_binding.hpp"
#include "engine/spec/field_contract.hpp"
#include "engine/spec/step_contract.hpp"
#include "engine/spec/view_contract.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_factory.hpp"
#include "policy/layout/base_layout.hpp"
#include "policy/layout/planar_layout.hpp"
#include "policy/layout/segmented_layout.hpp"

namespace achilles::test_support {

// A minimal, two-field FieldEnum + Traits for exercising the engine's generic
// machinery (field_contract, view_contract, View, op_contract,
// op_invoker, algorithm_step, traversals) against something smaller and
// more legible than the real ABAField (see algorithms/aba/aba_data.hpp,
// which several of these test files also use directly, as the "does this
// hold for the real, already-shipping case too" check). Built from a real
// production Assembler (Vector3Assembler<float>), not an invented one.
enum class ToyField : std::uint8_t {
  kPosition,
  kVelocity,
  kCount,
};

template <ToyField F>
struct ToyFieldTraits;

template <>
struct ToyFieldTraits<ToyField::kPosition> {
  using Assembler = achilles::domain::math::Vector3Assembler<float>;
  using Layout = achilles::policy::layout::PlanarLayout;
};
template <>
struct ToyFieldTraits<ToyField::kVelocity> {
  using Assembler = achilles::domain::math::Vector3Assembler<float>;
  using Layout = achilles::policy::layout::PlanarLayout;
};

static_assert(achilles::engine::spec::
                  FieldTraitsLike<ToyField, ToyFieldTraits>);

using ToyView = achilles::engine::view::ViewFactory<ToyField, ToyFieldTraits>;
static_assert(achilles::engine::spec::ViewLike<ToyView, ToyField>);

// Never stepped -- exists purely so tests can drive a real
// engine::allocation::SimAllocator to get a real, memory-backed ToyView,
// instead of hand-rolling one (see the now-deleted
// tests/support/view_fixture.hpp).
using ToyAlgorithm = achilles::engine::Algorithm<ToyField, ToyFieldTraits>;

// `instance_count` standalone single-joint instances of one root archetype
// -- no field data (neither ToyField declares kName, so SimAllocator's
// Populate pass has nothing to copy in), no parent/child relationship
// between them. Under the default policy::ordering::LinearOrdering (neither
// ToyFieldTraits specialization names an Ordering), SimAllocator places
// instance i's own joint at sorted row i exactly -- see
// policy::ordering::BaseOrdering::Consolidate: a single-joint archetype's block
// has exactly one local position, so row = 0 * padded_instance_count + i = i --
// so a caller that wants `instance_count` distinct, individually
// addressable real rows (row 0, 1, ..., instance_count - 1) via
// View::Load/Store<Field, float> gets exactly that, the same guarantee
// tests/support/view_fixture.hpp used to provide by construction instead of
// by this file's own derivation from the real ordering policy.
inline domain::archetype::Archetype MakeToyArchetype(std::size_t instance_count
) {
  std::vector<std::size_t> tree_structure = {domain::layout::kNoParent};
  std::vector<domain::layout::JointRef> root_parents(
      instance_count,
      domain::layout::JointRef{
          std::numeric_limits<std::size_t>::max(),
          std::numeric_limits<std::size_t>::max()
      }
  );
  return {"toy", std::move(tree_structure), std::move(root_parents), true, {}};
}

// A real, memory-backed ToyAlgorithm sim over `instance_count` standalone
// joints (see MakeToyArchetype) -- the caller keeps the returned
// SimAllocator alive for as long as any View drawn from it (via
// .State().View<ToyAlgorithm>()) is in use, the same lifetime rule
// SimAllocator itself documents. SimAllocator only reads `archetypes` for
// the duration of its own constructor call, so the local array here doesn't
// need to outlive this function.
inline engine::allocation::SimAllocator<ToyAlgorithm> MakeToySim(
    std::size_t instance_count
) {
  std::array<domain::archetype::Archetype, 1> archetypes = {
      MakeToyArchetype(instance_count)
  };
  return engine::allocation::SimAllocator<ToyAlgorithm>(
      achilles::domain::archetype::ArchetypeSet(archetypes)
  );
}

}  // namespace achilles::test_support
