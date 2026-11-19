#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "domain/archetype/archetype.hpp"
#include "domain/archetype/archetype_set.hpp"
#include "domain/math/scalar.hpp"
#include "engine/algorithm.hpp"
#include "engine/allocation/sim_allocator.hpp"
#include "engine/spec/field_contract.hpp"
#include "engine/spec/step_contract.hpp"
#include "engine/spec/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "policy/indexing/base_indexing.hpp"
#include "policy/indexing/global_indexing.hpp"
#include "policy/indexing/group_indexing.hpp"
#include "policy/indexing/row_indexing.hpp"
#include "policy/layout/base_layout.hpp"
#include "policy/layout/planar_layout.hpp"
#include "policy/layout/segmented_layout.hpp"
#include "toy_field.hpp"

namespace achilles::test_support {

// ToyField's counterpart for the engine's reductions (see
// policy::indexing::BroadcastIndex): one plain per-row scalar, and one scalar
// the whole sim shares (policy::indexing::GlobalIndexing) -- the smallest pair
// of fields a pass can reduce from one into the other, and a GlobalTraversal Op
// can then read back. Real production Assemblers (ScalarAssembler<float>), like
// ToyField's.
enum class ToyReduceField : std::uint8_t {
  kValue,
  kTotal,
  kCount,
};

template <ToyReduceField F>
struct ToyReduceFieldTraits;

template <>
struct ToyReduceFieldTraits<ToyReduceField::kValue> {
  using Assembler = domain::math::ScalarAssembler<float>;
  using Layout = policy::layout::PlanarLayout;
};
template <>
struct ToyReduceFieldTraits<ToyReduceField::kTotal> {
  using Assembler = domain::math::ScalarAssembler<float>;
  using Layout = policy::layout::PlanarLayout;
  using Indexing = policy::indexing::GlobalIndexing;
};

static_assert(engine::spec::
                  FieldTraitsLike<ToyReduceField, ToyReduceFieldTraits>);

using ToyReduceView =
    engine::view::ViewFactory<ToyReduceField, ToyReduceFieldTraits>;
static_assert(engine::spec::ViewLike<ToyReduceView, ToyReduceField>);

using ToyReduceAlgorithm =
    engine::Algorithm<ToyReduceField, ToyReduceFieldTraits>;

// `instance_count` standalone rows, exactly as MakeToySim lays them out
// (see MakeToyArchetype): row i is instance i.
inline engine::allocation::SimAllocator<ToyReduceAlgorithm> MakeToyReduceSim(
    std::size_t instance_count
) {
  std::array<domain::archetype::Archetype, 1> archetypes = {
      MakeToyArchetype(instance_count)
  };
  return engine::allocation::SimAllocator<ToyReduceAlgorithm>(
      achilles::domain::archetype::ArchetypeSet(archetypes)
  );
}

}  // namespace achilles::test_support
