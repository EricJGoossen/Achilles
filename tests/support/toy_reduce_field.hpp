#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "domain/archetype.hpp"
#include "domain/math/scalar.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/field_contract.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "toy_field.hpp"

namespace achilles::test_support {

// ToyField's counterpart for the engine's reductions (see
// topology::BroadcastIndex): one plain per-row scalar, and one scalar the
// whole sim shares (topology::GlobalIndexing) -- the smallest pair of
// fields a pass can reduce from one into the other, and a GlobalTraversal
// Op can then read back. Real production Assemblers (ScalarAssembler<float>),
// like ToyField's.
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
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct ToyReduceFieldTraits<ToyReduceField::kTotal> {
  using Assembler = domain::math::ScalarAssembler<float>;
  using Layout = engine::topology::PlanarLayout;
  using Indexing = engine::topology::GlobalIndexing;
};

static_assert(engine::FieldTraitsLike<ToyReduceField, ToyReduceFieldTraits>);

using ToyReduceView =
    engine::view::ViewFactory<ToyReduceField, ToyReduceFieldTraits>;
static_assert(engine::view::ViewLike<ToyReduceView, ToyReduceField>);

using ToyReduceAlgorithm =
    engine::Algorithm<ToyReduceField, ToyReduceFieldTraits>;

// `instance_count` standalone rows, exactly as MakeToySim lays them out
// (see MakeToyArchetype): row i is instance i.
inline engine::memory::SimAllocator<ToyReduceAlgorithm> MakeToyReduceSim(
    std::size_t instance_count
) {
  std::array<domain::Archetype, 1> archetypes = {
      MakeToyArchetype(instance_count)
  };
  return engine::memory::SimAllocator<ToyReduceAlgorithm>(archetypes);
}

}  // namespace achilles::test_support
