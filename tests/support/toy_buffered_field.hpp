#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

#include "domain/archetype/archetype.hpp"
#include "domain/archetype/archetype_set.hpp"
#include "domain/math/vector3.hpp"
#include "engine/algorithm.hpp"
#include "engine/allocation/sim_allocator.hpp"
#include "engine/spec/field_contract.hpp"
#include "engine/spec/step_contract.hpp"
#include "engine/spec/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "policy/buffering/base_buffering.hpp"
#include "policy/buffering/double_buffering.hpp"
#include "policy/buffering/single_buffering.hpp"
#include "policy/layout/base_layout.hpp"
#include "policy/layout/planar_layout.hpp"
#include "policy/layout/segmented_layout.hpp"

namespace achilles::test_support {

// A FieldEnum with one double-buffered field -- the smallest real Algorithm
// that exercises policy::buffering::DoubleBuffering through SimAllocator, View
// and SimContext, without ABA's fields in the way. kCommand (the read side) is
// populated from archetype data (its rest value), so a test can tell it
// apart from a zero-filled half; kState is an ordinary single-buffered
// field, the control that must not be affected by a swap.
enum class ToyBufferedField : std::uint8_t {
  kCommand,
  kState,
  kCount,
};

// The buffer group (and shared slot) kCommand belongs to.
struct ToyCommandSlot;

template <ToyBufferedField F>
struct ToyBufferedFieldTraits;

template <>
struct ToyBufferedFieldTraits<ToyBufferedField::kCommand> {
  using Assembler = domain::math::Vector3Assembler<float>;
  using SharedAs = ToyCommandSlot;
  using Buffering = engine::memory::
      DoubleBuffering<ToyCommandSlot, policy::buffering::BufferSide::kRead>;
  using Layout = policy::layout::PlanarLayout;
  static constexpr std::string_view kName = "command";
};
template <>
struct ToyBufferedFieldTraits<ToyBufferedField::kState> {
  using Assembler = domain::math::Vector3Assembler<float>;
  using Layout = policy::layout::PlanarLayout;
};
static_assert(engine::
                  FieldTraitsLike<ToyBufferedField, ToyBufferedFieldTraits>);

using ToyBufferedView =
    engine::view::ViewFactory<ToyBufferedField, ToyBufferedFieldTraits>;
static_assert(engine::spec::ViewLike<ToyBufferedView, ToyBufferedField>);

using ToyBufferedAlgorithm =
    engine::Algorithm<ToyBufferedField, ToyBufferedFieldTraits>;

// The rest value MakeToyBufferedSim gives instance i's kCommand: every leaf
// distinct, and none zero, so it can't be mistaken for a zero-filled block.
inline domain::math::Vector3<float> ToyCommandRest(std::size_t instance) {
  auto base = static_cast<float>(10 * (instance + 1));
  return {base + 1.0F, base + 2.0F, base + 3.0F};
}

// `instance_count` standalone single-joint instances (default
// LinearOrdering, so instance i sits at row i -- see MakeToyArchetype in
// toy_field.hpp), each with kCommand populated to ToyCommandRest(i).
inline engine::allocation::SimAllocator<ToyBufferedAlgorithm>
MakeToyBufferedSim(std::size_t instance_count) {
  std::vector<double> command;
  for (std::size_t i = 0; i < instance_count; ++i) {
    domain::math::Vector3<float> rest = ToyCommandRest(i);
    command.insert(command.end(), {rest.X(), rest.Y(), rest.Z()});
  }
  std::vector<domain::layout::JointField> fields;
  fields.push_back(domain::layout::JointField{"command", 3, std::move(command)}
  );
  std::array<domain::archetype::Archetype, 1> archetypes = {
      domain::archetype::Archetype(
          "toy",
          {domain::layout::kNoParent},
          std::vector<domain::layout::JointRef>(instance_count),
          true,
          std::move(fields)
      )
  };
  return engine::allocation::SimAllocator<ToyBufferedAlgorithm>(
      achilles::domain::archetype::ArchetypeSet(archetypes)
  );
}

// Swaps kCommand's halves the way an Algorithm's own Step would.
inline void SwapToyBuffers(
    const engine::allocation::SimAllocator<ToyBufferedAlgorithm>& sim
) {
  sim.SimContext().FlipParity<ToyCommandSlot>();
}

}  // namespace achilles::test_support
