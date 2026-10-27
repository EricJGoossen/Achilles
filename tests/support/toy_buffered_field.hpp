#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "domain/math/vector3.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/field_contract.hpp"
#include "engine/memory/buffering_policy.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::test_support {

// A two-field FieldEnum with one double-buffered field -- the smallest real
// Algorithm that exercises memory::DoubleBuffered through SimAllocator,
// View and SimContext, without ABA's eighteen fields in the way. kCommand is
// populated from archetype data (its rest value), so a test can tell the
// rest block apart from a zero-filled half; kState is an ordinary
// single-buffered field, the control that must not be affected by a swap.
enum class ToyBufferedField : std::uint8_t {
  kCommand,
  kState,
  kCount,
};

template <ToyBufferedField F>
struct ToyBufferedFieldTraits;

template <>
struct ToyBufferedFieldTraits<ToyBufferedField::kCommand> {
  using Assembler = domain::math::Vector3Assembler<float>;
  using Buffering = engine::memory::DoubleBuffered;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "command";
};
template <>
struct ToyBufferedFieldTraits<ToyBufferedField::kState> {
  using Assembler = domain::math::Vector3Assembler<float>;
  using Layout = engine::topology::PlanarLayout;
};
static_assert(engine::
                  FieldTraitsLike<ToyBufferedField, ToyBufferedFieldTraits>);

using ToyBufferedView =
    engine::view::ViewFactory<ToyBufferedField, ToyBufferedFieldTraits>;
static_assert(engine::view::ViewLike<ToyBufferedView, ToyBufferedField>);

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
inline engine::memory::SimAllocator<ToyBufferedAlgorithm> MakeToyBufferedSim(
    std::size_t instance_count
) {
  std::vector<double> command;
  for (std::size_t i = 0; i < instance_count; ++i) {
    domain::math::Vector3<float> rest = ToyCommandRest(i);
    command.insert(command.end(), {rest.X(), rest.Y(), rest.Z()});
  }
  std::vector<domain::ArchetypeField> fields;
  fields.push_back(domain::ArchetypeField{"command", 3, std::move(command)});
  std::array<domain::Archetype, 1> archetypes = {domain::Archetype(
      "toy",
      {domain::ArchetypeTreeStructure::kNoParent},
      std::vector<domain::ArchetypeJointHandle>(instance_count),
      true,
      std::move(fields)
  )};
  return engine::memory::SimAllocator<ToyBufferedAlgorithm>(archetypes);
}

}  // namespace achilles::test_support
