#pragma once

#include <cstdint>

#include "algorithms/shared_slots.hpp"
#include "domain/sim_clock.hpp"
#include "engine/assembler.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/orderings.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::clock {

enum class ClockField : std::uint8_t {
  kClock,  // the one domain::SimClock

  kCount
};
static_assert(engine::FieldEnumLike<ClockField>);

template <ClockField F>
struct ClockFieldTraits;

// One value for the whole sim, shared so anything that needs the time reads
// this block.
template <>
struct ClockFieldTraits<ClockField::kClock> {
  using Assembler = engine::LeafAssembler<domain::SimClock>;
  using SharedAs = SimClockSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  using Indexing = engine::topology::GlobalIndexing;
};
static_assert(engine::FieldTraitsLike<ClockField, ClockFieldTraits>);

using ClockView = engine::view::ViewFactory<ClockField, ClockFieldTraits>;
static_assert(engine::view::ViewLike<ClockView, ClockField>);

}  // namespace achilles::algorithms::clock
