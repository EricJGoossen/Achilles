#pragma once

#include <cstdint>

#include "algorithms/shared_slots.hpp"
#include "domain/clock/sim_clock.hpp"
#include "engine/assembly/assembler.hpp"
#include "engine/spec/field_contract.hpp"
#include "engine/spec/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "policy/indexing/base_indexing.hpp"
#include "policy/indexing/global_indexing.hpp"
#include "policy/indexing/group_indexing.hpp"
#include "policy/indexing/row_indexing.hpp"
#include "policy/layout/base_layout.hpp"
#include "policy/layout/planar_layout.hpp"
#include "policy/layout/segmented_layout.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "policy/ordering/linear_ordering.hpp"
#include "policy/ordering/list_ordering.hpp"
#include "policy/ordering/pair_ordering.hpp"
#include "policy/ordering/topological_ordering.hpp"

namespace achilles::algorithms::clock {

enum class ClockField : std::uint8_t {
  kClock,  // the one domain::SimClock

  kCount
};
static_assert(engine::spec::FieldEnumLike<ClockField>);

template <ClockField F>
struct ClockFieldTraits;

// One value for the whole sim, shared so anything that needs the time reads
// this block.
template <>
struct ClockFieldTraits<ClockField::kClock> {
  using Assembler = engine::assembly::LeafAssembler<domain::clock::SimClock>;
  using SharedAs = SimClockSlot;
  using Ordering = policy::ordering::TopologicalOrdering;
  using Layout = policy::layout::PlanarLayout;
  using Indexing = policy::indexing::GlobalIndexing;
};
static_assert(engine::spec::FieldTraitsLike<ClockField, ClockFieldTraits>);

using ClockView = engine::view::ViewFactory<ClockField, ClockFieldTraits>;
static_assert(engine::spec::ViewLike<ClockView, ClockField>);

}  // namespace achilles::algorithms::clock
