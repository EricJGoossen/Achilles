// The maps a sim's build keeps per ordering plan and per shared slot.

#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cstddef>
#include <initializer_list>
#include <memory>
#include <span>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "domain/archetype/archetype.hpp"
#include "domain/archetype/archetype_set.hpp"
#include "domain/layout/joint_layout.hpp"
#include "domain/topology/joint_topology.hpp"
#include "domain/topology/ordering_plan.hpp"
#include "domain/topology/pair_topology.hpp"
#include "domain/topology/segmented_topology.hpp"
#include "engine/algorithm.hpp"
#include "engine/allocation/plan_copy.hpp"
#include "engine/binding.hpp"
#include "engine/context/sim_context.hpp"
#include "engine/memory/arena.hpp"
#include "engine/spec/field_binding.hpp"
#include "engine/spec/field_contract.hpp"
#include "engine/spec/step_contract.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_factory.hpp"
#include "policy/buffering/base_buffering.hpp"
#include "policy/buffering/double_buffering.hpp"
#include "policy/buffering/single_buffering.hpp"
#include "policy/layout/base_layout.hpp"
#include "policy/layout/planar_layout.hpp"
#include "policy/layout/segmented_layout.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "policy/ordering/linear_ordering.hpp"
#include "policy/ordering/list_ordering.hpp"
#include "policy/ordering/pair_ordering.hpp"
#include "policy/ordering/topological_ordering.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::allocation {

using domain::archetype::Archetype;

namespace detail {

// One OrderingPlan per distinct ordering policy actually named by some
// field of some hosted Algorithm, keyed by SlotId<Policy>() -- the same
// "distinct link-time address per type" trick SlotId already uses for
// SharedAs tags, reused here since an ordering policy is just another type
// identity that needs a runtime-comparable key. There is no single
// program-wide row order: a field's own row order is entirely a function
// of its own Traits<F>::Ordering (default policy::ordering::LinearOrdering),
// never of what any other field in the sim happens to want. Scratch: the plans
// die with the SimAllocator, after BuildTopologies has copied what persists
// into the Arena.
using PlanMap = std::unordered_map<const void*, domain::topology::OrderingPlan>;

struct SlotRecord {
  std::byte* base = nullptr;
  std::size_t block_bytes = 0;
  std::size_t alignment = 0;
  std::size_t lane_count = 0;
  const void* ordering_id = nullptr;
  const void* indexing_id = nullptr;
  const void* buffering_id = nullptr;
};
using SlotMap = std::unordered_map<const void*, SlotRecord>;

// The parity a field's View reads: none for a single-buffered field, its
// group's for a double-buffered one -- looked up by group in the parities
// SimAllocator carved before any View (see pass::ParityTupleFor).
template <
    policy::buffering::BufferingPolicyLike Buffering,
    typename ParityTuple>
  requires(!policy::buffering::GroupBufferedLike<Buffering>)
const policy::buffering::BufferParity*
ParityFor(const ParityTuple& /*parities*/) {
  return nullptr;
}

template <policy::buffering::GroupBufferedLike Buffering, typename ParityTuple>
const policy::buffering::BufferParity* ParityFor(const ParityTuple& parities) {
  return std::get<context::BufferParitySlot<typename Buffering::BufferGroup>>(
             parities
  )
      .parity;
}

}  // namespace detail

}  // namespace achilles::engine::allocation
