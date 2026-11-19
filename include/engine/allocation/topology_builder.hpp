// Plans every ordering and places its topology, and measures the arena.

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
#include "engine/allocation/field_builder.hpp"
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

// Same shared-slot deduplicate Carve performs for real, but pure counting: no
// Arena, no pointers, just how many bytes the real Carve pass will
// actually consume -- an exact (not
// upper-bound) budget, now that SimAllocator's Archetype-taking
// constructor knows real instance counts up front rather than being handed
// an arbitrary count by the caller. `counted` is shared across every
// algorithm being measured, so a SharedAs tag two algorithms both name is
// still only ever counted once, same as a real Carve pass would.
using MeasureSlotSet = std::unordered_set<const void*>;

template <typename EnumT, template <EnumT> class Traits, EnumT F>
std::size_t MeasureField(MeasureSlotSet& counted, const PlanMap& plans) {
  using V = FieldBuilder<EnumT, Traits, F>;
  std::size_t bytes =
      V::RegionBytes(V::RowCountFor(plans), V::SegmentsFor(plans));
  std::size_t alignment = V::Alignment();

  if constexpr (engine::FieldIsShared<EnumT, Traits, F>) {
    using Tag = typename V::FieldTraits::SharedAs;
    if (!counted.insert(engine::SlotId<Tag>()).second) {
      return 0;
    }
  }
  // Arena::Allocate rounds its cursor up to `alignment` before bumping, so
  // a block carved right after a smaller-aligned one can waste up to
  // alignment-1 bytes this exact per-field sum wouldn't otherwise account
  // for. Padding every counted block by that worst case keeps this a safe
  // budget (Arena::Allocate asserts rather than grows on overflow) instead
  // of one that could under-allocate by a few bytes of alignment slop.
  return bytes + (alignment - 1);
}

template <typename EnumT, template <EnumT> class Traits, std::size_t... Is>
std::size_t MeasureAlgorithm(
    MeasureSlotSet& counted, std::index_sequence<Is...>, const PlanMap& plans
) {
  return (
      MeasureField<EnumT, Traits, static_cast<EnumT>(Is)>(counted, plans) + ...
  );
}

// Plans one row order per distinct ordering policy in `List`, keyed the
// same way FieldBuilder::RowsFor looks them back up (SlotId<Policy>()).
template <typename List>
struct BuildPlanMap;

template <typename... Policies>
struct BuildPlanMap<util::TypeList<Policies...>> {
  static_assert(
      sizeof...(Policies) > 0,
      "SimAllocator: no ordering policy found across any hosted "
      "Algorithm's fields -- every FieldTraits must resolve to at least "
      "policy::ordering::LinearOrdering (the default)."
  );

  static PlanMap Build(
      std::span<const domain::archetype::Archetype> tree, std::size_t lane_size
  ) {
    PlanMap plans;
    (plans.emplace(
         engine::SlotId<Policies>(), Policies::Build(tree, lane_size)
     ),
     ...);
    return plans;
  }
};

// Allocates one JointTopology and JointLayout per distinct ordering policy
// in `List` (the same List BuildPlanMap above was built from), wrapped into
// the exact tuple shape SimContext<Algorithms...> expects
// (pass::TopologyTupleFor, engine/context/sim_context.hpp) rather than a
// second, independently-shaped tuple this file would otherwise have to keep
// hand-in-sync with it.
template <typename List>
struct BuildTopologyTuple;

template <typename... Policies>
struct BuildTopologyTuple<util::TypeList<Policies...>> {
  static std::tuple<context::TopologySlot<Policies>...> Build(
      const PlanMap& plans, Arena& arena
  ) {
    return std::tuple<context::TopologySlot<Policies>...>{MakeSlot<Policies>(
        PlanCopy::Into(arena, plans.at(engine::SlotId<Policies>()))
    )...};
  }

 private:
  template <typename Policy>
    requires std::same_as<
        policy::ordering::TopologyOfT<Policy>,
        domain::topology::JointTopology>
  static context::TopologySlot<Policy> MakeSlot(PlanCopy::Allocated allocated) {
    return {allocated.topology, allocated.layout};
  }

  template <typename Policy>
    requires std::same_as<
        policy::ordering::TopologyOfT<Policy>,
        domain::topology::SegmentedTopology>
  static context::TopologySlot<Policy> MakeSlot(PlanCopy::Allocated allocated) {
    return {
        domain::topology::SegmentedTopology(
            allocated.topology, allocated.layout.InstanceOffsets()
        ),
        allocated.layout
    };
  }

  template <typename Policy>
    requires std::same_as<
        policy::ordering::TopologyOfT<Policy>,
        domain::topology::PairTopology>
  static context::TopologySlot<Policy> MakeSlot(PlanCopy::Allocated allocated) {
    return {
        domain::topology::PairTopology(allocated.topology, allocated.partners),
        allocated.layout
    };
  }
};

}  // namespace detail

}  // namespace achilles::engine::allocation
