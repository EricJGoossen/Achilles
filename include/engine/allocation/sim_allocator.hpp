// Builds the arena and the SimContext from the archetypes at init.
// Doc: docs/code/engine_allocation_sim_allocator.md

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
#include "engine/allocation/algorithm_builder.hpp"
#include "engine/allocation/plan_copy.hpp"
#include "engine/allocation/topology_builder.hpp"
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

// Owns the Arena for one sim. One JointTopology/JointLayout per distinct
// ordering policy any hosted field names; one View per hosted algorithm.
// Archetypes are runtime, supplied once at construction; to resize,
// destroy this SimAllocator and build a new one -- there's no in-place
// resize.
template <engine::AlgorithmLike... Algorithms>
class SimAllocator {
  // Reuses SimContext's own computation (engine/context/sim_context.hpp)
  // rather than keeping a second, independent copy here that this file's
  // BuildPlanMap/BuildTopologyTuple and SimContext's own topology tuple
  // could silently drift out of sync against.
  using UsedOrderings = context::UsedOrderingsFor<Algorithms...>;

 public:
  struct State {
    engine::memory::Arena arena;
    context::SimContext<Algorithms...> context;
  };

  // Builds every topology/layout/View from archetypes; archetypes need not
  // outlive this call.
  explicit SimAllocator(const domain::archetype::ArchetypeSet& archetypes)
      : plans_(BuildPlans(archetypes)),
        state_({{MeasureBytes(), RequiredAlignment()}, BuildContext(archetypes)}
        ) {}

  // Constructs and immediately extracts -- convenience for the common case.
  static State Build(const domain::archetype::ArchetypeSet& archetypes) {
    return SimAllocator(archetypes).Extract();
  }

  // Cheap fresh copy; doesn't borrow from this SimAllocator, only the Arena.
  context::SimContext<Algorithms...> context::SimContext() const {
    return state_.context;
  }

  // Rvalue-qualified: can't Extract() a SimAllocator still meant for use.
  State Extract() && { return std::move(state_); }

  // Forwards to the built SimContext's own LayoutFor.
  template <policy::layout::LayoutPolicyLike PolicyT>
  domain::layout::JointLayout LayoutFor() const {
    return state_.context.template LayoutFor<PolicyT>();
  }

  // Forwards to the built SimContext's own TopologyFor.
  template <policy::layout::LayoutPolicyLike PolicyT>
  policy::ordering::TopologyOfT<PolicyT> TopologyFor() const {
    return state_.context.template TopologyFor<PolicyT>();
  }

  // Forwards to the built SimContext's own ViewFor.
  template <engine::AlgorithmLike AlgorithmT>
  typename AlgorithmT::View ViewFor() const {
    return state_.context.template ViewFor<AlgorithmT>();
  }

  // Largest lane size any hosted algorithm requires.
  static constexpr std::size_t RequiredLaneSize() {
    return std::max(
        {std::size_t{1},
         detail::AlgorithmBuilder<Algorithms>::RequiredLaneSize()...}
    );
  }

  // Largest alignment any hosted algorithm requires.
  static constexpr std::size_t RequiredAlignment() {
    return std::max(
        {std::size_t{1},
         detail::AlgorithmBuilder<Algorithms>::RequiredAlignment()...}
    );
  }

 private:
  // Plans one row order per distinct ordering policy named by any hosted
  // field.
  static detail::PlanMap BuildPlans(
      const domain::archetype::ArchetypeSet& archetypes
  ) {
    return detail::BuildPlanMap<UsedOrderings>::Build(
        archetypes.All(), RequiredLaneSize()
    );
  }

  // Exact byte cost per field, deduped across shared slots.
  std::size_t MeasureBytes() const {
    detail::MeasureSlotSet counted;
    std::size_t total =
        (detail::AlgorithmBuilder<Algorithms>::MeasureExact(counted, plans_) +
         ...);
    for (const auto& [id, plan] : plans_) {
      total += PlanCopy::Bytes(plan);
    }
    total += std::tuple_size_v<context::ParityTupleFor<Algorithms...>> *
             (sizeof(policy::buffering::BufferParity) +
              (alignof(policy::buffering::BufferParity) - 1));
    return total;
  }

  // Everything SimContext holds, carved out of state_.arena (already
  // constructed by the time state_'s second member is -- see the
  // constructor): the topologies, one parity per buffer group, then every
  // View, each handed its fields' groups' parities.
  context::SimContext<Algorithms...> BuildContext(
      const domain::archetype::ArchetypeSet& archetypes
  ) {
    auto topologies = BuildTopologies();
    auto parities = std::apply(
        [this](auto... slots) {
          return context::ParityTupleFor<Algorithms...>{CarveParity(slots)...};
        },
        context::ParityTupleFor<Algorithms...>{}
    );
    auto views = BuildViews(
        archetypes, parities, std::make_index_sequence<sizeof...(Algorithms)>{}
    );
    return context::SimContext<Algorithms...>(
        std::move(topologies), std::move(views), parities
    );
  }

  // Value-initialized, so every group starts out reading half 0.
  template <typename Group>
  context::BufferParitySlot<Group> CarveParity(
      context::BufferParitySlot<Group> /*group*/
  ) {
    std::byte* bytes = state_.arena.Allocate(
        sizeof(policy::buffering::BufferParity),
        alignof(policy::buffering::BufferParity)
    );
    return {std::construct_at(
        std::bit_cast<policy::buffering::BufferParity*>(bytes)
    )};
  }

  // Allocates one JointTopology and JointLayout per distinct ordering
  // policy in the Arena, wrapped into the exact tuple shape
  // SimContext<Algorithms...> expects.
  context::TopologyTupleFor<Algorithms...> BuildTopologies() {
    return detail::BuildTopologyTuple<UsedOrderings>::Build(
        plans_, state_.arena
    );
  }

  // Pack order is left-to-right: first algorithm naming a SharedAs tag owns it.
  template <std::size_t... Is>
  std::tuple<typename Algorithms::View...> BuildViews(
      const domain::archetype::ArchetypeSet& archetypes,
      const context::ParityTupleFor<Algorithms...>& parities,
      std::index_sequence<Is...> /*algorithms*/
  ) {
    detail::SlotMap slots;
    return std::tuple<typename Algorithms::View...>(
        detail::AlgorithmBuilder<Algorithms>::Carve(
            state_.arena, slots, plans_, archetypes, parities
        )...
    );
  }

  detail::PlanMap plans_;
  State state_;
};

// Builds SimAllocator<Algorithms...> from a util::TypeList<Algorithms...>
// instead of a caller restating the pack -- lets every hosted Algorithm be
// registered exactly once (see e.g. algorithms/registry.hpp) and reused
// here for the Sim type itself. Nothing downstream needs its own separate
// registration: engine::op_invoker::Step's own Algorithms... pack is deduced
// from whichever SimAllocator it's given, so building the Sim type this way is
// the only place the registered list has to be named at all.
template <typename List>
struct SimAllocatorFor;

template <engine::AlgorithmLike... Algorithms>
struct SimAllocatorFor<util::TypeList<Algorithms...>> {
  using Type = SimAllocator<Algorithms...>;
};

template <typename List>
using SimAllocatorForT = typename SimAllocatorFor<List>::Type;

}  // namespace achilles::engine::allocation
