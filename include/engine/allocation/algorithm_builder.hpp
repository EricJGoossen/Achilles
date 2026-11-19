// Builds one algorithm's fields and view.

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

namespace detail {

// Recovers EnumT and the Traits template-template parameter from a concrete
// Algorithm<EnumT, Traits> via partial-specialization pattern matching --
// the same trick FixtureFor (tests/support/view_fixture.hpp) already
// uses to pick a resolved View type's Assemblers... back apart.
template <typename AlgorithmT>
struct AlgorithmBuilder;

template <typename EnumT, template <EnumT> class Traits>
struct AlgorithmBuilder<engine::Algorithm<EnumT, Traits>> {
  using View = typename engine::Algorithm<EnumT, Traits>::View;

  template <std::size_t... Is>
  static constexpr std::size_t MaxLaneCount(std::index_sequence<Is...>) {
    std::size_t result = 1;
    ((result = std::max(
          result,
          FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::LaneCount()
      )),
     ...);
    return result;
  }

  template <std::size_t... Is>
  static constexpr std::size_t MaxAlignment(std::index_sequence<Is...>) {
    std::size_t result = 1;
    ((result = std::max(
          result,
          FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::Alignment()
      )),
     ...);
    return result;
  }

  static constexpr std::size_t RequiredLaneSize() {
    return MaxLaneCount(
        std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{}
    );
  }
  static constexpr std::size_t RequiredAlignment() {
    return MaxAlignment(
        std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{}
    );
  }

  static std::size_t MeasureExact(
      MeasureSlotSet& counted, const PlanMap& plans
  ) {
    return MeasureAlgorithm<EnumT, Traits>(
        counted,
        std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{},
        plans
    );
  }

  // The View's row count per ordering its fields use (View::Orderings).
  template <typename... Orderings>
  static typename View::RowCounts RowCountsOf(
      util::TypeList<Orderings...> /*orderings*/, const PlanMap& plans
  ) {
    return {plans.at(engine::SlotId<Orderings>()).ViewInstanceCount()...};
  }

  template <typename ParityTuple, std::size_t... Is>
  static View CarveView(
      std::index_sequence<Is...>,
      Arena& arena,
      SlotMap& slots,
      const PlanMap& plans,
      const domain::archetype::ArchetypeSet& archetypes,
      const ParityTuple& parities
  ) {
    const std::array<std::size_t, sizeof...(Is)> row_counts{
        FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::RowCountFor(plans
        )...
    };
    const std::array<std::vector<std::size_t>, sizeof...(Is)> segments{
        FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::SegmentsFor(plans
        )...
    };
    std::array<std::byte*, sizeof...(Is)> type_bases{
        FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::Carve(
            arena, slots, row_counts[Is], segments[Is]
        )...
    };
    // Each field's RowTable: a segmented field's slot is initialized from
    // its table; an empty one has nothing to initialize.
    typename View::RowTables row_tables{
        FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::template RowSlot<
            Is>(segments[Is])...
    };
    // Seed after every field's block exists, so a shared field carved once
    // by an earlier algorithm still gets seeded exactly once here too (a
    // second SeedPadding call over the same memory is idempotent -- it
    // rewrites the same seed values -- so this is safe even for shared
    // fields visited from more than one Algorithm).
    (FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::SeedPadding(
         type_bases[Is],
         row_counts[Is],
         FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::RowsFor(plans)
     ),
     ...);
    // Populate after seeding, so real archetype data always wins over a
    // padding seed for any row that happens to be both (never possible
    // today -- ToSorted only ever names a real row -- but keeping the
    // order this way costs nothing and stays correct if that ever
    // changes).
    (FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::Populate(
         type_bases[Is],
         row_counts[Is],
         FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::RowsFor(plans),
         archetypes
     ),
     ...);
    // Last, once the first block holds its final initial contents.
    (FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::Replicate(
         type_bases[Is], row_counts[Is], segments[Is]
     ),
     ...);
    std::array<const policy::buffering::BufferParity*, sizeof...(Is)>
        field_parities{ParityFor<
            typename FieldBuilder<EnumT, Traits, static_cast<EnumT>(Is)>::
                FieldBuffering>(parities)...};
    return View(
        type_bases,
        RowCountsOf(typename View::Orderings{}, plans),
        field_parities,
        row_tables
    );
  }

  template <typename ParityTuple>
  static View Carve(
      Arena& arena,
      SlotMap& slots,
      const PlanMap& plans,
      const domain::archetype::ArchetypeSet& archetypes,
      const ParityTuple& parities
  ) {
    return CarveView(
        std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{},
        arena,
        slots,
        plans,
        archetypes,
        parities
    );
  }
};

}  // namespace detail

}  // namespace achilles::engine::allocation
