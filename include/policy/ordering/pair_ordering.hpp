// One row per pair of another ordering's rows that a filter admits.

#pragma once

#include <cstddef>
#include <functional>
#include <span>
#include <string_view>
#include <type_traits>
#include <vector>

#include "domain/archetype/archetype.hpp"
#include "domain/topology/joint_topology.hpp"
#include "policy/filter/base_filter.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "util/tmp.hpp"

namespace achilles::policy::ordering {

// Shared by every PairOrdering: the pairs of `row_plan`'s rows, a lane
// group at a time (see PairOrdering), keeping each pair `admits` -- a
// filter's verdict on two real rows -- allows, as long as the partner is
// above the source and neither is the other's parent.
domain::topology::OrderingPlan BuildPairPlan(
    const domain::topology::OrderingPlan& row_plan,
    const std::function<bool(std::size_t, std::size_t)>& admits,
    std::size_t lane_size
);

// One row per pair of RowOrdering's rows that might interact -- bodies
// that might collide, say -- worked out once from the archetypes rather
// than every tick. Rows come a lane group at a time: one source row of
// RowOrdering against one lane group of its rows, lane t holding the pair
// (source, group * lanes + t). Each row's topology entry is its source, and
// its partner the other row (domain::PairTopology), so a pass over pairs
// (PairTraversal, policy/traversal/) can read RowOrdering's fields
// at either row and its own at the pair.
//
// A pair is kept when its partner is a real row above the source -- so
// each unordered pair appears once, and never a row against itself --
// neither row is the other's parent (a joint's two bodies always touch
// where they meet), and Filter admits it. A lane group is kept when any of
// its lanes is; its other lanes are padding, partnered with nothing
// (domain::PairTopology::kNoPartner). Everything that varies tick to tick
// is the pass's own business.
//
// The pairs are this plan's one JointLayout instance, its kept pairs that
// instance's joints in row order, the same way a ListOrdering numbers a
// list. Plans RowOrdering again to find its rows, as ListOrdering plans its
// LinkOrdering.
template <OrderingPolicyLike RowOrdering, filter::PairFilterLike Filter>
struct PairOrdering {
  using Topology = domain::topology::PairTopology;

  static domain::topology::OrderingPlan Build(
      std::span<const Archetype> tree, std::size_t lane_size
  ) {
    const domain::topology::OrderingPlan row_plan =
        RowOrdering::Build(tree, lane_size);
    const domain::layout::JointLayout layout = row_plan.Layout();

    // Each global instance's archetype, then each real row's key.
    std::vector<std::size_t> archetype_of;
    for (std::size_t a = 0; a < tree.size(); ++a) {
      archetype_of.insert(archetype_of.end(), tree[a].root_parents.size(), a);
    }
    using Key = decltype(Filter::Key(tree[0], 0));
    std::vector<Key> keys(layout.PaddedSize());
    for (std::size_t row = 0; row < layout.PaddedSize(); ++row) {
      if (!layout.IsPadding(row)) {
        const domain::layout::JointRef joint = layout.ToPhysical(row);
        keys[row] = Filter::Key(
            tree[archetype_of[joint.instance_index]], joint.joint_index
        );
      }
    }
    return BuildPairPlan(
        row_plan,
        [&](std::size_t source, std::size_t partner) {
          return Filter::Admits(keys[source], keys[partner]);
        },
        lane_size
    );
  }
};

}  // namespace achilles::policy::ordering
