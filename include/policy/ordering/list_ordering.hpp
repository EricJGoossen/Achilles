// One row per list entry, each linked to a row of another ordering.

#pragma once

#include <cstddef>
#include <functional>
#include <span>
#include <string_view>
#include <type_traits>
#include <vector>

#include "domain/archetype/archetype.hpp"
#include "domain/topology/joint_topology.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "util/tmp.hpp"

namespace achilles::policy::ordering {

// Every attachment (domain::JointAttachment) whose list is `list`,
// as the joint it's linked to: archetype by archetype, instance by instance, in
// each archetype's own declaration order -- the order ListOrdering numbers its
// rows in.
std::vector<domain::layout::JointRef> CollectAttachments(
    std::span<const Archetype> tree, std::string_view list
);

// Shared by every ListOrdering: one row per entry of each of `lists`, list
// by list, each linked to the row its entry names in `link_layout` (the
// link ordering's own JointLayout). List k's rows are segment k of the
// plan's topology, and its entry j is "instance k, joint j" of the plan's
// own JointLayout. Padding rows, after every list, link to the link
// ordering's base row.
domain::topology::OrderingPlan BuildListPlan(
    std::span<const std::vector<domain::layout::JointRef>> lists,
    const domain::layout::JointLayout& link_layout,
    std::size_t lane_size
);

// A list's tag: names the list (Tag::kListName) the attachments it orders
// were declared under.
template <typename Tag>
concept ListTagLike = requires {
  { Tag::kListName } -> std::convertible_to<std::string_view>;
};

// Rows that aren't joints: one per attachment the archetypes declare under
// each of Lists' kListName (see CollectAttachments for the order within a
// list), list by list in Lists' order, padded to a lane multiple --
// sensors, say, rather than bodies. Lists is a util::TypeList of tags. Each
// list's rows are one segment of the topology
// (domain::JointTopology::SegmentBegin), so a pass can walk one list's rows
// alone (SegmentTraversal, policy/traversal/). Each row's topology
// entry is its *link*: the row, in LinkOrdering's own row space, of the
// joint that entry is attached to -- handed as a pass's source index, so an
// Op over a list can read any field ordered by LinkOrdering at the entry's
// joint, the way a TreeTraversal hands a joint's parent.
//
// Plans LinkOrdering again to find those rows rather than looking up its
// plan: a policy's Build is a pure function of the archetypes, so this
// lands on the exact rows the sim's own LinkOrdering plan does.
template <OrderingPolicyLike LinkOrdering, typename Lists>
struct ListOrdering;

template <OrderingPolicyLike LinkOrdering, ListTagLike... Tags>
  requires(sizeof...(Tags) > 0)
struct ListOrdering<LinkOrdering, util::TypeList<Tags...>> {
  // Its rows come in segments, one per list -- the only topology that
  // carries them (see TopologyOfT). List k is instance k of its JointLayout,
  // its entries in order, so the layout's instance offsets are the segment
  // boundaries.
  using Topology = domain::topology::SegmentedTopology;

  static domain::topology::OrderingPlan Build(
      std::span<const Archetype> tree, std::size_t lane_size
  ) {
    const domain::topology::OrderingPlan link_plan =
        LinkOrdering::Build(tree, lane_size);
    const std::vector<std::vector<domain::layout::JointRef>> lists = {
        CollectAttachments(tree, Tags::kListName)...
    };
    return BuildListPlan(lists, link_plan.Layout(), lane_size);
  }
};

}  // namespace achilles::policy::ordering
