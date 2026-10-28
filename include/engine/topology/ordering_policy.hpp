#pragma once

#include <cstddef>
#include <span>
#include <string_view>
#include <type_traits>
#include <vector>

#include "domain/archetype.hpp"
#include "domain/joint_topology.hpp"
#include "engine/topology/ordering_plan.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::topology {

using domain::ArchetypeJointHandle;
using domain::ArchetypeTreeStructure;

// One archetype's own local joint ordering, bundled with what Consolidate
// needs to place and wire up its block. TopologicalOrdering builds this
// from SortTree's output; LinearOrdering builds it with an identity
// local_topo_order (no per-joint reordering, since "no dependency
// structure between instances" means there's nothing to reorder for).
struct LocalOrder {
  // Local joint -> sorted local position, dense over [0, k). Every
  // instance of this archetype shares this exact ordering, which is what
  // lets Consolidate interleave them: instance i's joint at sorted
  // position p and instance j's joint at that same position p are always
  // exactly padded_instance_count apart.
  std::span<const std::size_t> local_topo_order;
  // This archetype's own ArchetypeTreeStructure::root_parents, unchanged
  // -- ignored entirely when is_root_archetype is true.
  std::span<const ArchetypeJointHandle> root_parents;
  bool is_root_archetype = false;
};

// The per-field sequencing/padding strategy -- how a field's instances get
// numbered into sorted rows and padded out to a lane multiple. A stateless
// policy type, mirroring PlanarLayout (layout_policy.hpp): the kind that
// knows what "unordered" or "topological" MEANS is also the kind that
// implements it, so a new ordering is a new policy struct here, not a new
// branch inside one central builder.
template <typename P>
concept OrderingPolicyLike =
    std::is_empty_v<P> &&
    requires(std::span<ArchetypeTreeStructure> tree, std::size_t lane_size) {
      { P::Build(tree, lane_size) } -> std::same_as<OrderingPlan>;
    };

class Ordering {
 protected:
  struct ConsolidatedOrder {
    // Indexed by original archetype index, then by physical instance-major
    // index within that archetype (instance * k + local_joint): the
    // absolute global sorted row.
    std::vector<std::vector<std::size_t>> physical_to_sorted;
    // Indexed by original archetype index, then by instance: the absolute
    // row of that instance's resolved external parent, or
    // ArchetypeTreeStructure::kNoParent for a root archetype's instances
    // (substituting that sentinel with the reserved base row is Build's
    // job, not this one's).
    std::vector<std::vector<std::size_t>> root_parent_rows;
    // The inverse of physical_to_sorted, indexed by absolute row: which
    // (instance, local joint) lives there, or {kNoParent, kNoParent} for a
    // padding row that no real joint ever occupies.
    std::vector<ArchetypeJointHandle> sorted_to_physical;
    // Total rows across every archetype's block. Each block's own size
    // (its joint count times its lane-rounded instance count) is already a
    // lane_size multiple, so this sum is one too.
    std::size_t padded_size = 0;
  };

  // Takes every archetype's own local joint ordering (in original
  // archetype-index order, matching how ArchetypeJointHandle::instance_index is
  // numbered) and a placement order for their blocks, and lays every
  // archetype's instances out back to back: each archetype gets a block of
  // local_topo_order.size() * RoundUp(root_parents.size(), lane_size) rows,
  // placed in `placement_order`, with instance i's joint at sorted local
  // position p landing at that block's row (p * padded_instance_count + i)
  // -- so a fixed p groups every instance's corresponding joint together,
  // and a lane group never straddles two archetypes. Also resolves each
  // instance's external parent (root_parents) to an absolute row, via the
  // target archetype's own placement and LocalOrder.
  static ConsolidatedOrder Consolidate(
      std::span<const LocalOrder> archetypes,
      std::span<const std::size_t> placement_order,
      std::size_t lane_size
  );

  // Shared by both policies' Build: pairs each archetype's own local joint
  // ordering with its ArchetypeTreeStructure (root_parents,
  // is_root_archetype), consolidates them, and resolves the one thing
  // Consolidate leaves to the caller -- the per-row `parents` array a
  // JointTopology needs -- by walking each instance's own tree_structure
  // for its interior joints and falling back to Consolidate's
  // root_parent_rows (or the reserved base row) at each instance's root.
  // Padding rows also default to the base row -- nothing ever resolves a
  // parent into the padded tail of an archetype's instance range (see the
  // .cpp), so there's no need for a separate scratch/sink row to protect.
  static OrderingPlan BuildFromLocalOrders(
      std::span<ArchetypeTreeStructure> tree,
      std::span<const std::vector<std::size_t>> local_topo_orders,
      std::span<const std::size_t> placement_order,
      std::size_t lane_size
  );
};

// The single-tree topological sort: dependency-level order (BFS from the
// roots), each level padded to a lane multiple so no parent/child pair ever
// shares a lane group, plus the reserved base row.
//
struct TopologicalOrdering : public Ordering {
 public:
  static OrderingPlan Build(
      std::span<ArchetypeTreeStructure> tree, std::size_t lane_size
  );

 private:
  // Takes one archetype's own instance shape (a single tree: tree_structure
  // has exactly one kNoParent root -- that instance's own root joint) and
  // creates a mapping from local joint index to sorted local position
  // (dense over [0, tree_structure.size())) in dependency order, while
  // preserving the original span. Every instance of this archetype shares
  // this exact ordering -- SortTree runs once per archetype, not once per
  // instance -- so interleaving different instances together is entirely
  // Consolidate's job, not this one's.
  static std::vector<std::size_t> SortTree(
      std::span<const std::size_t> tree_structure
  );

  // Computes each archetype's dependency depth: the one archetype flagged
  // is_root_archetype is depth 0; every other archetype is one more than
  // the deepest archetype any of its instances attaches into (decoding
  // root_parents' instance_index against every archetype's instance count,
  // in `tree` order). This is what Build sorts archetype blocks by.
  //
  // A depth is only well-defined if every instance of one archetype can
  // agree on it, which fails if the archetype (in)directly depends on
  // itself -- e.g. one instance attaching to another instance of that same
  // archetype, which Consolidate has no way to place: both instances share
  // one block, so they can't also disagree about which one comes first.
  // That case reports back as kNoParent instead of a depth. Build should
  // assert no entry is kNoParent before using this to order archetype
  // blocks -- unsupported for now, so it's a precondition violation, not
  // something to silently work around here.
  static std::vector<std::size_t> ArchetypeDepths(
      std::span<const ArchetypeTreeStructure> tree
  );
};

// No dependency structure between instances -- every instance stands alone,
// so nothing needs to be pushed out to a strictly later row than anything
// else the way TopologicalOrdering pushes a child later than its parent.
// Consolidate still applies (each archetype's block is placed back to back
// and lane-padded), just fed identity LocalOrders instead of sorted ones.
//
struct LinearOrdering : public Ordering {
  static OrderingPlan Build(
      std::span<ArchetypeTreeStructure> tree, std::size_t lane_size
  );
};

namespace detail {
template <typename Policy>
using DeclaredTopology = typename Policy::Topology;
}  // namespace detail

// The topology type an ordering's rows come with: domain::JointTopology,
// unless the policy declares another (`using Topology = ...`) -- a list
// ordering's domain::SegmentedTopology. So only the orderings that need
// segments pay for them.
template <typename Policy>
using TopologyOfT =
    util::DetectedOrT<domain::JointTopology, detail::DeclaredTopology, Policy>;

// Every attachment (domain::ArchetypeAttachment) whose list is `list`, as
// the joint it's linked to: archetype by archetype, instance by instance,
// in each archetype's own declaration order -- the order ListOrdering
// numbers its rows in.
std::vector<ArchetypeJointHandle> CollectAttachments(
    std::span<ArchetypeTreeStructure> tree, std::string_view list
);

// Shared by every ListOrdering: one row per entry of each of `lists`, list
// by list, each linked to the row its entry names in `link_layout` (the
// link ordering's own JointLayout). List k's rows are segment k of the
// plan's topology, and its entry j is "instance k, joint j" of the plan's
// own JointLayout. Padding rows, after every list, link to the link
// ordering's base row.
OrderingPlan BuildListPlan(
    std::span<const std::vector<ArchetypeJointHandle>> lists,
    const domain::JointLayout& link_layout,
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
// alone (SegmentTraversal, engine/pass/traversals.hpp). Each row's topology
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
  using Topology = domain::SegmentedTopology;

  static OrderingPlan Build(
      std::span<ArchetypeTreeStructure> tree, std::size_t lane_size
  ) {
    const OrderingPlan link_plan = LinkOrdering::Build(tree, lane_size);
    const std::vector<std::vector<ArchetypeJointHandle>> lists = {
        CollectAttachments(tree, Tags::kListName)...
    };
    return BuildListPlan(lists, link_plan.Layout(), lane_size);
  }
};

}  // namespace achilles::engine::topology
