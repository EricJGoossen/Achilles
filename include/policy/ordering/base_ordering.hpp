// The base every ordering policy derives from, and its concept.
// Doc: docs/code/policy_ordering_base_ordering.md

#pragma once

#include <cstddef>
#include <functional>
#include <span>
#include <string_view>
#include <type_traits>
#include <vector>

#include "domain/archetype/archetype.hpp"
#include "domain/topology/joint_topology.hpp"
#include "domain/topology/ordering_plan.hpp"
#include "util/tmp.hpp"

namespace achilles::policy::ordering {

using domain::archetype::Archetype;

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
  // This archetype's own Archetype::root_parents, unchanged
  // -- ignored entirely when is_root_archetype is true.
  std::span<const domain::layout::JointRef> root_parents;
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
    requires(std::span<const Archetype> tree, std::size_t lane_size) {
      {
        P::Build(tree, lane_size)
      } -> std::same_as<domain::topology::OrderingPlan>;
    };

class BaseOrdering {
 protected:
  struct ConsolidatedOrder {
    // Indexed by original archetype index, then by physical instance-major
    // index within that archetype (instance * k + local_joint): the
    // absolute global sorted row.
    std::vector<std::vector<std::size_t>> physical_to_sorted;
    // Indexed by original archetype index, then by instance: the absolute
    // row of that instance's resolved external parent, or
    // domain::kNoParent for a root archetype's instances
    // (substituting that sentinel with the reserved base row is Build's
    // job, not this one's).
    std::vector<std::vector<std::size_t>> root_parent_rows;
    // The inverse of physical_to_sorted, indexed by absolute row: which
    // (instance, local joint) lives there, or {kNoParent, kNoParent} for a
    // padding row that no real joint ever occupies.
    std::vector<domain::layout::JointRef> sorted_to_physical;
    // Total rows across every archetype's block. Each block's own size
    // (its joint count times its lane-rounded instance count) is already a
    // lane_size multiple, so this sum is one too.
    std::size_t padded_size = 0;
  };

  // Takes every archetype's own local joint ordering (in original
  // archetype-index order, matching how
  // domain::JointRef::instance_index is numbered) and a placement
  // order for their blocks, and lays every archetype's instances out back to
  // back: each archetype gets a block of local_topo_order.size() *
  // RoundUp(root_parents.size(), lane_size) rows, placed in `placement_order`,
  // with instance i's joint at sorted local position p landing at that block's
  // row (p * padded_instance_count + i)
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
  // ordering with its Archetype (root_parents,
  // is_root_archetype), consolidates them, and resolves the one thing
  // Consolidate leaves to the caller -- the per-row `parents` array a
  // JointTopology needs -- by walking each instance's own tree_structure
  // for its interior joints and falling back to Consolidate's
  // root_parent_rows (or the reserved base row) at each instance's root.
  // Padding rows also default to the base row -- nothing ever resolves a
  // parent into the padded tail of an archetype's instance range (see the
  // .cpp), so there's no need for a separate scratch/sink row to protect.
  static domain::topology::OrderingPlan BuildFromLocalOrders(
      std::span<const Archetype> tree,
      std::span<const std::vector<std::size_t>> local_topo_orders,
      std::span<const std::size_t> placement_order,
      std::size_t lane_size
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
using TopologyOfT = util::DetectedOrT<
    domain::topology::JointTopology,
    detail::DeclaredTopology,
    Policy>;

}  // namespace achilles::policy::ordering
