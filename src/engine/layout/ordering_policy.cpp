#include "engine/topology/ordering_policy.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <span>
#include <utility>
#include <vector>

#include "engine/topology/layout.hpp"

namespace achilles::engine::topology {

namespace {
constexpr std::size_t kNoParent = ArchetypeTreeStructure::kNoParent;

constexpr std::size_t RoundUpToLane(std::size_t count, std::size_t lane_size) {
  return (count + lane_size - 1) / lane_size * lane_size;
}

// Prefix sums of each archetype's own instance count (root_parents.size()),
// in the same order as `archetypes` -- offsets[a] is the first global
// instance index belonging to archetype a, offsets.back() is the total
// instance count. Shared by ArchetypeDepths and Consolidate, since both
// need to decode an ArchetypeJointHandle::instance_index back into which
// archetype (and which of its instances) it names.
template <typename T>
std::vector<std::size_t> InstanceOffsets(std::span<const T> archetypes) {
  std::vector<std::size_t> offsets(archetypes.size() + 1, 0);
  for (std::size_t a = 0; a < archetypes.size(); ++a) {
    offsets[a + 1] = offsets[a] + archetypes[a].root_parents.size();
  }
  return offsets;
}

struct DecodedInstance {
  std::size_t archetype;
  std::size_t local_instance;
};

DecodedInstance DecodeInstance(
    std::span<const std::size_t> instance_offsets, std::size_t global_instance
) {
  assert(global_instance < instance_offsets.back());
  auto it = std::upper_bound(
      instance_offsets.begin(), instance_offsets.end(), global_instance
  );
  std::size_t archetype =
      static_cast<std::size_t>(it - instance_offsets.begin()) - 1;
  return {archetype, global_instance - instance_offsets[archetype]};
}

struct ChildrenCsr {
  std::vector<std::size_t> offsets;
  std::vector<std::size_t> children;
};

// CSR-style children lists for a tree given as one parent-per-node array:
// two passes over one count array rather than `n` individually-allocated
// per-node vectors. Split out of SortTree (rather than inlined there) so
// SortTree's own cognitive complexity stays under the project's threshold.
ChildrenCsr BuildChildrenCsr(std::span<const std::size_t> tree_structure) {
  const std::size_t n = tree_structure.size();

  std::vector<std::size_t> offsets(n + 1, 0);
  for (std::size_t i = 0; i < n; ++i) {
    std::size_t p = tree_structure[i];
    if (p != kNoParent) {
      assert(
          p < n &&
          "BuildChildrenCsr: tree_structure[i] names an out-of-range parent."
      );
      ++offsets[p + 1];
    }
  }
  for (std::size_t p = 0; p < n; ++p) {
    offsets[p + 1] += offsets[p];
  }

  std::vector<std::size_t> cursor(offsets.begin(), offsets.end() - 1);
  std::vector<std::size_t> children(n);
  for (std::size_t i = 0; i < n; ++i) {
    std::size_t p = tree_structure[i];
    if (p != kNoParent) {
      children[cursor[p]++] = i;
    }
  }
  return {std::move(offsets), std::move(children)};
}
}  // namespace

Layout Ordering::BuildFromLocalOrders(
    std::span<ArchetypeTreeStructure> tree,
    std::span<const std::vector<std::size_t>> local_topo_orders,
    std::span<const std::size_t> placement_order,
    std::size_t lane_size
) {
  std::vector<LocalOrder> locals(tree.size());
  for (std::size_t a = 0; a < tree.size(); ++a) {
    locals[a] = LocalOrder{
        local_topo_orders[a], tree[a].root_parents, tree[a].is_root_archetype
    };
  }

  ConsolidatedOrder consolidated =
      Consolidate(locals, placement_order, lane_size);

  // No dedicated sink row: a padding row's own parent index is never read
  // back by anything real (see the assignment loop below -- both the
  // interior-joint and root-resolution branches only ever address real
  // instances, 0..n-1, never the padded tail), so there's nothing to
  // protect it from. Defaulting it to the base row is just "any valid,
  // already-seeded row" -- one fewer reserved slot than inventing a
  // separate scratch row for a case nothing ever reads. Layout's
  // constructor requires parents to hold exactly one more row than
  // sorted_to_physical (the reserved base row itself, at index
  // padded_size) -- AllocateTopology never reads that last slot, but the
  // shape invariant still expects it to be there.
  std::size_t base_row = consolidated.padded_size;
  std::vector<std::size_t> parents(consolidated.padded_size + 1, base_row);

  for (std::size_t a = 0; a < tree.size(); ++a) {
    std::span<const std::size_t> tree_structure = tree[a].tree_structure;
    std::size_t k = tree_structure.size();
    std::size_t n = tree[a].root_parents.size();

    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < k; ++j) {
        std::size_t row = consolidated.physical_to_sorted[a][i * k + j];
        std::size_t local_parent = tree_structure[j];

        if (local_parent == kNoParent) {
          std::size_t resolved = consolidated.root_parent_rows[a][i];
          parents[row] = (resolved == kNoParent) ? base_row : resolved;
        } else {
          parents[row] =
              consolidated.physical_to_sorted[a][i * k + local_parent];
        }
      }
    }
  }

  return {
      std::move(consolidated.physical_to_sorted),
      std::move(consolidated.sorted_to_physical),
      std::move(parents),
      lane_size
  };
}

Ordering::ConsolidatedOrder Ordering::Consolidate(
    std::span<const LocalOrder> archetypes,
    std::span<const std::size_t> placement_order,
    std::size_t lane_size
) {
  assert(lane_size > 0 && "Ordering::Consolidate: lane_size must be positive.");
  assert(placement_order.size() == archetypes.size());

  std::vector<std::size_t> instance_offsets = InstanceOffsets(archetypes);

  // Every archetype's block, sized by its own joint count times its own
  // lane-rounded instance count, placed back to back in whatever order
  // `placement_order` gives -- TopologicalOrdering places dependency-
  // shallower archetypes first (see ArchetypeDepths); LinearOrdering
  // doesn't reorder them at all.
  std::vector<std::size_t> block_offset(archetypes.size());
  std::size_t running_total = 0;
  for (std::size_t a : placement_order) {
    block_offset[a] = running_total;
    std::size_t padded_instances =
        RoundUpToLane(archetypes[a].root_parents.size(), lane_size);
    running_total += archetypes[a].local_topo_order.size() * padded_instances;
  }

  auto GlobalRow = [&](std::size_t archetype,
                       std::size_t instance,
                       std::size_t local_joint) {
    const LocalOrder& local = archetypes[archetype];
    assert(local_joint < local.local_topo_order.size());
    std::size_t padded_instances =
        RoundUpToLane(local.root_parents.size(), lane_size);
    return block_offset[archetype] +
           local.local_topo_order[local_joint] * padded_instances + instance;
  };

  ConsolidatedOrder out;
  out.padded_size = running_total;
  out.physical_to_sorted.resize(archetypes.size());
  out.root_parent_rows.resize(archetypes.size());
  out.sorted_to_physical.assign(
      running_total, ArchetypeJointHandle{kNoParent, kNoParent}
  );

  for (std::size_t a = 0; a < archetypes.size(); ++a) {
    const LocalOrder& local = archetypes[a];
    std::size_t n = local.root_parents.size();
    std::size_t k = local.local_topo_order.size();

    out.physical_to_sorted[a].resize(n * k);
    out.root_parent_rows[a].resize(n);

    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < k; ++j) {
        std::size_t row = GlobalRow(a, i, j);
        out.physical_to_sorted[a][i * k + j] = row;
        out.sorted_to_physical[row] =
            ArchetypeJointHandle{instance_offsets[a] + i, j};
      }

      if (local.is_root_archetype) {
        out.root_parent_rows[a][i] = kNoParent;
        continue;
      }

      const ArchetypeJointHandle& parent = local.root_parents[i];
      DecodedInstance target =
          DecodeInstance(instance_offsets, parent.instance_index);
      out.root_parent_rows[a][i] = GlobalRow(
          target.archetype, target.local_instance, parent.joint_index
      );
    }
  }

  return out;
}

std::vector<std::size_t> TopologicalOrdering::SortTree(
    std::span<const std::size_t> tree_structure
) {
  const std::size_t n = tree_structure.size();
  auto [offsets, children] = BuildChildrenCsr(tree_structure);

  // BFS by level using two flat frontier buffers swapped level to level
  // (bounded by n, no per-level heap churn). One instance's own root seeds
  // the frontier -- interleaving across an archetype's many instances is
  // Consolidate's job, not this one's, since this runs once per archetype.
  std::vector<std::size_t> physical_to_sorted(n, kNoParent);
  std::vector<std::size_t> frontier;
  std::vector<std::size_t> next_frontier;
  frontier.reserve(n);
  next_frontier.reserve(n);

  std::size_t root_count = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (tree_structure[i] == kNoParent) {
      frontier.push_back(i);
      ++root_count;
    }
  }
  assert(
      root_count == 1 &&
      "SortTree: tree_structure must be a single instance's tree -- exactly "
      "one joint (that instance's own root) may have no local parent."
  );

  std::size_t row = 0;
  std::size_t visited = 0;
  while (!frontier.empty()) {
    for (std::size_t physical : frontier) {
      physical_to_sorted[physical] = row++;
    }
    visited += frontier.size();

    next_frontier.clear();
    for (std::size_t physical : frontier) {
      for (std::size_t k = offsets[physical]; k < offsets[physical + 1]; ++k) {
        next_frontier.push_back(children[k]);
      }
    }
    frontier.swap(next_frontier);
  }

  assert(
      visited == n &&
      "SortTree: tree_structure is not a single tree reachable from its "
      "root -- a cycle or an unreachable joint was found."
  );

  return physical_to_sorted;
}

std::vector<std::size_t> TopologicalOrdering::ArchetypeDepths(
    std::span<const ArchetypeTreeStructure> tree
) {
  assert(
      std::count_if(
          tree.begin(),
          tree.end(),
          [](const ArchetypeTreeStructure& a) { return a.is_root_archetype; }
      ) == 1 &&
      "ArchetypeDepths: exactly one archetype must be flagged "
      "is_root_archetype."
  );

  std::vector<std::size_t> instance_offsets = InstanceOffsets(tree);

  enum class VisitState : uint8_t { kUnvisited, kInProgress, kDone };
  std::vector<VisitState> state(tree.size(), VisitState::kUnvisited);
  std::vector<std::size_t> depth(tree.size(), 0);

  auto Resolve = [&](auto&& self, std::size_t archetype) -> std::size_t {
    if (state[archetype] == VisitState::kDone) {
      return depth[archetype];
    }
    if (state[archetype] == VisitState::kInProgress) {
      return kNoParent;  // an archetype depends on itself, directly or not
    }
    state[archetype] = VisitState::kInProgress;

    std::size_t result = 0;
    if (!tree[archetype].is_root_archetype) {
      std::size_t deepest_parent = 0;
      for (const ArchetypeJointHandle& parent : tree[archetype].root_parents) {
        DecodedInstance target =
            DecodeInstance(instance_offsets, parent.instance_index);
        std::size_t parent_depth = self(self, target.archetype);
        if (parent_depth == kNoParent) {
          state[archetype] = VisitState::kDone;
          depth[archetype] = kNoParent;
          return kNoParent;
        }
        deepest_parent = std::max(deepest_parent, parent_depth);
      }
      result = deepest_parent + 1;
    }

    state[archetype] = VisitState::kDone;
    depth[archetype] = result;
    return result;
  };

  for (std::size_t a = 0; a < tree.size(); ++a) {
    Resolve(Resolve, a);
  }
  return depth;
}

Layout TopologicalOrdering::Build(
    std::span<ArchetypeTreeStructure> tree, std::size_t lane_size
) {
  std::vector<std::size_t> depths = ArchetypeDepths(tree);
  assert(
      std::ranges::none_of(
          depths, [](std::size_t d) { return d == kNoParent; }
      ) &&
      "TopologicalOrdering::Build: an archetype (in)directly depends on "
      "itself -- e.g. two instances of one archetype attaching to each "
      "other -- which isn't supported."
  );

  // Dependency-shallower archetypes first, so a target archetype's block is
  // always placed before anything that attaches into it. std::sort, not
  // std::stable_sort: libstdc++'s stable_sort instantiates the deprecated
  // std::get_temporary_buffer internally. placement_order already starts
  // as 0..n-1 (via iota above), so breaking ties on the original index
  // reproduces the same stable order without it.
  std::vector<std::size_t> placement_order(tree.size());
  std::iota(placement_order.begin(), placement_order.end(), 0);
  std::ranges::sort(placement_order, [&](std::size_t a, std::size_t b) {
    if (depths[a] != depths[b]) {
      return depths[a] < depths[b];
    }
    return a < b;
  });

  std::vector<std::vector<std::size_t>> local_topo_orders(tree.size());
  for (std::size_t a = 0; a < tree.size(); ++a) {
    local_topo_orders[a] = SortTree(tree[a].tree_structure);
  }

  return BuildFromLocalOrders(
      tree, local_topo_orders, placement_order, lane_size
  );
}

Layout LinearOrdering::Build(
    std::span<ArchetypeTreeStructure> tree, std::size_t lane_size
) {
  // No sort, no archetype reordering -- every instance stands alone, so
  // there's nothing to push later than anything else for.
  std::vector<std::vector<std::size_t>> local_topo_orders(tree.size());
  for (std::size_t a = 0; a < tree.size(); ++a) {
    std::vector<std::size_t> identity(tree[a].tree_structure.size());
    std::iota(identity.begin(), identity.end(), 0);
    local_topo_orders[a] = std::move(identity);
  }

  std::vector<std::size_t> placement_order(tree.size());
  std::iota(placement_order.begin(), placement_order.end(), 0);

  return BuildFromLocalOrders(
      tree, local_topo_orders, placement_order, lane_size
  );
}

}  // namespace achilles::engine::topology
