// Rows in dependency order, each level padded so no parent shares a lane
// group with its child.

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

// The single-tree topological sort: dependency-level order (BFS from the
// roots), each level padded to a lane multiple so no parent/child pair ever
// shares a lane group, plus the reserved base row.
//
struct TopologicalOrdering : public BaseOrdering {
 public:
  static domain::topology::OrderingPlan Build(
      std::span<const Archetype> tree, std::size_t lane_size
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
      std::span<const Archetype> tree
  );
};

}  // namespace achilles::policy::ordering
