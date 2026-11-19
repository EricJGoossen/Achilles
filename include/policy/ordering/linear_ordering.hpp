// Every instance on its own, in archetype order: the default ordering.

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

// No dependency structure between instances -- every instance stands alone,
// so nothing needs to be pushed out to a strictly later row than anything
// else the way TopologicalOrdering pushes a child later than its parent.
// Consolidate still applies (each archetype's block is placed back to back
// and lane-padded), just fed identity LocalOrders instead of sorted ones.
//
struct LinearOrdering : public BaseOrdering {
  static domain::topology::OrderingPlan Build(
      std::span<const Archetype> tree, std::size_t lane_size
  );
};

}  // namespace achilles::policy::ordering
