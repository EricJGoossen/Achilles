#pragma once

#include <cstddef>

#include "engine/topology/orderings.hpp"
#include "interface/simulation.hpp"

namespace achilles::test_support {

// The row `joint` lives at in ABA's Views, for a test checking what's
// underneath the handles.
inline std::size_t JointRow(
    const interface::Simulation& sim, const interface::JointAddress& joint
) {
  return sim.Archetypes().RowOf(
      joint, sim.Context().LayoutFor<engine::topology::TopologicalOrdering>()
  );
}

}  // namespace achilles::test_support
