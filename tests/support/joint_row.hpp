#pragma once

#include <cstddef>

#include "interface/simulation.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "policy/ordering/linear_ordering.hpp"
#include "policy/ordering/list_ordering.hpp"
#include "policy/ordering/pair_ordering.hpp"
#include "policy/ordering/topological_ordering.hpp"

namespace achilles::test_support {

// The row `joint` lives at in ABA's Views, for a test checking what's
// underneath the handles.
inline std::size_t JointRow(
    const interface::Simulation& sim, const interface::JointAddress& joint
) {
  return sim.Archetypes().RowOf(
      joint, sim.Context().LayoutFor<policy::ordering::TopologicalOrdering>()
  );
}

}  // namespace achilles::test_support
