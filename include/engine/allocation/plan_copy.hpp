// Copies an ordering's plan into the arena, as the topology and layout the sim
// keeps.

#pragma once

#include <algorithm>
#include <bit>
#include <cstddef>
#include <span>
#include <vector>

#include "domain/layout/joint_layout.hpp"
#include "domain/topology/joint_topology.hpp"
#include "domain/topology/ordering_plan.hpp"

namespace achilles::engine::allocation {

// Places an OrderingPlan's arrays in the arena: the JointTopology (each row's
// parent), the JointLayout (each row's joint) and, for a pair ordering, each
// row's partner.
struct PlanCopy {
  struct Allocated {
    domain::topology::JointTopology topology;
    domain::layout::JointLayout layout;
    std::span<const std::size_t> partners;
  };

  // Arena bytes Into needs, including worst-case alignment slack for each
  // array.
  static std::size_t Bytes(const domain::topology::OrderingPlan& plan) {
    return ArrayBytes(plan.parents) + ArrayBytes(plan.instance_offsets) +
           ArrayBytes(plan.sorted_rows) + ArrayBytes(plan.joints) +
           ArrayBytes(plan.partners);
  }

  template <typename ArenaT>
  static Allocated Into(
      ArenaT& arena, const domain::topology::OrderingPlan& plan
  ) {
    return {
        domain::topology::JointTopology(
            CopyInto(arena, plan.parents), plan.lane_size
        ),
        domain::layout::JointLayout(
            CopyInto(arena, plan.instance_offsets),
            CopyInto(arena, plan.sorted_rows),
            CopyInto(arena, plan.joints)
        ),
        CopyInto(arena, plan.partners)
    };
  }

 private:
  // Nothing for an empty array: CopyInto doesn't allocate one.
  template <typename T>
  static std::size_t ArrayBytes(const std::vector<T>& values) {
    return values.empty() ? 0 : (values.size() * sizeof(T)) + (alignof(T) - 1);
  }

  // An empty array (a list ordering with no entries) takes no Arena space.
  template <typename ArenaT, typename T>
  static std::span<const T> CopyInto(
      ArenaT& arena, const std::vector<T>& values
  ) {
    if (values.empty()) {
      return {};
    }
    auto* storage =
        std::bit_cast<T*>(arena.Allocate(values.size() * sizeof(T), alignof(T))
        );
    std::ranges::copy(values, storage);
    return {storage, values.size()};
  }
};

}  // namespace achilles::engine::allocation
