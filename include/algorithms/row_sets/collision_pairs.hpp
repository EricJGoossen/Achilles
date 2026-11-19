#pragma once

#include <string_view>

#include "policy/filter/group_mask_filter.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "policy/ordering/linear_ordering.hpp"
#include "policy/ordering/list_ordering.hpp"
#include "policy/ordering/pair_ordering.hpp"
#include "policy/ordering/topological_ordering.hpp"

namespace achilles::algorithms::row_sets {

// The scene fields a body's collision groups and mask are given in --
// the broad phase's kCollisionGroup/kCollisionMask (broad_phase_data.hpp),
// read by name here before they're loaded.
struct CollisionMaskFields {
  static constexpr std::string_view kGroup = "collision_group";
  static constexpr std::string_view kMask = "collision_mask";
};

// The pairs of bodies that might collide: one row per pair of
// TopologicalOrdering's body rows whose groups and masks let them (see
// policy::ordering::PairOrdering, GroupMaskFilter). Every per-pair field of
// the collision pipeline -- the broad phase's overlap bits, the narrow
// phase's contacts, the contact solver's rows -- is ordered by it, and
// every pass over pairs walks its topology.
using CollisionPairs = policy::ordering::PairOrdering<
    policy::ordering::TopologicalOrdering,
    policy::filter::GroupMaskFilter<CollisionMaskFields>>;

}  // namespace achilles::algorithms::row_sets
