#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "domain/archetype.hpp"

namespace achilles::engine::topology {

// Names the two archetype fields a GroupMaskFilter reads: Fields::kGroup,
// the set of groups a row belongs to, and Fields::kMask, the set of groups
// it pairs with -- each one integer bitmask per joint.
template <typename Fields>
concept GroupMaskFieldsLike = requires {
  { Fields::kGroup } -> std::convertible_to<std::string_view>;
  { Fields::kMask } -> std::convertible_to<std::string_view>;
};

// A PairOrdering filter (orderings.hpp), Bullet-style: two rows pair
// when each one's group intersects the other's mask. Every instance of an
// archetype shares its joints' values, so instance 0's stand for all of
// them; a joint whose archetype gives no group pairs with nothing.
template <GroupMaskFieldsLike Fields>
struct GroupMaskFilter {
  struct Groups {
    std::uint64_t group = 0;
    std::uint64_t mask = 0;
  };

  static Groups Key(
      const domain::ArchetypeTreeStructure& archetype, std::size_t joint
  ) {
    return {
        Bits(archetype, Fields::kGroup, joint),
        Bits(archetype, Fields::kMask, joint)
    };
  }

  static bool Admits(const Groups& a, const Groups& b) {
    return (a.group & b.mask) != 0 && (b.group & a.mask) != 0;
  }

 private:
  static std::uint64_t Bits(
      const domain::ArchetypeTreeStructure& archetype,
      std::string_view name,
      std::size_t joint
  ) {
    for (const domain::ArchetypeField& field : archetype.fields) {
      if (field.name == name) {
        return static_cast<std::uint64_t>(domain::ArchetypeFieldAt(
            field, 0, joint, 0, archetype.tree_structure.size()
        ));
      }
    }
    return 0;
  }
};

}  // namespace achilles::engine::topology
