#include <gtest/gtest.h>

#include <cstddef>
#include <string_view>
#include <vector>

#include "domain/archetype.hpp"
#include "engine/topology/group_mask_filter.hpp"
#include "engine/topology/orderings.hpp"

using achilles::domain::ArchetypeField;
using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::engine::topology::GroupMaskFilter;
using achilles::engine::topology::PairFilterLike;

namespace {

struct ToyFields {
  static constexpr std::string_view kGroup = "toy_group";
  static constexpr std::string_view kMask = "toy_mask";
};
using Filter = GroupMaskFilter<ToyFields>;
static_assert(PairFilterLike<Filter>);

// One instance of three joints, each with its own group and mask.
struct ThreeJoints {
  std::vector<std::size_t> tree_structure = {
      ArchetypeTreeStructure::kNoParent, 0, 0
  };
  std::vector<ArchetypeJointHandle> root_parents = {ArchetypeJointHandle{
      ArchetypeTreeStructure::kNoParent, ArchetypeTreeStructure::kNoParent
  }};
  std::vector<ArchetypeField> fields = {
      ArchetypeField{"toy_group", 1, {1, 2, 4}},
      ArchetypeField{"toy_mask", 1, {2, 1, 0}},
  };

  ArchetypeTreeStructure Tree() const {
    return ArchetypeTreeStructure{
        true, tree_structure, root_parents, {}, fields
    };
  }
};

}  // namespace

TEST(GroupMaskFilter, KeyReadsEachJointsGroupAndMask) {
  ThreeJoints joints;
  const auto key = Filter::Key(joints.Tree(), 1);
  EXPECT_EQ(key.group, 2U);
  EXPECT_EQ(key.mask, 1U);
}

TEST(GroupMaskFilter, AdmitsOnlyWhenEachGroupMeetsTheOthersMask) {
  ThreeJoints joints;
  const ArchetypeTreeStructure tree = joints.Tree();
  // 0 (group 1, mask 2) and 1 (group 2, mask 1): both ways.
  EXPECT_TRUE(Filter::Admits(Filter::Key(tree, 0), Filter::Key(tree, 1)));
  EXPECT_TRUE(Filter::Admits(Filter::Key(tree, 1), Filter::Key(tree, 0)));
  // 0 and 2 (group 4, mask 0): 2's mask takes nothing.
  EXPECT_FALSE(Filter::Admits(Filter::Key(tree, 0), Filter::Key(tree, 2)));
  // 0 with itself: group 1 isn't in mask 2.
  EXPECT_FALSE(Filter::Admits(Filter::Key(tree, 0), Filter::Key(tree, 0)));
}

TEST(GroupMaskFilter, AJointWithNoFieldsPairsWithNothing) {
  ThreeJoints joints;
  joints.fields.clear();
  const ArchetypeTreeStructure tree = joints.Tree();
  const auto key = Filter::Key(tree, 0);
  EXPECT_EQ(key.group, 0U);
  EXPECT_EQ(key.mask, 0U);
  EXPECT_FALSE(Filter::Admits(key, key));
}
