#include <gtest/gtest.h>

#include <cstddef>
#include <string_view>
#include <vector>

#include "domain/archetype/archetype.hpp"
#include "domain/layout/joint_records.hpp"
#include "policy/filter/group_mask_filter.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "policy/ordering/linear_ordering.hpp"
#include "policy/ordering/list_ordering.hpp"
#include "policy/ordering/pair_ordering.hpp"
#include "policy/ordering/topological_ordering.hpp"

using achilles::domain::archetype::Archetype;
using achilles::domain::layout::JointField;
using achilles::domain::layout::JointRef;
using achilles::policy::filter::GroupMaskFilter;
using achilles::policy::filter::PairFilterLike;

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
      achilles::domain::layout::kNoParent, 0, 0
  };
  std::vector<JointRef> root_parents = {JointRef{
      achilles::domain::layout::kNoParent, achilles::domain::layout::kNoParent
  }};
  std::vector<JointField> fields = {
      JointField{"toy_group", 1, {1, 2, 4}},
      JointField{"toy_mask", 1, {2, 1, 0}},
  };

  Archetype Tree() const {
    return {"joints", tree_structure, root_parents, true, fields};
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
  const Archetype tree = joints.Tree();
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
  const Archetype tree = joints.Tree();
  const auto key = Filter::Key(tree, 0);
  EXPECT_EQ(key.group, 0U);
  EXPECT_EQ(key.mask, 0U);
  EXPECT_FALSE(Filter::Admits(key, key));
}
