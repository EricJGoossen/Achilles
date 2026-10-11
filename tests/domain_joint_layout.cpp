#include <gtest/gtest.h>

#include <cstddef>
#include <span>
#include <vector>

#include "domain/archetype.hpp"
#include "domain/joint_layout.hpp"

using achilles::domain::ArchetypeJointHandle;
using achilles::domain::JointLayout;

namespace {

constexpr std::size_t kNone = JointLayout::kNoInstance;

// Two instances: instance 0 has two joints, instance 1 has one. Rows:
// 0 = (0,0), 1 = (1,0), 2 = padding, 3 = (0,1).
struct TwoInstanceLayout {
  std::vector<std::size_t> instance_offsets = {0, 2, 3};
  std::vector<std::size_t> sorted_rows = {0, 3, 1};
  std::vector<ArchetypeJointHandle> joints = {
      {0, 0}, {1, 0}, {kNone, kNone}, {0, 1}
  };

  JointLayout View() const { return {instance_offsets, sorted_rows, joints}; }
};

}  // namespace

TEST(JointLayout, ToSortedAndToPhysicalAreInverses) {
  TwoInstanceLayout data;
  JointLayout layout = data.View();

  EXPECT_EQ(layout.ToSorted({0, 0}), 0U);
  EXPECT_EQ(layout.ToSorted({0, 1}), 3U);
  EXPECT_EQ(layout.ToSorted({1, 0}), 1U);
  for (std::size_t row : {0U, 1U, 3U}) {
    EXPECT_EQ(layout.ToSorted(layout.ToPhysical(row)), row);
  }
}

TEST(JointLayout, PaddingAndSizes) {
  TwoInstanceLayout data;
  JointLayout layout = data.View();

  EXPECT_FALSE(layout.IsPadding(0));
  EXPECT_TRUE(layout.IsPadding(2));
  EXPECT_EQ(layout.PaddedSize(), 4U);
  EXPECT_EQ(layout.BaseRowIndex(), 4U);
  EXPECT_EQ(layout.InstanceCount(), 2U);
}

TEST(JointLayoutConstructorPrecondition, DiesWhenOffsetsDontEndAtRowCount) {
  std::vector<std::size_t> instance_offsets = {0, 2};
  std::vector<std::size_t> sorted_rows = {0};
  std::vector<ArchetypeJointHandle> joints = {{0, 0}};

  EXPECT_DEATH(
      JointLayout(instance_offsets, sorted_rows, joints),
      "must end at sorted_rows.size"
  );
}

TEST(JointLayoutAccessPrecondition, DiesOnJointPastItsInstance) {
  TwoInstanceLayout data;
  JointLayout layout = data.View();

  EXPECT_DEATH({ (void)layout.ToSorted({1, 1}); }, "");
}
