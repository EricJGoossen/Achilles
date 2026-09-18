#include <gtest/gtest.h>

#include <cstddef>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"

using achilles::domain::Archetype;
using achilles::domain::ArchetypeField;
using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;

namespace {

// A two-joint chain (joint 1's parent is joint 0), two instances, one
// field ("transform") with 3 leaves per joint -- enough to exercise
// instance-major/joint-major/leaf-minor indexing without it being
// coincidentally symmetric (every value below is distinct).
Archetype MakeTwoJointArchetype() {
  std::vector<std::size_t> tree_structure = {
      ArchetypeTreeStructure::kNoParent, 0
  };
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{0, 0}, ArchetypeJointHandle{1, 0}
  };
  std::vector<ArchetypeField> fields;
  fields.push_back(ArchetypeField{
      "transform",
      3,
      // instance 0: joint0=[0,1,2] joint1=[3,4,5]
      // instance 1: joint0=[6,7,8] joint1=[9,10,11]
      {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}
  });
  return {
      "body",
      std::move(tree_structure),
      std::move(root_parents),
      true,
      std::move(fields)
  };
}

}  // namespace

TEST(ArchetypeAccessors, BasicShape) {
  Archetype archetype = MakeTwoJointArchetype();

  EXPECT_EQ(archetype.Name(), "body");
  EXPECT_EQ(archetype.JointCount(), 2U);
  EXPECT_EQ(archetype.InstanceCount(), 2U);
  EXPECT_TRUE(archetype.IsRootArchetype());
}

// TreeStructure() must hand back exactly what the archetype was built from
// -- SimAllocator's PickLayout borrows this straight into Ordering::Build.
TEST(ArchetypeAccessors, TreeStructureBorrowsOwnStorage) {
  Archetype archetype = MakeTwoJointArchetype();
  ArchetypeTreeStructure tree = archetype.TreeStructure();

  EXPECT_TRUE(tree.is_root_archetype);
  ASSERT_EQ(tree.tree_structure.size(), 2U);
  EXPECT_EQ(tree.tree_structure[0], ArchetypeTreeStructure::kNoParent);
  EXPECT_EQ(tree.tree_structure[1], 0U);
  ASSERT_EQ(tree.root_parents.size(), 2U);
  EXPECT_EQ(tree.root_parents[1].instance_index, 1U);
}

TEST(ArchetypeFields, FindFieldReturnsMatchByName) {
  Archetype archetype = MakeTwoJointArchetype();

  const ArchetypeField* found = archetype.FindField("transform");
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->scalars_per_leaf, 3U);
}

TEST(ArchetypeFields, FindFieldReturnsNullForUnknownName) {
  Archetype archetype = MakeTwoJointArchetype();

  EXPECT_EQ(archetype.FindField("does_not_exist"), nullptr);
}

TEST(ArchetypeFields, FieldsExposesEveryDeclaredField) {
  Archetype archetype = MakeTwoJointArchetype();

  EXPECT_EQ(archetype.Fields().size(), 1U);
}

// Instance-major, then joint-major, then leaf-minor -- SimAllocator's
// Populate pass relies on exactly this indexing to write archetype data
// into the right sorted row.
TEST(ArchetypeFieldAt, IndexesInstanceMajorJointMajorLeafMinor) {
  Archetype archetype = MakeTwoJointArchetype();
  const ArchetypeField* field = archetype.FindField("transform");
  ASSERT_NE(field, nullptr);

  EXPECT_DOUBLE_EQ(ArchetypeFieldAt(*field, 0, 0, 0, 2), 0.0);
  EXPECT_DOUBLE_EQ(ArchetypeFieldAt(*field, 0, 1, 2, 2), 5.0);
  EXPECT_DOUBLE_EQ(ArchetypeFieldAt(*field, 1, 0, 0, 2), 6.0);
  EXPECT_DOUBLE_EQ(ArchetypeFieldAt(*field, 1, 1, 2, 2), 11.0);
}

// Proves the assert in ArchetypeFieldAt actually fires for a genuine
// out-of-range access, not just that the formula happens to look right.
TEST(ArchetypeFieldAt, DiesOnOutOfRangeIndex) {
  Archetype archetype = MakeTwoJointArchetype();
  const ArchetypeField* field = archetype.FindField("transform");
  ASSERT_NE(field, nullptr);

  EXPECT_DEATH(ArchetypeFieldAt(*field, 5, 5, 5, 2), "out of range");
}
