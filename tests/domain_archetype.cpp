#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"

using achilles::domain::Archetype;
using achilles::domain::ArchetypeField;
using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::domain::JointAddress;
using achilles::domain::ResolveJoint;

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

// A named archetype of `copies` instances, each a two-joint chain named
// "a" -> "b" -- no field data, which nothing here reads.
Archetype MakeNamedChain(std::string name, std::size_t copies) {
  return {
      std::move(name),
      {ArchetypeTreeStructure::kNoParent, 0},
      std::vector<ArchetypeJointHandle>(copies),
      copies == 1,
      {},
      {"a", "b"}
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

// JointIndex is the name -> local index lookup ResolveJoint is built on.
TEST(ArchetypeJointNames, JointIndexFindsEachNamedJoint) {
  Archetype chain = MakeNamedChain("chain", 1);
  EXPECT_EQ(chain.JointIndex("a"), 0U);
  EXPECT_EQ(chain.JointIndex("b"), 1U);
  EXPECT_FALSE(chain.JointIndex("c").has_value());
}

// Joint names are optional (a hand-built Archetype, as in most tests, has
// none) -- every lookup is then a miss, not an out-of-range read.
TEST(ArchetypeJointNames, JointIndexMissesOnAnArchetypeBuiltWithoutNames) {
  Archetype unnamed = MakeTwoJointArchetype();
  EXPECT_FALSE(unnamed.JointIndex("a").has_value());
}

// Proves the constructor's shape check fires: a name list that doesn't line
// up with the tree would silently map names onto the wrong joints.
TEST(ArchetypeJointNames, DiesWhenNamesDoNotCoverEveryJoint) {
  EXPECT_DEATH(
      Archetype(
          "short",
          {ArchetypeTreeStructure::kNoParent, 0},
          {ArchetypeJointHandle{0, 0}},
          true,
          {},
          {"only_one"}
      ),
      "must name every joint"
  );
}

// Instances are numbered end to end across archetypes, in order -- the same
// global numbering JointLayout::ToSorted takes. A later archetype's
// instance 0 is therefore offset by every earlier archetype's copies.
TEST(ResolveJoint, NumbersInstancesEndToEndAcrossArchetypes) {
  std::array<Archetype, 2> archetypes = {
      MakeNamedChain("root", 1), MakeNamedChain("leg", 3)
  };
  ArchetypeJointHandle root = ResolveJoint(archetypes, {"root", "b", 0});
  EXPECT_EQ(root.instance_index, 0U);
  EXPECT_EQ(root.joint_index, 1U);

  ArchetypeJointHandle leg = ResolveJoint(archetypes, {"leg", "a", 2});
  EXPECT_EQ(leg.instance_index, 3U);
  EXPECT_EQ(leg.joint_index, 0U);
}

// User input, so each kind of miss is a catchable error naming the problem
// rather than a crash.
TEST(ResolveJoint, ThrowsOnEachKindOfMiss) {
  std::array<Archetype, 1> archetypes = {MakeNamedChain("chain", 2)};
  EXPECT_THROW(
      ResolveJoint(archetypes, {"nope", "a", 0}), std::invalid_argument
  );
  EXPECT_THROW(
      ResolveJoint(archetypes, {"chain", "nope", 0}), std::invalid_argument
  );
  EXPECT_THROW(
      ResolveJoint(archetypes, {"chain", "a", 2}), std::invalid_argument
  );
}
