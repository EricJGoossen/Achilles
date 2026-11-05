#include <gtest/gtest.h>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "domain/archetype/archetype.hpp"
#include "domain/archetype/archetype_set.hpp"

using achilles::domain::archetype::Archetype;
using achilles::domain::archetype::ArchetypeSet;
using achilles::domain::JointField;
using achilles::domain::JointRef;

namespace {

// A layout whose rows spell out the instance and joint they were asked for.
struct RowNumbers {
  std::size_t ToSorted(std::size_t instance, std::size_t joint) const {
    return (instance * 100) + joint;
  }
};

// A two-joint chain (joint 1's parent is joint 0), two instances, one
// field ("transform") with 3 leaves per joint -- enough to exercise
// instance-major/joint-major/leaf-minor indexing without it being
// coincidentally symmetric (every value below is distinct).
Archetype MakeTwoJointArchetype() {
  std::vector<std::size_t> tree_structure = {achilles::domain::kNoParent, 0};
  std::vector<JointRef> root_parents = {JointRef{0, 0}, JointRef{1, 0}};
  std::vector<JointField> fields;
  fields.push_back(JointField{
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
      {achilles::domain::kNoParent, 0},
      std::vector<JointRef>(copies),
      copies == 1,
      {},
      {"a", "b"}
  };
}

}  // namespace

TEST(ArchetypeAccessors, BasicShape) {
  Archetype archetype = MakeTwoJointArchetype();

  EXPECT_EQ(archetype.name, "body");
  EXPECT_EQ(archetype.tree_structure.size(), 2U);
  EXPECT_EQ(archetype.root_parents.size(), 2U);
  EXPECT_TRUE(archetype.is_root_archetype);
}

// The archetype must hold exactly what the archetype was built from
// -- SimAllocator's PickLayout borrows this straight into Ordering::Build.
TEST(ArchetypeAccessors, TreeStructureBorrowsOwnStorage) {
  Archetype archetype = MakeTwoJointArchetype();
  const Archetype& tree = archetype;

  EXPECT_TRUE(tree.is_root_archetype);
  ASSERT_EQ(tree.tree_structure.size(), 2U);
  EXPECT_EQ(tree.tree_structure[0], achilles::domain::kNoParent);
  EXPECT_EQ(tree.tree_structure[1], 0U);
  ASSERT_EQ(tree.root_parents.size(), 2U);
  EXPECT_EQ(tree.root_parents[1].instance_index, 1U);
}

TEST(ArchetypeFields, FindFieldReturnsMatchByName) {
  Archetype archetype = MakeTwoJointArchetype();

  const JointField* found = ArchetypeSet::FindField(archetype, "transform");
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->scalars_per_leaf, 3U);
}

TEST(ArchetypeFields, FindFieldReturnsNullForUnknownName) {
  Archetype archetype = MakeTwoJointArchetype();

  EXPECT_EQ(ArchetypeSet::FindField(archetype, "does_not_exist"), nullptr);
}

TEST(ArchetypeFields, FieldsExposesEveryDeclaredField) {
  Archetype archetype = MakeTwoJointArchetype();

  EXPECT_EQ(archetype.fields.size(), 1U);
}

// Instance-major, then joint-major, then leaf-minor -- SimAllocator's
// Populate pass relies on exactly this indexing to write archetype data
// into the right sorted row.
TEST(JointFieldAt, IndexesInstanceMajorJointMajorLeafMinor) {
  Archetype archetype = MakeTwoJointArchetype();
  const JointField* field = ArchetypeSet::FindField(archetype, "transform");
  ASSERT_NE(field, nullptr);

  EXPECT_DOUBLE_EQ(JointFieldAt(*field, 0, 0, 0, 2), 0.0);
  EXPECT_DOUBLE_EQ(JointFieldAt(*field, 0, 1, 2, 2), 5.0);
  EXPECT_DOUBLE_EQ(JointFieldAt(*field, 1, 0, 0, 2), 6.0);
  EXPECT_DOUBLE_EQ(JointFieldAt(*field, 1, 1, 2, 2), 11.0);
}

// Proves the assert in JointFieldAt actually fires for a genuine
// out-of-range access, not just that the formula happens to look right.
TEST(JointFieldAt, DiesOnOutOfRangeIndex) {
  Archetype archetype = MakeTwoJointArchetype();
  const JointField* field = ArchetypeSet::FindField(archetype, "transform");
  ASSERT_NE(field, nullptr);

  EXPECT_DEATH(JointFieldAt(*field, 5, 5, 5, 2), "out of range");
}

// JointIndex is the name -> local index lookup RowOf is built on.
TEST(ArchetypeJointNames, JointIndexFindsEachNamedJoint) {
  Archetype chain = MakeNamedChain("chain", 1);
  EXPECT_EQ(ArchetypeSet::JointIndex(chain, "a"), 0U);
  EXPECT_EQ(ArchetypeSet::JointIndex(chain, "b"), 1U);
  EXPECT_FALSE(ArchetypeSet::JointIndex(chain, "c").has_value());
}

// Joint names are optional (a hand-built Archetype, as in most tests, has
// none) -- every lookup is then a miss, not an out-of-range read.
TEST(ArchetypeJointNames, JointIndexMissesOnAnArchetypeBuiltWithoutNames) {
  Archetype unnamed = MakeTwoJointArchetype();
  EXPECT_FALSE(ArchetypeSet::JointIndex(unnamed, "a").has_value());
}

// Proves the constructor's shape check fires: a name list that doesn't line
// up with the tree would silently map names onto the wrong joints.
TEST(ArchetypeJointNames, DiesWhenNamesDoNotCoverEveryJoint) {
  EXPECT_DEATH(
      Archetype(
          "short",
          {achilles::domain::kNoParent, 0},
          {JointRef{0, 0}},
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
TEST(ArchetypeSetRowOf, NumbersInstancesEndToEndAcrossArchetypes) {
  const ArchetypeSet archetypes(std::vector<Archetype>{
      MakeNamedChain("root", 1), MakeNamedChain("leg", 3)
  });
  EXPECT_EQ(archetypes.RowOf({"root", "b", 0}, RowNumbers{}), 1U);
  EXPECT_EQ(archetypes.RowOf({"leg", "a", 2}, RowNumbers{}), 300U);
}

// User input, so each kind of miss is a catchable error naming the problem
// rather than a crash.
TEST(ArchetypeSetRowOf, ThrowsOnEachKindOfMiss) {
  const ArchetypeSet archetypes(
      std::vector<Archetype>{MakeNamedChain("chain", 2)}
  );
  EXPECT_THROW(
      archetypes.RowOf({"nope", "a", 0}, RowNumbers{}), std::invalid_argument
  );
  EXPECT_THROW(
      archetypes.RowOf({"chain", "nope", 0}, RowNumbers{}),
      std::invalid_argument
  );
  EXPECT_THROW(
      archetypes.RowOf({"chain", "a", 2}, RowNumbers{}), std::invalid_argument
  );
}
