#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <string_view>
#include <vector>

#include "domain/archetype.hpp"
#include "interface/archetype_loader.hpp"
#include "support/temp_dir.hpp"

using achilles::domain::Archetype;
using achilles::domain::ArchetypeTreeStructure;
using achilles::interface::ArchetypeLoadError;
using achilles::interface::LoadArchetypes;
using achilles::test_support::TempDir;

namespace {

const Archetype& FindByName(
    const std::vector<Archetype>& archetypes, std::string_view name
) {
  for (const Archetype& archetype : archetypes) {
    if (archetype.Name() == name) {
      return archetype;
    }
  }
  ADD_FAILURE() << "no archetype named " << name;
  return archetypes.front();
}

}  // namespace

TEST(LoadArchetypes, SingleRootArchetypeWithFlatFieldValues) {
  TempDir dir;
  auto file = dir.Write("root.arow", R"(
archetype: base
joints:
  - name: mount
    fields:
      subspace: [0, 0, 1, 0, 0, 0]
)");

  std::vector<Archetype> archetypes = LoadArchetypes(file);

  ASSERT_EQ(archetypes.size(), 1U);
  const Archetype& base = archetypes.front();
  EXPECT_EQ(base.Name(), "base");
  EXPECT_TRUE(base.IsRootArchetype());
  EXPECT_EQ(base.JointCount(), 1U);
  EXPECT_EQ(base.InstanceCount(), 1U);

  const auto* subspace = base.FindField("subspace");
  ASSERT_NE(subspace, nullptr);
  EXPECT_EQ(subspace->scalars_per_leaf, 6U);
  EXPECT_DOUBLE_EQ(ArchetypeFieldAt(*subspace, 0, 0, 2, 1), 1.0);
}

TEST(LoadArchetypes, TwoJointLocalTreeResolvesParentByName) {
  TempDir dir;
  auto file = dir.Write("root.arow", R"(
archetype: arm
joints:
  - name: shoulder
    fields: { x: [1] }
  - name: elbow
    parent: shoulder
    fields: { x: [2] }
)");

  std::vector<Archetype> archetypes = LoadArchetypes(file);

  ASSERT_EQ(archetypes.size(), 1U);
  auto tree = archetypes.front().TreeStructure();
  ASSERT_EQ(tree.tree_structure.size(), 2U);
  EXPECT_EQ(tree.tree_structure[0], ArchetypeTreeStructure::kNoParent);
  EXPECT_EQ(tree.tree_structure[1], 0U);
}

// A field value can be an arbitrarily nested mapping/sequence of scalars
// (here: a mapping of two sequences) -- every leaf gets appended in
// encounter order, generically, with no per-field-name knowledge.
TEST(LoadArchetypes, FlattensNestedMappingFieldValuesInEncounterOrder) {
  TempDir dir;
  auto file = dir.Write("root.arow", R"(
archetype: base
joints:
  - name: mount
    fields:
      transform: {translation: [1, 2, 3], rotation: [4, 5, 6, 7]}
)");

  std::vector<Archetype> archetypes = LoadArchetypes(file);
  const auto* transform = archetypes.front().FindField("transform");
  ASSERT_NE(transform, nullptr);
  EXPECT_EQ(transform->scalars_per_leaf, 7U);
  for (std::size_t leaf = 0; leaf < 7; ++leaf) {
    EXPECT_DOUBLE_EQ(
        ArchetypeFieldAt(*transform, 0, 0, leaf, 1),
        static_cast<double>(leaf + 1)
    );
  }
}

TEST(LoadArchetypes, ParsesBooleanFieldValuesAsOneAndZero) {
  TempDir dir;
  auto file = dir.Write("root.arow", R"(
archetype: base
joints:
  - name: mount
    fields:
      mask: [true, false, true]
)");

  std::vector<Archetype> archetypes = LoadArchetypes(file);
  const auto* mask = archetypes.front().FindField("mask");
  ASSERT_NE(mask, nullptr);
  EXPECT_DOUBLE_EQ(ArchetypeFieldAt(*mask, 0, 0, 0, 1), 1.0);
  EXPECT_DOUBLE_EQ(ArchetypeFieldAt(*mask, 0, 0, 1, 1), 0.0);
  EXPECT_DOUBLE_EQ(ArchetypeFieldAt(*mask, 0, 0, 2, 1), 1.0);
}

// The chassis (root, 1 copy) is pulled in via `includes`; every one of the
// 4 wheel copies attaches to that single chassis instance's own "axle"
// joint -- the "target has exactly 1 copy, broadcast to every child
// instance" rule.
TEST(LoadArchetypes, RecursiveIncludeAndBroadcastAttachToSingleParentInstance) {
  TempDir dir;
  dir.Write("chassis.arow", R"(
archetype: chassis
joints:
  - name: axle
    fields: { x: [0] }
)");
  auto root = dir.Write("wheel.arow", R"(
archetype: wheel
copies: 4
includes: [chassis.arow]
attach: {archetype: chassis, joint: axle}
joints:
  - name: hub
    fields: { x: [0] }
)");

  std::vector<Archetype> archetypes = LoadArchetypes(root);

  ASSERT_EQ(archetypes.size(), 2U);
  // Root (chassis) must come first -- InstanceOffsets/DecodeInstance
  // (ordering_policy.cpp) number global instances in `archetypes` order.
  EXPECT_EQ(archetypes[0].Name(), "chassis");
  EXPECT_EQ(archetypes[1].Name(), "wheel");

  const Archetype& wheel = FindByName(archetypes, "wheel");
  EXPECT_EQ(wheel.InstanceCount(), 4U);
  auto tree = wheel.TreeStructure();
  ASSERT_EQ(tree.root_parents.size(), 4U);
  for (const auto& parent : tree.root_parents) {
    EXPECT_EQ(parent.instance_index, 0U);  // chassis's one instance
    EXPECT_EQ(parent.joint_index, 0U);     // "axle", chassis's only joint
  }
}

// Both segment and wheel (neither the root) declare 3 copies -- attach
// must pair instance i of the child with instance i of the parent, not
// broadcast. segment's own root_parents (attaching to the 1-copy root)
// exercise the broadcast rule at the same time.
TEST(LoadArchetypes, OneToOneAttachWhenCopiesMatch) {
  TempDir dir;
  dir.Write("world.arow", R"(
archetype: world
joints:
  - name: origin
    fields: { x: [0] }
)");
  dir.Write("segment.arow", R"(
archetype: segment
copies: 3
includes: [world.arow]
attach: {archetype: world, joint: origin}
joints:
  - name: axle
    fields: { x: [0] }
)");
  auto root = dir.Write("wheel.arow", R"(
archetype: wheel
copies: 3
includes: [segment.arow]
attach: {archetype: segment, joint: axle}
joints:
  - name: hub
    fields: { x: [0] }
)");

  std::vector<Archetype> archetypes = LoadArchetypes(root);
  const Archetype& segment = FindByName(archetypes, "segment");
  std::size_t segment_offset = 1;  // world's single instance comes first
  auto segment_tree = segment.TreeStructure();
  ASSERT_EQ(segment_tree.root_parents.size(), 3U);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(
        segment_tree.root_parents[i].instance_index, 0U
    );  // world's one instance
  }

  const Archetype& wheel = FindByName(archetypes, "wheel");
  auto wheel_tree = wheel.TreeStructure();
  ASSERT_EQ(wheel_tree.root_parents.size(), 3U);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(wheel_tree.root_parents[i].instance_index, segment_offset + i);
  }
}

TEST(LoadArchetypes, ThrowsWhenArchetypeKeyIsMissing) {
  TempDir dir;
  auto file = dir.Write("root.arow", R"(
joints:
  - name: mount
    fields: { x: [0] }
)");

  EXPECT_THROW(LoadArchetypes(file), ArchetypeLoadError);
}

TEST(LoadArchetypes, ThrowsWhenNoArchetypeIsRoot) {
  TempDir dir;
  dir.Write("a.arow", R"(
archetype: a
includes: [b.arow]
attach: {archetype: b, joint: mount}
joints:
  - name: mount
    fields: { x: [0] }
)");
  auto root = dir.Write("b.arow", R"(
archetype: b
attach: {archetype: a, joint: mount}
joints:
  - name: mount
    fields: { x: [0] }
)");

  try {
    LoadArchetypes(root);
    FAIL() << "expected ArchetypeLoadError";
  } catch (const ArchetypeLoadError& e) {
    EXPECT_NE(std::string(e.what()).find("root"), std::string::npos);
  }
}

TEST(LoadArchetypes, ThrowsWhenMoreThanOneArchetypeIsRoot) {
  TempDir dir;
  dir.Write("a.arow", R"(
archetype: a
joints:
  - name: mount
    fields: { x: [0] }
)");
  auto root = dir.Write("b.arow", R"(
archetype: b
includes: [a.arow]
joints:
  - name: mount
    fields: { x: [0] }
)");

  EXPECT_THROW(LoadArchetypes(root), ArchetypeLoadError);
}

TEST(LoadArchetypes, ThrowsWhenAttachTargetWasNeverDeclared) {
  TempDir dir;
  auto file = dir.Write("root.arow", R"(
archetype: wheel
attach: {archetype: chassis, joint: axle}
joints:
  - name: hub
    fields: { x: [0] }
)");

  EXPECT_THROW(LoadArchetypes(file), ArchetypeLoadError);
}

TEST(LoadArchetypes, ThrowsWhenAttachTargetJointNameIsUnknown) {
  TempDir dir;
  dir.Write("chassis.arow", R"(
archetype: chassis
joints:
  - name: axle
    fields: { x: [0] }
)");
  auto root = dir.Write("wheel.arow", R"(
archetype: wheel
includes: [chassis.arow]
attach: {archetype: chassis, joint: does_not_exist}
joints:
  - name: hub
    fields: { x: [0] }
)");

  EXPECT_THROW(LoadArchetypes(root), ArchetypeLoadError);
}

TEST(LoadArchetypes, ThrowsWhenCopiesNeitherMatchNorTargetIsSingle) {
  TempDir dir;
  dir.Write("world.arow", R"(
archetype: world
joints:
  - name: origin
    fields: { x: [0] }
)");
  dir.Write("chassis.arow", R"(
archetype: chassis
copies: 2
includes: [world.arow]
attach: {archetype: world, joint: origin}
joints:
  - name: axle
    fields: { x: [0] }
)");
  auto root = dir.Write("wheel.arow", R"(
archetype: wheel
copies: 3
includes: [chassis.arow]
attach: {archetype: chassis, joint: axle}
joints:
  - name: hub
    fields: { x: [0] }
)");

  try {
    LoadArchetypes(root);
    FAIL() << "expected ArchetypeLoadError";
  } catch (const ArchetypeLoadError& e) {
    EXPECT_NE(std::string(e.what()).find("copies"), std::string::npos);
  }
}

TEST(LoadArchetypes, ThrowsWhenRootArchetypeHasMoreThanOneCopy) {
  TempDir dir;
  auto file = dir.Write("root.arow", R"(
archetype: base
copies: 2
joints:
  - name: mount
    fields: { x: [0] }
)");

  EXPECT_THROW(LoadArchetypes(file), ArchetypeLoadError);
}

TEST(LoadArchetypes, ThrowsWhenJointParentNameIsUnknown) {
  TempDir dir;
  auto file = dir.Write("root.arow", R"(
archetype: base
joints:
  - name: mount
    parent: does_not_exist
    fields: { x: [0] }
)");

  EXPECT_THROW(LoadArchetypes(file), ArchetypeLoadError);
}

TEST(LoadArchetypes, ThrowsWhenNoJointIsLocalRoot) {
  TempDir dir;
  auto file = dir.Write("root.arow", R"(
archetype: base
joints:
  - name: a
    parent: b
    fields: { x: [0] }
  - name: b
    parent: a
    fields: { x: [0] }
)");

  EXPECT_THROW(LoadArchetypes(file), ArchetypeLoadError);
}

TEST(LoadArchetypes, ThrowsWhenFieldIsPartiallyDeclaredAcrossJoints) {
  TempDir dir;
  auto file = dir.Write("root.arow", R"(
archetype: base
joints:
  - name: a
    fields: { x: [0] }
  - name: b
    parent: a
    fields: {}
)");

  EXPECT_THROW(LoadArchetypes(file), ArchetypeLoadError);
}

TEST(LoadArchetypes, ThrowsOnIncludeCycle) {
  TempDir dir;
  dir.Write("a.arow", R"(
archetype: a
includes: [b.arow]
joints:
  - name: mount
    fields: { x: [0] }
)");
  auto root = dir.Write("b.arow", R"(
archetype: b
includes: [a.arow]
attach: {archetype: a, joint: mount}
joints:
  - name: mount
    fields: { x: [0] }
)");

  EXPECT_THROW(LoadArchetypes(root), ArchetypeLoadError);
}

TEST(LoadArchetypes, ThrowsOnMissingFile) {
  EXPECT_THROW(
      LoadArchetypes(std::filesystem::path("/nonexistent/path/root.arow")),
      ArchetypeLoadError
  );
}
