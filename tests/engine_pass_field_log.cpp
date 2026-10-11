#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "domain/math/vector3.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/pass/field_log.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "support/toy_field.hpp"

using achilles::domain::Archetype;
using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::domain::math::Vector3;
using achilles::domain::math::Vector3Assembler;
using achilles::engine::Algorithm;
using achilles::engine::memory::SimAllocator;
using achilles::engine::pass::FieldLog;
using achilles::engine::pass::FieldLogForT;
using achilles::engine::topology::PlanarLayout;
using achilles::test_support::ToyAlgorithm;

namespace {

// A toy algorithm with two named fields and one unnamed one (never
// offered to logging). kVelocity is SharedAs a slot so another Algorithm
// can share the name below.
enum class LogField : std::uint8_t { kPosition, kVelocity, kScratch, kCount };

template <LogField F>
struct LogFieldTraits {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
};
template <>
struct LogFieldTraits<LogField::kPosition> {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
  static constexpr std::string_view kName = "position";
};
struct VelocitySlot;
template <>
struct LogFieldTraits<LogField::kVelocity> {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
  using SharedAs = VelocitySlot;
  static constexpr std::string_view kName = "velocity";
};

using LogAlgorithm = Algorithm<LogField, LogFieldTraits>;

// Shares "velocity" with LogAlgorithm through the same SharedAs slot --
// one block of memory, so it must be logged once.
enum class ShareField : std::uint8_t { kVelocity, kCount };
template <ShareField F>
struct ShareFieldTraits {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
  using SharedAs = VelocitySlot;
  static constexpr std::string_view kName = "velocity";
};
using ShareAlgorithm = Algorithm<ShareField, ShareFieldTraits>;

// Names a field "position" with its own private storage -- a different
// block than LogAlgorithm's "position", so the two can't share a name.
enum class ClashField : std::uint8_t { kPosition, kCount };
template <ClashField F>
struct ClashFieldTraits {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
  static constexpr std::string_view kName = "position";
};
using ClashAlgorithm = Algorithm<ClashField, ClashFieldTraits>;

using achilles::engine::AllFields;
using achilles::engine::SameNameMeansSameSlot;
static_assert(SameNameMeansSameSlot(AllFields<LogAlgorithm, ToyAlgorithm>()));
static_assert(SameNameMeansSameSlot(AllFields<LogAlgorithm, ShareAlgorithm>()));
static_assert(!SameNameMeansSameSlot(AllFields<LogAlgorithm, ClashAlgorithm>())
);

// Hosts the loggable algorithm next to a non-loggable one, so selection
// has to skip the latter.
using Sim = SimAllocator<LogAlgorithm, ToyAlgorithm>;
using Log = FieldLogForT<decltype(std::declval<Sim>().SimContext())>;
static_assert(std::is_same_v<Log, FieldLog<LogAlgorithm, ToyAlgorithm>>);

// `instance_count` standalone single-joint instances of one archetype.
Archetype MakeArchetype(std::size_t instance_count) {
  constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
  return Archetype(
      "toy",
      {ArchetypeTreeStructure::kNoParent},
      std::vector<ArchetypeJointHandle>(
          instance_count, ArchetypeJointHandle{kNone, kNone}
      ),
      true,
      {},
      {"body"}
  );
}

Sim MakeSim(std::size_t instance_count) {
  std::array<Archetype, 1> archetypes = {MakeArchetype(instance_count)};
  return Sim(archetypes);
}

std::string Label(ArchetypeJointHandle handle) {
  return "i" + std::to_string(handle.instance_index) + ".j" +
         std::to_string(handle.joint_index);
}

}  // namespace

TEST(FieldLogSelection, ExactNameSelectsOneField) {
  Sim sim = MakeSim(2);
  std::vector<std::string> unknown;
  std::vector<std::string> names = {"position"};
  Log log(sim.SimContext(), names, unknown);

  EXPECT_TRUE(unknown.empty());
  EXPECT_EQ(log.Width(), 2U * 3U);  // 2 joints x 3 scalars.
}

// A name two Algorithms share (one SharedAs block) is logged once.
TEST(FieldLogSelection, SharedNameIsLoggedOnce) {
  std::array<Archetype, 1> archetypes = {MakeArchetype(2)};
  SimAllocator<LogAlgorithm, ShareAlgorithm> sim(archetypes);
  std::vector<std::string> unknown;
  std::vector<std::string> names = {"velocity"};
  FieldLog<LogAlgorithm, ShareAlgorithm> log(sim.SimContext(), names, unknown);

  EXPECT_TRUE(unknown.empty());
  EXPECT_EQ(log.Width(), 2U * 3U);  // 2 joints x 3 scalars, not doubled.
}

// "*" spans every hosted Algorithm but only picks up named fields -- here
// LogAlgorithm's position and velocity (ToyAlgorithm names none).
TEST(FieldLogSelection, WildcardSelectsEveryNamedField) {
  Sim sim = MakeSim(1);
  std::vector<std::string> unknown;
  std::vector<std::string> names = {"*"};
  Log log(sim.SimContext(), names, unknown);

  EXPECT_TRUE(unknown.empty());
  EXPECT_EQ(log.Width(), 2U * 3U);
}

TEST(FieldLogSelection, ReportsUnclaimedNames) {
  Sim sim = MakeSim(1);
  std::vector<std::string> unknown;
  std::vector<std::string> names = {"position", "nope", "toy.position", ""};
  Log log(sim.SimContext(), names, unknown);

  EXPECT_EQ(unknown, (std::vector<std::string>{"nope", "toy.position", ""}));
  EXPECT_EQ(log.Width(), 3U);
}

TEST(FieldLogSelection, NothingSelectedIsEmpty) {
  Sim sim = MakeSim(1);
  std::vector<std::string> unknown;
  Log log(sim.SimContext(), {}, unknown);

  EXPECT_TRUE(log.Empty());
}

TEST(FieldLogColumns, OrderIsJointThenScalar) {
  Sim sim = MakeSim(2);
  std::vector<std::string> unknown;
  std::vector<std::string> names = {"position"};
  Log log(sim.SimContext(), names, unknown);

  EXPECT_EQ(
      log.ColumnNames(Label),
      (std::vector<std::string>{
          "i0.j0.position.0",
          "i0.j0.position.1",
          "i0.j0.position.2",
          "i1.j0.position.0",
          "i1.j0.position.1",
          "i1.j0.position.2",
      })
  );
}

TEST(FieldLogGather, CopiesStoredValuesInColumnOrder) {
  Sim sim = MakeSim(2);
  auto context = sim.SimContext();
  auto view = context.ViewFor<LogAlgorithm>();
  const auto& layout =
      context.LayoutFor<achilles::engine::topology::LinearOrdering>();
  view.Store<LogField::kPosition, float>(
      layout.ToSorted(0, 0), Vector3<float>(1.0F, 2.0F, 3.0F)
  );
  view.Store<LogField::kPosition, float>(
      layout.ToSorted(0, 1), Vector3<float>(4.0F, 5.0F, 6.0F)
  );

  std::vector<std::string> unknown;
  std::vector<std::string> names = {"position"};
  Log log(context, names, unknown);
  std::vector<double> row(log.Width());
  log.Gather(context, std::span<double>(row));

  EXPECT_EQ(row, (std::vector<double>{1, 2, 3, 4, 5, 6}));
}
