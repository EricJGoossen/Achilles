#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <ratio>
#include <set>
#include <utility>
#include <vector>

#include "algorithms/conventions.hpp"
#include "algorithms/rk4/rk4_data.hpp"
#include "algorithms/rk4/rk4_step.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "domain/joint_layout.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "util/simd_ops.hpp"

// BeginStep, AccumulateStep<Weight>, AdvanceStep<DtFraction> and CommitStep
// wire RK4's Ops into a real RK4View via a real SimAllocator. What's under
// test is the Step-level wiring -- which fields each Step reads and writes,
// and the weight/dt each hands its Op (the template arguments are the
// whole point of AccumulateStep/AdvanceStep) -- not the per-row math
// (algorithms_rk4_rk4_ops.cpp) or the full four-stage tick with ABA
// (algorithms_integrators.cpp).
//
// As in algorithms_vi_vi_step.cpp, every archetype is a flat set of
// independent single-joint roots, and values are written one per lane GROUP
// (a batched Store writes a whole group at once). No archetype data is
// loaded, so every pose a test depends on is stored explicitly rather than
// left at the allocator's zero fill.

using namespace achilles::algorithms;
using namespace achilles::algorithms::rk4;
using achilles::domain::Archetype;
using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::engine::memory::SimAllocator;
using achilles::engine::topology::TopologicalOrdering;

namespace {

using B = MathematicalT;

std::size_t Lane() { return B::size; }

::testing::AssertionResult BatchTrue(const auto& mask) {
  if (achilles::util::AllTrue(mask)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "not all lanes true";
}

::testing::AssertionResult BatchApprox(const auto& lhs, const auto& rhs) {
  return BatchTrue(lhs.IsApprox(rhs));
}

SimAllocator<RK4Algorithm> MakeSim(std::size_t instance_count) {
  std::vector<ArchetypeJointHandle> root_parents(
      instance_count, ArchetypeJointHandle{0, 0}
  );
  std::array<Archetype, 1> archetypes = {Archetype(
      "joints",
      {ArchetypeTreeStructure::kNoParent},
      std::move(root_parents),
      true,
      {}
  )};
  return SimAllocator<RK4Algorithm>(archetypes);
}

std::size_t BatchGroup(
    const achilles::domain::JointLayout& layout, std::size_t instance
) {
  return layout.ToSorted({instance, 0}) / Lane();
}

Matrix6x6 RevoluteZSubspace() {
  Matrix6x6 s = Matrix6x6::Zero();
  s(2, 0) = B(1.0F);
  return s;
}

Velocity Slot0(float rate) {
  Velocity value(Vector6(B(rate), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F)));
  return value;
}

Acceleration Slot0Acceleration(float rate) {
  Acceleration value(
      Vector6(B(rate), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F))
  );
  return value;
}

Quaternion RotationZ(float theta) {
  return {
      B(std::cos(theta * 0.5F)), B(0.0F), B(0.0F), B(std::sin(theta * 0.5F))
  };
}

Transform PoseZ(float theta) { return {Vector3::Zero(), RotationZ(theta)}; }

// One joint's group with a revolute-Z subspace and the tick's start state
// already snapshotted into the scratch fields.
struct Fixture {
  SimAllocator<RK4Algorithm> sim = MakeSim(1);
  RK4View view = sim.ViewFor<RK4Algorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);

  Fixture(float q_start, float qd_start) {
    view.Store<RK4Field::kJointSubspace, B>(group, RevoluteZSubspace());
    view.Store<RK4Field::kStartPosition, B>(group, PoseZ(q_start));
    view.Store<RK4Field::kStartVelocity, B>(group, Slot0(qd_start));
  }
};

}  // namespace

// BeginStep snapshots the live (q, qd) and zeroes both sums.
TEST(RK4BeginStep, SnapshotsLiveStateAndZeroesSums) {
  SimAllocator<RK4Algorithm> sim = MakeSim(1);
  RK4View view = sim.ViewFor<RK4Algorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);
  view.Store<RK4Field::kJointPosition, B>(group, PoseZ(0.3F));
  view.Store<RK4Field::kJointVelocity, B>(group, Slot0(1.5F));
  view.Store<RK4Field::kVelocitySum, B>(group, Slot0(-9.0F));
  view.Store<RK4Field::kAccelerationSum, B>(group, Slot0Acceleration(-9.0F));

  BeginStep::Step(sim.SimContext(), SimConfig{}, 0.5F);

  EXPECT_TRUE(BatchTrue(view.Load<RK4Field::kStartPosition, B>(group)
                            .Rotation()
                            .IsApprox(RotationZ(0.3F))));
  EXPECT_TRUE(
      BatchApprox(view.Load<RK4Field::kStartVelocity, B>(group), Slot0(1.5F))
  );
  EXPECT_TRUE(BatchTrue(view.Load<RK4Field::kVelocitySum, B>(group).IsZero()));
  EXPECT_TRUE(BatchTrue(view.Load<RK4Field::kAccelerationSum, B>(group).IsZero()
  ));
}

namespace {

// Runs AccumulateStep<Weight> once over a live stage (qd, qdd) = (3, 3)
// with both sums starting at zero, and returns the resulting sums' slot 0.
template <typename Weight>
std::pair<Velocity, Acceleration> AccumulateOnceFromZero() {
  Fixture f(0.0F, 0.0F);
  f.view.Store<RK4Field::kJointVelocity, B>(f.group, Slot0(3.0F));
  f.view.Store<RK4Field::kJointAcceleration, B>(
      f.group, Slot0Acceleration(3.0F)
  );
  f.view.Store<RK4Field::kVelocitySum, B>(f.group, Velocity::Zero());
  f.view.Store<RK4Field::kAccelerationSum, B>(f.group, Acceleration::Zero());

  AccumulateStep<Weight>::Step(f.sim.SimContext(), SimConfig{}, 0.5F);

  return {
      f.view.Load<RK4Field::kVelocitySum, B>(f.group),
      f.view.Load<RK4Field::kAccelerationSum, B>(f.group)
  };
}

}  // namespace

// AccumulateStep<Weight> applies its own weight: weight 1 and weight 2 give
// different sums, so a Step that ignored Weight couldn't pass both.
TEST(RK4AccumulateStep, AccumulatesTheLiveStageWithWeightOne) {
  auto [qd_sum, qdd_sum] = AccumulateOnceFromZero<std::ratio<1>>();
  EXPECT_TRUE(BatchApprox(qd_sum, Slot0(3.0F)));
  EXPECT_TRUE(BatchApprox(qdd_sum, Slot0Acceleration(3.0F)));
}

TEST(RK4AccumulateStep, AccumulatesTheLiveStageWithWeightTwo) {
  auto [qd_sum, qdd_sum] = AccumulateOnceFromZero<std::ratio<2>>();
  EXPECT_TRUE(BatchApprox(qd_sum, Slot0(6.0F)));
  EXPECT_TRUE(BatchApprox(qdd_sum, Slot0Acceleration(6.0F)));
}

// AdvanceStep<DtFraction> advances from the tick's start over
// DtFraction * dt, reading the live qd as k_v and the live qdd as k_a.
TEST(RK4AdvanceStep, AdvancesFromTickStartOverItsDtFraction) {
  Fixture f(0.1F, 1.0F);
  f.view.Store<RK4Field::kJointVelocity, B>(f.group, Slot0(2.0F));
  f.view.Store<RK4Field::kJointAcceleration, B>(
      f.group, Slot0Acceleration(4.0F)
  );

  // h = 1/2 * 1.0: q = 0.1 + 2 * 0.5, qd = 1 + 4 * 0.5.
  AdvanceStep<std::ratio<1, 2>>::Step(f.sim.SimContext(), SimConfig{}, 1.0F);

  EXPECT_TRUE(BatchTrue(f.view.Load<RK4Field::kJointPosition, B>(f.group)
                            .Rotation()
                            .IsApprox(RotationZ(1.1F))));
  EXPECT_TRUE(BatchApprox(
      f.view.Load<RK4Field::kJointVelocity, B>(f.group), Slot0(3.0F)
  ));
}

// CommitStep blends the sums into the live fields over the full dt.
TEST(RK4CommitStep, CommitsTheBlendIntoTheLiveFields) {
  Fixture f(0.1F, 1.0F);
  f.view.Store<RK4Field::kVelocitySum, B>(f.group, Slot0(12.0F));
  f.view.Store<RK4Field::kAccelerationSum, B>(
      f.group, Slot0Acceleration(24.0F)
  );

  CommitStep::Step(f.sim.SimContext(), SimConfig{}, 0.5F);

  EXPECT_TRUE(BatchTrue(f.view.Load<RK4Field::kJointPosition, B>(f.group)
                            .Rotation()
                            .IsApprox(RotationZ(1.1F))));
  EXPECT_TRUE(BatchApprox(
      f.view.Load<RK4Field::kJointVelocity, B>(f.group), Slot0(3.0F)
  ));
}

// More than one lane group: proves the Steps' traversal walks every group
// view.Size() covers, not just group 0.
TEST(RK4AccumulateStep, AccumulatesEveryLaneGroupIndependently) {
  std::size_t instance_count = Lane() * 2;
  SimAllocator<RK4Algorithm> sim = MakeSim(instance_count);
  RK4View view = sim.ViewFor<RK4Algorithm>();
  const auto& layout = sim.LayoutFor<TopologicalOrdering>();

  std::set<std::size_t> groups;
  for (std::size_t i = 0; i < instance_count; ++i) {
    groups.insert(BatchGroup(layout, i));
  }
  ASSERT_GT(groups.size(), 1U);

  for (std::size_t group : groups) {
    auto scale = static_cast<float>(group + 1);
    view.Store<RK4Field::kJointVelocity, B>(group, Slot0(scale));
  }

  AccumulateStep<std::ratio<2>>::Step(sim.SimContext(), SimConfig{}, 1.0F);

  for (std::size_t group : groups) {
    auto scale = static_cast<float>(group + 1);
    EXPECT_TRUE(BatchApprox(
        view.Load<RK4Field::kVelocitySum, B>(group), Slot0(scale * 2.0F)
    )) << "group "
       << group;
  }
}
