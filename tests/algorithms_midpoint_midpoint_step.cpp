#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <set>
#include <utility>
#include <vector>

#include "algorithms/conventions.hpp"
#include "algorithms/midpoint/midpoint_data.hpp"
#include "algorithms/midpoint/midpoint_step.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "domain/joint_layout.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/orderings.hpp"
#include "util/simd_ops.hpp"

// BeginStep, TrialStep, UpdateStep and CommitStep wire implicit midpoint's
// Ops into a real MidpointView via a real SimAllocator. What's under test is
// the Step-level wiring -- which fields each Step reads and writes, and the
// dt it hands its Op -- not the per-row math
// (algorithms_midpoint_midpoint_ops.cpp) or the full fixed-point tick with
// ABA (algorithms_integrators.cpp).
//
// As in algorithms_vi_vi_step.cpp, every archetype is a flat set of
// independent single-joint roots, and values are written one per lane GROUP
// (a batched Store writes a whole group at once). No archetype data is
// loaded, so every pose a test depends on is stored explicitly rather than
// left at the allocator's zero fill.

using namespace achilles::algorithms;
using namespace achilles::algorithms::midpoint;
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

SimAllocator<MidpointAlgorithm> MakeSim(std::size_t instance_count) {
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
  return SimAllocator<MidpointAlgorithm>(archetypes);
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

// One joint's group, with a revolute-Z subspace and the tick's start state
// (q_n, qd_n) already snapshotted into the scratch fields, plus the
// estimate qd_next.
struct Fixture {
  SimAllocator<MidpointAlgorithm> sim = MakeSim(1);
  MidpointView view = sim.ViewFor<MidpointAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);

  Fixture(float q_start, float qd_start, float qd_next) {
    view.Store<MidpointField::kJointSubspace, B>(group, RevoluteZSubspace());
    view.Store<MidpointField::kStartPosition, B>(group, PoseZ(q_start));
    view.Store<MidpointField::kStartVelocity, B>(group, Slot0(qd_start));
    view.Store<MidpointField::kNextVelocity, B>(group, Slot0(qd_next));
  }
};

}  // namespace

// BeginStep reads the live (q, qd) ABA shares and writes the scratch
// fields -- the snapshot every later Step builds from.
TEST(MidpointBeginStep, SnapshotsLiveStateIntoScratchFields) {
  SimAllocator<MidpointAlgorithm> sim = MakeSim(1);
  MidpointView view = sim.ViewFor<MidpointAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);
  view.Store<MidpointField::kJointPosition, B>(group, PoseZ(0.3F));
  view.Store<MidpointField::kJointVelocity, B>(group, Slot0(1.5F));

  BeginStep::Step(sim.SimContext(), SimConfig{}, 0.5F);

  EXPECT_TRUE(BatchTrue(view.Load<MidpointField::kStartPosition, B>(group)
                            .Rotation()
                            .IsApprox(RotationZ(0.3F))));
  EXPECT_TRUE(BatchApprox(
      view.Load<MidpointField::kStartVelocity, B>(group), Slot0(1.5F)
  ));
  EXPECT_TRUE(BatchApprox(
      view.Load<MidpointField::kNextVelocity, B>(group), Slot0(1.5F)
  ));
}

// TrialStep writes the trial into the live fields ABA reads, over half dt.
TEST(MidpointTrialStep, WritesTheTrialWhereAbaReadsIt) {
  Fixture f(0.1F, 1.0F, 3.0F);

  TrialStep::Step(f.sim.SimContext(), SimConfig{}, 0.5F);

  EXPECT_TRUE(BatchApprox(
      f.view.Load<MidpointField::kJointVelocity, B>(f.group), Slot0(2.0F)
  ));
  EXPECT_TRUE(BatchTrue(f.view.Load<MidpointField::kJointPosition, B>(f.group)
                            .Rotation()
                            .IsApprox(RotationZ(0.6F))));
}

// UpdateStep folds the live qdd (ABA's output) into the estimate.
TEST(MidpointUpdateStep, FoldsLiveAccelerationIntoTheEstimate) {
  Fixture f(0.0F, 1.0F, 100.0F);
  f.view.Store<MidpointField::kJointAcceleration, B>(
      f.group, Slot0Acceleration(4.0F)
  );

  UpdateStep::Step(f.sim.SimContext(), SimConfig{}, 0.5F);

  EXPECT_TRUE(BatchApprox(
      f.view.Load<MidpointField::kNextVelocity, B>(f.group), Slot0(3.0F)
  ));
}

// CommitStep writes the committed state into the live fields, over full dt.
TEST(MidpointCommitStep, CommitsIntoTheLiveFields) {
  Fixture f(0.1F, 1.0F, 3.0F);

  CommitStep::Step(f.sim.SimContext(), SimConfig{}, 0.5F);

  EXPECT_TRUE(BatchApprox(
      f.view.Load<MidpointField::kJointVelocity, B>(f.group), Slot0(3.0F)
  ));
  EXPECT_TRUE(BatchTrue(f.view.Load<MidpointField::kJointPosition, B>(f.group)
                            .Rotation()
                            .IsApprox(RotationZ(1.1F))));
}

// More than one lane group: proves the Steps' traversal walks every group
// view.Size() covers, not just group 0.
TEST(MidpointUpdateStep, UpdatesEveryLaneGroupIndependently) {
  std::size_t instance_count = Lane() * 2;
  SimAllocator<MidpointAlgorithm> sim = MakeSim(instance_count);
  MidpointView view = sim.ViewFor<MidpointAlgorithm>();
  const auto& layout = sim.LayoutFor<TopologicalOrdering>();

  std::set<std::size_t> groups;
  for (std::size_t i = 0; i < instance_count; ++i) {
    groups.insert(BatchGroup(layout, i));
  }
  ASSERT_GT(groups.size(), 1U);

  for (std::size_t group : groups) {
    auto scale = static_cast<float>(group + 1);
    view.Store<MidpointField::kStartVelocity, B>(group, Slot0(0.0F));
    view.Store<MidpointField::kJointAcceleration, B>(
        group, Slot0Acceleration(scale)
    );
  }

  UpdateStep::Step(sim.SimContext(), SimConfig{}, 1.0F);

  for (std::size_t group : groups) {
    auto scale = static_cast<float>(group + 1);
    EXPECT_TRUE(BatchApprox(
        view.Load<MidpointField::kNextVelocity, B>(group), Slot0(scale)
    )) << "group "
       << group;
  }
}
