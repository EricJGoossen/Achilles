#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <ratio>
#include <set>
#include <utility>
#include <vector>

#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/verlet/verlet_data.hpp"
#include "algorithms/verlet/verlet_step.hpp"
#include "domain/archetype.hpp"
#include "domain/joint_layout.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/orderings.hpp"
#include "util/simd_ops.hpp"

// KickStep<DtFraction> and DriftStep wire KickVelocityOp/DriftPositionOp
// into a real VerletView via a real SimAllocator, the same way
// algorithms_vi_vi_step.cpp exercises VIStep. What's under test here is the
// Step-level wiring -- which field each Step reads and writes, and which
// dt it hands its Op (DtFraction * dt for a kick, the full dt for the
// drift) -- not the per-row math, which algorithms_verlet_verlet_ops.cpp
// covers. The full Verlet tick (these Steps interleaved with ABA) is
// covered by algorithms_integrators.cpp.
//
// As in algorithms_vi_vi_step.cpp, every archetype is a flat set of
// independent single-joint roots, and values are written one per lane
// GROUP (a batched Store writes a whole group at once).

using namespace achilles::algorithms;
using namespace achilles::algorithms::verlet;
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

SimAllocator<VerletAlgorithm> MakeSim(std::size_t instance_count) {
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
  return SimAllocator<VerletAlgorithm>(archetypes);
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

}  // namespace

// KickStep<1/2> must kick by half of the dt it's given -- the fraction is
// the whole point of the template argument (Verlet kicks twice per tick).
TEST(VerletKickStep, KicksByItsDtFractionOfTheTickDt) {
  SimAllocator<VerletAlgorithm> sim = MakeSim(1);
  VerletView view = sim.ViewFor<VerletAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);
  view.Store<VerletField::kJointAcceleration, B>(
      group, Slot0Acceleration(4.0F)
  );
  view.Store<VerletField::kJointVelocity, B>(group, Slot0(1.0F));

  KickStep<std::ratio<1, 2>>::Step(sim.SimContext(), SimConfig{}, 0.5F);

  EXPECT_TRUE(
      BatchApprox(view.Load<VerletField::kJointVelocity, B>(group), Slot0(2.0F))
  );
}

// A different fraction gives a different kick -- rules out a Step that
// ignores DtFraction and happens to hard-code one half.
TEST(VerletKickStep, AFullFractionKicksByTheWholeDt) {
  SimAllocator<VerletAlgorithm> sim = MakeSim(1);
  VerletView view = sim.ViewFor<VerletAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);
  view.Store<VerletField::kJointAcceleration, B>(
      group, Slot0Acceleration(4.0F)
  );
  view.Store<VerletField::kJointVelocity, B>(group, Slot0(1.0F));

  KickStep<std::ratio<1>>::Step(sim.SimContext(), SimConfig{}, 0.5F);

  EXPECT_TRUE(
      BatchApprox(view.Load<VerletField::kJointVelocity, B>(group), Slot0(3.0F))
  );
}

// DriftStep moves q by the full dt at the current qd, and leaves qd alone.
TEST(VerletDriftStep, DriftsPositionByTheFullDtAndLeavesVelocity) {
  SimAllocator<VerletAlgorithm> sim = MakeSim(1);
  VerletView view = sim.ViewFor<VerletAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);
  view.Store<VerletField::kJointSubspace, B>(group, RevoluteZSubspace());
  view.Store<VerletField::kJointVelocity, B>(group, Slot0(0.8F));
  // No archetype data populates kJointPosition here, so start it from an
  // explicit identity pose rather than the allocator's zero fill.
  view.Store<VerletField::kJointPosition, B>(group, Transform::Identity());

  DriftStep::Step(sim.SimContext(), SimConfig{}, 0.5F);

  EXPECT_TRUE(BatchTrue(view.Load<VerletField::kJointPosition, B>(group)
                            .Rotation()
                            .IsApprox(RotationZ(0.4F))));
  EXPECT_TRUE(
      BatchApprox(view.Load<VerletField::kJointVelocity, B>(group), Slot0(0.8F))
  );
}

// More than one lane group: proves the Step's traversal walks every group
// view.Size() covers, not just group 0 (see the matching
// VIStep.MultipleJointsAcrossLaneGroupsIntegrateIndependently).
TEST(VerletKickStep, KicksEveryLaneGroupIndependently) {
  std::size_t instance_count = Lane() * 2;
  SimAllocator<VerletAlgorithm> sim = MakeSim(instance_count);
  VerletView view = sim.ViewFor<VerletAlgorithm>();
  const auto& layout = sim.LayoutFor<TopologicalOrdering>();

  std::set<std::size_t> groups;
  for (std::size_t i = 0; i < instance_count; ++i) {
    groups.insert(BatchGroup(layout, i));
  }
  ASSERT_GT(groups.size(), 1U);

  for (std::size_t group : groups) {
    auto scale = static_cast<float>(group + 1);
    view.Store<VerletField::kJointAcceleration, B>(
        group, Slot0Acceleration(scale)
    );
    view.Store<VerletField::kJointVelocity, B>(group, Slot0(0.0F));
  }

  KickStep<std::ratio<1, 2>>::Step(sim.SimContext(), SimConfig{}, 1.0F);

  for (std::size_t group : groups) {
    auto scale = static_cast<float>(group + 1);
    EXPECT_TRUE(BatchApprox(
        view.Load<VerletField::kJointVelocity, B>(group), Slot0(scale * 0.5F)
    )) << "group "
       << group;
  }
}
