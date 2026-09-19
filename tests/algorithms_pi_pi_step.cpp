#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <set>
#include <utility>
#include <vector>

#include "algorithms/conventions.hpp"
#include "algorithms/pi/pi_data.hpp"
#include "algorithms/pi/pi_step.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/layout.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "util/simd_ops.hpp"

// PIStep::Step wires IntegratePositionOp into a real PIView via a real
// SimAllocator, the same way algorithms_vi_vi_step.cpp exercises VIStep --
// see that file's own header comment for why every archetype below is a
// flat set of independent, always-root, single-joint instances (PI, like
// VI, names no JointTopology of its own; TopologicalOrdering is only
// there so a sim hosting PI/VI/ABA together needs just one JointTopology).

using namespace achilles::algorithms;
using namespace achilles::algorithms::pi;
using achilles::domain::Archetype;
using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::engine::memory::SimAllocator;
using achilles::engine::topology::TopologicalOrdering;

namespace {

using B = MathematicalT;

std::size_t Lane() { return xsimd::batch<float>::size; }

::testing::AssertionResult BatchTrue(const auto& mask) {
  if (achilles::util::AllTrue(mask)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "not all lanes true";
}

Matrix6x6 RevoluteZSubspace() {
  Matrix6x6 s = Matrix6x6::Zero();
  s(2, 0) = B(1.0F);
  return s;
}

SimAllocator<PIAlgorithm> MakeSim(std::size_t instance_count) {
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
  return SimAllocator<PIAlgorithm>(archetypes);
}

std::size_t BatchGroup(
    const achilles::engine::topology::Layout& layout, std::size_t joint
) {
  return layout.ToSorted(0, joint) / Lane();
}

void PopulateJoint(
    PIView& view,
    std::size_t group,
    const Matrix6x6& s,
    const Velocity& qd,
    const Transform& x_joint = Transform::Identity()
) {
  view.Store<PIField::kJointSubspace, B>(group, s);
  view.Store<PIField::kJointVelocity, B>(group, qd);
  view.Store<PIField::kJointPosition, B>(group, x_joint);
}

}  // namespace

// A single real joint, starting at Identity: after one Step, its pose
// must equal Exp(S*qd*dt) composed onto Identity -- recomputed
// independently via Transform::Exp/operator* (not by re-typing
// IntegratePositionOp's own expression) -- proving the full Step (real
// PIView, real traversal, real OpInvoker) actually carries the velocity
// field into the position field, not just that IntegratePositionOp::
// operator() does so in isolation (algorithms_pi_pi_ops.cpp).
TEST(PIStep, SingleJointIntegratesConstantVelocity) {
  SimAllocator<PIAlgorithm> sim = MakeSim(1);
  PIView view = sim.ViewFor<PIAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);

  Matrix6x6 s = RevoluteZSubspace();
  Vector6 qd_coords(B(0.4F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  Velocity qd(qd_coords);
  PopulateJoint(view, group, s, qd);

  SimConfig sim_config;
  PIStep::Step(sim.SimContext(), sim_config, 0.5F);

  Velocity qd_spatial(s * qd_coords);
  Transform expected =
      Transform::Identity() * Transform::Exp(qd_spatial * B(0.5F));

  Transform actual = view.Load<PIField::kJointPosition, B>(group);
  EXPECT_TRUE(BatchTrue(actual.Translation().IsApprox(expected.Translation()))
  );
  EXPECT_TRUE(BatchTrue(actual.Rotation().IsApprox(expected.Rotation())));
}

// Zero velocity must leave a nonzero pre-existing pose untouched -- the
// Step-level counterpart to IntegratePositionOpTest.
// ZeroVelocityLeavesPositionUnchanged, now through the real traversal and
// View rather than a direct operator() call.
TEST(PIStep, ZeroVelocityLeavesPositionAtInitialValue) {
  SimAllocator<PIAlgorithm> sim = MakeSim(1);
  PIView view = sim.ViewFor<PIAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);

  Transform x_initial(
      Vector3(B(1.0F), B(2.0F), B(3.0F)), Quaternion::Identity()
  );
  PopulateJoint(view, group, RevoluteZSubspace(), Velocity::Zero(), x_initial);

  SimConfig sim_config;
  PIStep::Step(sim.SimContext(), sim_config, 1.0F);

  Transform actual = view.Load<PIField::kJointPosition, B>(group);
  EXPECT_TRUE(BatchTrue(actual.Translation().IsApprox(x_initial.Translation()))
  );
  EXPECT_TRUE(BatchTrue(actual.Rotation().IsApprox(x_initial.Rotation())));
}

// IntegratePositionOp has no Initialize, so nothing resets kJointPosition
// between Step() calls -- repeated Step calls with the same constant
// velocity must keep composing onto the previous result (accumulating
// rotation about the same fixed axis, which stays exact -- a revolute
// joint's own one-parameter subgroup -- rather than resetting each time).
TEST(PIStep, RepeatedStepsComposeOntoPreviousPosition) {
  SimAllocator<PIAlgorithm> sim = MakeSim(1);
  PIView view = sim.ViewFor<PIAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);

  Matrix6x6 s = RevoluteZSubspace();
  Vector6 qd_coords(B(0.2F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  Velocity qd(qd_coords);
  PopulateJoint(view, group, s, qd);

  SimConfig sim_config;
  constexpr int kSteps = 3;
  constexpr float kDt = 0.1F;
  for (int i = 0; i < kSteps; ++i) {
    PIStep::Step(sim.SimContext(), sim_config, kDt);
  }

  Velocity qd_spatial(s * qd_coords);
  Transform expected = Transform::Exp(qd_spatial * B(kDt * 3.0F));

  Transform actual = view.Load<PIField::kJointPosition, B>(group);
  EXPECT_TRUE(BatchTrue(actual.Translation().IsApprox(expected.Translation()))
  );
  EXPECT_TRUE(BatchTrue(actual.Rotation().IsApprox(expected.Rotation())));
}

// More than one lane group's worth of real joints: proves the
// LinearTraversal driving IntegratePositionOp actually walks every group
// view.Size()/Stride covers, not just group 0.
TEST(PIStep, MultipleJointsAcrossLaneGroupsIntegrateIndependently) {
  std::size_t lane = Lane();
  std::size_t instance_count = lane * 2;
  SimAllocator<PIAlgorithm> sim = MakeSim(instance_count);
  PIView view = sim.ViewFor<PIAlgorithm>();
  const auto& layout = sim.LayoutFor<TopologicalOrdering>();

  std::set<std::size_t> groups;
  for (std::size_t i = 0; i < instance_count; ++i) {
    groups.insert(BatchGroup(layout, i));
  }
  ASSERT_GT(groups.size(), 1U)
      << "Test premise violated: fewer than two lane groups were "
         "populated, so this test can't distinguish per-group processing "
         "from a traversal that only ever touches group 0.";

  Matrix6x6 s = RevoluteZSubspace();
  for (std::size_t group : groups) {
    float scale = static_cast<float>(group + 1);
    Vector6 qd_coords(B(scale * 0.1F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
    PopulateJoint(view, group, s, Velocity(qd_coords));
  }

  SimConfig sim_config;
  PIStep::Step(sim.SimContext(), sim_config, 1.0F);

  for (std::size_t group : groups) {
    float scale = static_cast<float>(group + 1);
    Vector6 qd_coords(B(scale * 0.1F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
    Velocity qd_spatial(s * qd_coords);
    Transform expected = Transform::Exp(qd_spatial * B(1.0F));

    Transform actual = view.Load<PIField::kJointPosition, B>(group);
    EXPECT_TRUE(
        BatchTrue(actual.Translation().IsApprox(expected.Translation()))
    );
    EXPECT_TRUE(BatchTrue(actual.Rotation().IsApprox(expected.Rotation())));
  }
}
