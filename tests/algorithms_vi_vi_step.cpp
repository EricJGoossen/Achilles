#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <set>
#include <utility>
#include <vector>

#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/vi/vi_data.hpp"
#include "algorithms/vi/vi_step.hpp"
#include "domain/archetype.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/layout.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "util/simd_ops.hpp"

// VIStep::Step wires IntegrateVelocityOp into a real VIView via a real
// SimAllocator, the same way algorithms_aba_aba_step.cpp exercises ABAStep.
// Unlike ABA, VI names no JointTopology of its own -- ForwardBatched is a
// LinearTraversal driven by view.Size() (see vi_step.hpp's header comment),
// so there's no parent/child relationship for these tests to set up. Every
// archetype below is therefore a flat set of independent, always-root,
// single-joint instances: what matters for VI is how many rows exist and
// how they're batched into lane groups, not the tree shape.
//
// VIFieldTraits still names TopologicalOrdering (shared with ABA, see
// vi_data.hpp) purely so a sim hosting both algorithms only needs one
// JointTopology; that's why MakeSim below still goes through the same
// Archetype/root_parents machinery aba_step.cpp's own MakeSim does, and why
// BatchGroup still resolves a joint to a lane group via the real Layout
// rather than assuming a row layout by hand.
//
// A View's batched Store/Store<F, MathematicalT> always writes one whole
// lane group at a time (see view.hpp's own comment on Load/Store), so two
// joints that land in the same lane group can't independently hold two
// different values through this API -- exactly the same constraint
// aba_step.cpp works within by only ever giving one real joint's data per
// group. The multi-group test below follows that same shape: it assigns
// one value per distinct lane GROUP (not per joint), so a joint's expected
// result only ever depends on which group it landed in.

using namespace achilles::algorithms;
using namespace achilles::algorithms::vi;
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

::testing::AssertionResult BatchApprox(const auto& lhs, const auto& rhs) {
  return BatchTrue(lhs.IsApprox(rhs));
}

// One archetype, `instance_count` independent single-joint instances, every
// one its own root (VI never reads a parent, so there's no tree to build
// beyond satisfying TopologicalOrdering::Build's own single-root-per-
// instance precondition -- see ordering_policy.hpp's SortTree comment).
SimAllocator<VIAlgorithm> MakeSim(std::size_t instance_count) {
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
  return SimAllocator<VIAlgorithm>(archetypes);
}

std::size_t BatchGroup(
    const achilles::engine::topology::Layout& layout, std::size_t joint
) {
  return layout.ToSorted(0, joint) / Lane();
}

void PopulateJoint(
    VIView& view,
    std::size_t group,
    const Acceleration& qdd,
    const Velocity& qd = Velocity::Zero()
) {
  view.Store<VIField::kJointAcceleration, B>(group, qdd);
  view.Store<VIField::kJointVelocity, B>(group, qd);
}

}  // namespace

// A single real joint: after one Step, its velocity must equal qdd*dt
// (starting from zero), recomputed independently via Vector6's own scalar
// multiply rather than by calling Acceleration::Integrate again -- proves
// the full Step (real VIView, real traversal, real OpInvoker) actually
// carries the acceleration field into the velocity field, not just that
// IntegrateVelocityOp::operator() does so in isolation
// (algorithms_vi_vi_ops.cpp).
TEST(VIStep, SingleJointIntegratesConstantAcceleration) {
  SimAllocator<VIAlgorithm> sim = MakeSim(1);
  VIView view = sim.ViewFor<VIAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);

  Acceleration qdd(
      Vector3(B(1.0F), B(0.0F), B(0.0F)), Vector3(B(0.0F), B(0.0F), B(-2.0F))
  );
  PopulateJoint(view, group, qdd);

  SimConfig sim_config;
  VIStep::Step(sim.SimContext(), sim_config, 0.5F);

  Velocity expected(qdd.AsVector6() * B(0.5F));
  EXPECT_TRUE(
      BatchApprox(view.Load<VIField::kJointVelocity, B>(group), expected)
  );
}

// Zero acceleration must leave a nonzero pre-existing velocity untouched --
// the Step-level counterpart to IntegrateVelocityOpTest.
// ZeroAccelerationLeavesVelocityUnchanged, now through the real traversal
// and View rather than a direct operator() call.
TEST(VIStep, ZeroAccelerationLeavesVelocityAtInitialValue) {
  SimAllocator<VIAlgorithm> sim = MakeSim(1);
  VIView view = sim.ViewFor<VIAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);

  Velocity qd_initial(
      Vector3(B(0.1F), B(0.2F), B(0.3F)), Vector3(B(0.4F), B(0.5F), B(0.6F))
  );
  PopulateJoint(view, group, Acceleration::Zero(), qd_initial);

  SimConfig sim_config;
  VIStep::Step(sim.SimContext(), sim_config, 1.0F);

  EXPECT_TRUE(BatchApprox(
      view.Load<VIField::kJointVelocity, B>(group), qd_initial
  ));
}

// IntegrateVelocityOp has no Initialize, so -- unlike ABA's articulated-
// inertia accumulator (see AbaStep.RepeatedStepsProduceConsistentResults)
// -- nothing resets kJointVelocity between Step() calls. That's the whole
// point of an integrator: repeated Step calls with the same constant
// acceleration must keep accumulating, so N steps of dt each must land on
// N*qdd*dt, not qdd*dt.
TEST(VIStep, RepeatedStepsAccumulateVelocity) {
  SimAllocator<VIAlgorithm> sim = MakeSim(1);
  VIView view = sim.ViewFor<VIAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);

  Acceleration qdd(Vector3::Zero(), Vector3(B(0.0F), B(0.0F), B(-9.8F)));
  PopulateJoint(view, group, qdd);

  SimConfig sim_config;
  constexpr int kSteps = 3;
  constexpr float kDt = 0.1F;
  for (int i = 0; i < kSteps; ++i) {
    VIStep::Step(sim.SimContext(), sim_config, kDt);
  }

  Velocity expected(qdd.AsVector6() * B(kDt * static_cast<float>(kSteps)));
  EXPECT_TRUE(
      BatchApprox(view.Load<VIField::kJointVelocity, B>(group), expected)
  );
}

// More than one lane group's worth of real joints: proves the
// LinearTraversal driving IntegrateVelocityOp actually walks every group
// view.Size()/Stride covers, not just group 0 -- the shape every other test
// in this file (a single real joint, always padded up to exactly one lane
// group) can't distinguish from a traversal that silently stopped after the
// first group.
TEST(VIStep, MultipleJointsAcrossLaneGroupsIntegrateIndependently) {
  std::size_t lane = Lane();
  std::size_t instance_count = lane * 2;
  SimAllocator<VIAlgorithm> sim = MakeSim(instance_count);
  VIView view = sim.ViewFor<VIAlgorithm>();
  const auto& layout = sim.LayoutFor<TopologicalOrdering>();

  std::set<std::size_t> groups;
  for (std::size_t i = 0; i < instance_count; ++i) {
    groups.insert(BatchGroup(layout, i));
  }
  ASSERT_GT(groups.size(), 1U)
      << "Test premise violated: fewer than two lane groups were "
         "populated, so this test can't distinguish per-group processing "
         "from a traversal that only ever touches group 0.";

  // One value per distinct GROUP (not per joint -- see this file's header
  // comment on why joints sharing a group must share a value).
  for (std::size_t group : groups) {
    float scale = static_cast<float>(group + 1);
    Acceleration qdd(Vector3::Zero(), Vector3(B(scale), B(0.0F), B(0.0F)));
    Velocity qd0(Vector3::Zero(), Vector3(B(0.0F), B(scale * 0.1F), B(0.0F)));
    PopulateJoint(view, group, qdd, qd0);
  }

  SimConfig sim_config;
  VIStep::Step(sim.SimContext(), sim_config, 1.0F);

  for (std::size_t group : groups) {
    float scale = static_cast<float>(group + 1);
    Acceleration qdd(Vector3::Zero(), Vector3(B(scale), B(0.0F), B(0.0F)));
    Velocity qd0(Vector3::Zero(), Vector3(B(0.0F), B(scale * 0.1F), B(0.0F)));
    Velocity expected = qd0 + Velocity(qdd.AsVector6() * B(1.0F));
    EXPECT_TRUE(BatchApprox(
        view.Load<VIField::kJointVelocity, B>(group), expected
    ));
  }
}
