#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/archetype_loader.hpp"
#include "interface/simulation.hpp"
#include "support/aba_reference.hpp"
#include "support/temp_dir.hpp"
#include "util/simd_ops.hpp"

using achilles::algorithms::Acceleration;
using achilles::algorithms::InertiaOperator;
using achilles::algorithms::MathematicalT;
using achilles::algorithms::SimConfig;
using achilles::algorithms::Transform;
using achilles::algorithms::Vector3;
using achilles::algorithms::Velocity;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::algorithms::aba::ABAStep;
using achilles::domain::Archetype;
using achilles::engine::memory::SimAllocator;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::LoadArchetypes;
using achilles::interface::Simulation;
using achilles::test_support::AccumulateAbaInertia;
using achilles::test_support::ComputeAbaAcceleration;
using achilles::test_support::ComputeAbaVelocity;
using achilles::test_support::TempDir;

using B = MathematicalT;

namespace {

// Same revolute-Z-about-slot-0 joint tests/support/aba_reference.hpp's own
// RevoluteZSubspace/DOF0ActiveMask/SimpleInertia describe, as the body of
// one joint's `fields:` block -- shared by both the single-joint archetype
// (MakeRevoluteZArow) and the multi-archetype chain
// (MultiArchetypeChainAcrossIncludedArowFilesPropagatesVelocityExactly)
// below, so both build the exact same physical joint. Flattened row-major
// (Matrix::operator()(i,j) == data_[i*N+j], see domain/math/matrix.hpp), so
// entry (2, 0) lands at flat index 12; rotation is [w,x,y,z] (Quaternion::
// ToTuple() is (W(),X(),Y(),Z()), scalar-first -- see domain/archetype.hpp),
// so identity is [1,0,0,0], not [0,0,0,1].
std::string RevoluteZFieldsYaml(
    float q0, float qd0 = 0.0F, float tau0 = 0.0F, int mask = 1
) {
  std::ostringstream out;
  out << "      joint_subspace: [0,0,0,0,0,0,0,0,0,0,0,0,"
         "1,"
         "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]\n"
      << "      joint_activation_mask: [" << mask << "]\n"
      << "      fixed_joint_transform: [0,0,0, 1,0,0,0]\n"
      << "      rigid_body_inertia: [2, 0,0,0, 2,3,4, 0,0,0]\n"
      << "      joint_position: [" << q0 << ", 0,0,0,0,0]\n"
      << "      joint_velocity: [" << qd0 << ", 0,0,0,0,0]\n"
      << "      joint_torque: [" << tau0 << ", 0,0,0,0,0]\n";
  return out.str();
}

// A single-joint, single-archetype .arow file using RevoluteZFieldsYaml --
// this is what makes every test below that uses it exercise the *real*
// LoadArchetypes/SimAllocator path (see interface/archetype_loader.hpp)
// rather than a hand-built Archetype the way algorithms_aba_aba_step.cpp's
// own tests do -- and, since the physical setup is identical to that
// file's, a real Simulation's own output can be checked against the exact
// same tests/support/aba_reference.hpp helpers those tests use.
std::string MakeRevoluteZArow(
    float q0, float qd0 = 0.0F, float tau0 = 0.0F, int mask = 1
) {
  std::ostringstream out;
  out << "archetype: base\n"
         "joints:\n"
         "  - name: joint\n"
         "    fields:\n"
      << RevoluteZFieldsYaml(q0, qd0, tau0, mask);
  return out.str();
}

// A Simulation loaded and initialized from a single real revolute-Z .arow
// file -- the common setup every test below starts from.
Simulation MakeInitializedSim(
    const TempDir& dir, float q0, float qd0 = 0.0F, float tau0 = 0.0F
) {
  std::filesystem::path file =
      dir.Write("base.arow", MakeRevoluteZArow(q0, qd0, tau0));
  Simulation sim;
  [[maybe_unused]] bool loaded = sim.LoadArowFile(file.string());
  [[maybe_unused]] bool initialized = sim.Init();
  return sim;
}

Transform WorldTransform(Simulation& sim) {
  return sim.ViewFor<ABAAlgorithm>().Load<ABAField::kWorldTransform, B>(0);
}

Velocity SpatialVelocity(Simulation& sim) {
  return sim.ViewFor<ABAAlgorithm>().Load<ABAField::kSpatialVelocity, B>(0);
}

Acceleration SpatialAcceleration(Simulation& sim) {
  return sim.ViewFor<ABAAlgorithm>().Load<ABAField::kSpatialAcceleration, B>(0);
}

::testing::AssertionResult BatchTrue(const auto& mask) {
  if (achilles::util::AllTrue(mask)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "not all lanes true";
}

::testing::AssertionResult BatchApprox(const auto& lhs, const auto& rhs) {
  return BatchTrue(lhs.IsApprox(rhs));
}

// Real archetype-driven Populate() only ever writes the *real* row(s) a
// .arow file actually declares -- every other raw row sharing that same
// SIMD lane group is genuine padding (SeedPadding's own PaddingSeed()
// value, e.g. Velocity::Zero()), not "the same scenario copied into every
// lane" the way algorithms_aba_aba_step.cpp's own PopulateRevoluteZJoint
// gets away with (it Store<Field, batch>()s one broadcast value straight
// into every lane of a group by construction). A tests/support/
// aba_reference.hpp reference call, by contrast, always broadcasts its
// scalar inputs (via B(x)) uniformly across all 4 lanes. So a real
// Simulation's batched output and a broadcast reference agree on lane 0
// (the one real joint) but generally do NOT agree on the other, genuinely-
// padding lanes -- comparing the whole batch via IsApprox/AllTrue the way
// BatchApprox above does is only valid when both sides are known to be
// uniform across lanes (e.g. comparing a real result against itself, or
// against another real result from the same population path). Whenever
// one side is a broadcast reference, this lane-0-only comparison is the
// correct one.
bool Lane0True(const auto& mask) {
  if constexpr (requires { mask.get(0); }) {
    return mask.get(0);
  } else {
    return mask;
  }
}

::testing::AssertionResult Lane0Approx(const auto& lhs, const auto& rhs) {
  if (Lane0True(lhs.IsApprox(rhs))) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "lane 0 not approximately equal";
}

}  // namespace

TEST(Simulation, LoadArowFileReturnsFalseForMissingFile) {
  Simulation sim;
  EXPECT_FALSE(sim.LoadArowFile("/nonexistent/path/does_not_exist.arow"));
}

TEST(Simulation, LoadArowFileReturnsTrueForARealFile) {
  TempDir dir;
  std::filesystem::path file = dir.Write("base.arow", MakeRevoluteZArow(0.3F));

  Simulation sim;
  EXPECT_TRUE(sim.LoadArowFile(file.string()));
}

// A real, but structurally invalid, .arow file (no `archetype:` key --
// see interface_archetype_loader.cpp's own
// LoadArchetypes.ThrowsWhenArchetypeKeyIsMissing for the same case exercised
// directly) must come back as a load failure through Simulation too, and
// Init() afterward must still see no archetypes loaded rather than
// whatever partial state a failed parse might have left behind.
TEST(Simulation, MalformedArowFileFailsToLoadAndInitStillFails) {
  TempDir dir;
  std::filesystem::path file = dir.Write("bad.arow", R"(
joints:
  - name: mount
    fields: { x: [0] }
)");

  Simulation sim;
  EXPECT_FALSE(sim.LoadArowFile(file.string()));
  EXPECT_FALSE(sim.Init());
}

// LoadSimConfig itself never throws -- a missing file, unparsable YAML, or a
// malformed field all fall back to SimConfig::Default() internally (see
// interface_sim_config_loader.cpp) -- so LoadConfigFile's own try/catch
// never actually fires for a missing path; it still returns true, having
// silently kept the default config. Locks in that real, load-bearing
// behavior rather than assuming a missing config file is an error.
TEST(Simulation, LoadConfigFileReturnsTrueAndFallsBackForMissingFile) {
  Simulation sim;
  EXPECT_TRUE(sim.LoadConfigFile("/nonexistent/path/does_not_exist.yaml"));
}

TEST(Simulation, InitReturnsFalseWithoutLoadingAnArowFileFirst) {
  Simulation sim;
  EXPECT_FALSE(sim.Init());
}

TEST(Simulation, StepReturnsFalseBeforeInit) {
  Simulation sim;
  EXPECT_FALSE(sim.Step(0.1F));
}

// Full stack: a real .arow file, loaded through the real LoadArchetypes,
// allocated by the real SimAllocatorForT<RegisteredAlgorithms>, stepped
// through the real ABAStep -- verifies the joint's own nonzero initial
// position actually reaches kWorldTransform as a nonzero rotation, proving
// every layer (archetype loading, allocation, View construction, the
// engine::pass::Step dispatch, ABAStep's own strided traversal) is wired
// together correctly end to end, not just that each layer works in
// isolation (which algorithms_aba_aba_step.cpp,
// engine_memory_sim_allocator.cpp, etc. already cover).
TEST(Simulation, LoadInitAndStepProduceARealNonzeroWorldTransform) {
  TempDir dir;
  Simulation sim = MakeInitializedSim(dir, 0.3F);
  ASSERT_TRUE(sim.Step(0.0F));

  EXPECT_FALSE(
      achilles::util::AllTrue(WorldTransform(sim).Rotation().IsIdentity())
  );
}

// At rest (q=0, qd=0), no gravity, no torque: a system already in
// equilibrium must stay in equilibrium, through the real full stack --
// the same property algorithms_aba_aba_step.cpp's AtRestStaysAtRest proves
// directly against a hand-built Archetype.
TEST(Simulation, AtRestStaysAtRestThroughTheRealStack) {
  TempDir dir;
  Simulation sim = MakeInitializedSim(dir, 0.0F);
  ASSERT_TRUE(sim.Step(0.0F));

  EXPECT_TRUE(BatchApprox(
      WorldTransform(sim).Translation(), Transform::Identity().Translation()
  ));
  EXPECT_TRUE(BatchTrue(SpatialVelocity(sim).IsZero()));
  EXPECT_TRUE(BatchTrue(SpatialAcceleration(sim).IsZero()));
}

// A nonzero initial joint velocity, with q=0 (so the fixed_joint_transform-
// composed x_up is exactly identity) and no gravity, must reach
// kSpatialVelocity exactly: v = x_up^-1.Apply(v_parent=0) + S*qd == qd's own
// slot-0 value read straight through, with no parent contribution to muddy
// the comparison -- checked against tests/support/aba_reference.hpp's own
// ComputeAbaVelocity, called directly with the same (v_parent, q, qd), so
// this is an exact equality, not just "changed to something nonzero".
TEST(Simulation, NonzeroVelocityFromArowProducesTheExactSpatialVelocity) {
  TempDir dir;
  Velocity qd0(Vector3(B(0.5F), B(0.0F), B(0.0F)), Vector3::Zero());
  Simulation sim = MakeInitializedSim(dir, 0.0F, 0.5F);
  ASSERT_TRUE(sim.Step(0.0F));

  auto reference =
      ComputeAbaVelocity(Transform::Identity(), Velocity::Zero(), B(0.0F), qd0);
  ASSERT_FALSE(achilles::util::AllTrue(reference.v.IsZero()))
      << "Test premise violated: qd0 produced zero velocity.";
  EXPECT_TRUE(Lane0Approx(SpatialVelocity(sim), reference.v));
}

// No LoadConfigFile call at all -- Simulation's own default-constructed
// SimConfig (world_base_transform=Identity, base_velocity=Zero,
// base_acceleration=Zero) must mean no gravity reaches a joint at rest.
TEST(Simulation, NoConfigFileMeansZeroGravityAtRest) {
  TempDir dir;
  Simulation sim = MakeInitializedSim(dir, 0.0F);
  ASSERT_TRUE(sim.Step(0.0F));

  EXPECT_TRUE(BatchTrue(SpatialAcceleration(sim).IsZero()));
}

// A real sim_config.yaml's own base_acceleration must reach a joint at rest
// as a nonzero kSpatialAcceleration -- proving LoadConfigFile's real,
// LoadSimConfig-parsed value actually flows into ABAStep::Step's own
// sim_config argument through Simulation::Step, not just that it parses.
// The reference chain (ComputeAbaVelocity -> AccumulateAbaInertia ->
// ComputeAbaAcceleration) mirrors exactly what a single real joint's
// Step passes do, so this is an exact comparison.
TEST(Simulation, GravityFromConfigFileProducesTheExactAcceleration) {
  TempDir dir;
  std::filesystem::path config_file = dir.Write("config.yaml", R"(
base_acceleration:
  angular: [0, 0, 0]
  linear: [0, 0, -9.8]
)");
  Simulation sim = MakeInitializedSim(dir, 0.0F);
  ASSERT_TRUE(sim.LoadConfigFile(config_file.string()));
  ASSERT_TRUE(sim.Step(0.0F));

  Acceleration gravity(Vector3::Zero(), Vector3(B(0.0F), B(0.0F), B(-9.8F)));
  auto velocity =
      ComputeAbaVelocity(Transform::Identity(), Velocity::Zero(), B(0.0F));
  InertiaOperator<false> i_a_base = InertiaOperator<false>::Zero();
  achilles::algorithms::Force p_base = achilles::algorithms::Force::Zero();
  auto inertia = AccumulateAbaInertia(
      velocity,
      achilles::algorithms::Force(Vector3::Zero(), Vector3::Zero()),
      i_a_base,
      p_base
  );
  Acceleration expected =
      ComputeAbaAcceleration(inertia, velocity.x_up, velocity.c, gravity);
  ASSERT_FALSE(achilles::util::AllTrue(expected.IsZero()))
      << "Test premise violated: gravity produced zero acceleration.";

  EXPECT_TRUE(Lane0Approx(SpatialAcceleration(sim), expected));
}

// LoadConfigFile can run *after* Init() -- config_ is read fresh by every
// Step() call (SimContext::Step takes it by parameter, never snapshotted
// at Init time), so loading a real gravity config between Init() and
// Step() must still reach that Step() call.
TEST(Simulation, ConfigFileLoadedAfterInitStillAffectsStep) {
  TempDir dir;
  Simulation sim = MakeInitializedSim(dir, 0.0F);

  std::filesystem::path config_file = dir.Write("config.yaml", R"(
base_acceleration:
  linear: [0, 0, -9.8]
)");
  ASSERT_TRUE(sim.LoadConfigFile(config_file.string()));
  ASSERT_TRUE(sim.Step(0.0F));

  EXPECT_FALSE(achilles::util::AllTrue(SpatialAcceleration(sim).IsZero()));
}

// world_base_transform from a real config file, with q=0 (x_up == Identity),
// must reach kWorldTransform exactly unchanged: x_world = x_world_parent *
// x_up == x_world_parent == the configured base transform.
TEST(
    Simulation, WorldBaseTransformFromConfigComposesExactlyIntoWorldTransform
) {
  TempDir dir;
  std::filesystem::path config_file = dir.Write("config.yaml", R"(
world_base_transform:
  translation: [1, 2, 3]
  rotation: [1, 0, 0, 0]
)");
  Simulation sim = MakeInitializedSim(dir, 0.0F);
  ASSERT_TRUE(sim.LoadConfigFile(config_file.string()));
  ASSERT_TRUE(sim.Step(0.0F));

  Vector3 expected_translation(B(1.0F), B(2.0F), B(3.0F));
  EXPECT_TRUE(
      BatchApprox(WorldTransform(sim).Translation(), expected_translation)
  );
}

// base_velocity from a real config file, with q=0 and qd=0 (so the joint
// contributes nothing of its own), must reach kSpatialVelocity exactly
// unchanged: v = x_up^-1.Apply(v_base) + S*qd(=0) == v_base (x_up ==
// Identity).
TEST(Simulation, BaseVelocityFromConfigComposesExactlyIntoSpatialVelocity) {
  TempDir dir;
  std::filesystem::path config_file = dir.Write("config.yaml", R"(
base_velocity:
  linear: [1, 0, 0]
)");
  Simulation sim = MakeInitializedSim(dir, 0.0F);
  ASSERT_TRUE(sim.LoadConfigFile(config_file.string()));
  ASSERT_TRUE(sim.Step(0.0F));

  Velocity expected(Vector3::Zero(), Vector3(B(1.0F), B(0.0F), B(0.0F)));
  EXPECT_TRUE(Lane0Approx(SpatialVelocity(sim), expected));
}

// A nonzero joint_torque from the .arow file must change the resulting
// acceleration relative to the same joint with zero torque -- proving
// torque genuinely reaches PropagateInertiaOp's own `u = tau - S^T p`
// through the real archetype-loading path, not just that Step() runs.
TEST(Simulation, TorqueFromArowChangesAcceleration) {
  TempDir no_torque_dir;
  Simulation no_torque = MakeInitializedSim(no_torque_dir, 0.0F);
  ASSERT_TRUE(no_torque.Step(0.0F));

  TempDir torque_dir;
  Simulation with_torque = MakeInitializedSim(torque_dir, 0.0F, 0.0F, 5.0F);
  ASSERT_TRUE(with_torque.Step(0.0F));

  EXPECT_FALSE(achilles::util::AllTrue(
      SpatialAcceleration(no_torque).IsApprox(SpatialAcceleration(with_torque))
  ));
}

// ABA solves for acceleration directly -- it doesn't integrate over time --
// so `dt` is declared but never read (see ABAStep::Step's own comment).
// Two otherwise-identical simulations stepped with wildly different dt
// values must produce bit-for-bit the same acceleration through the real
// full stack, not just when calling ABAStep::Step directly.
TEST(Simulation, DtDoesNotAffectTheAbaResult) {
  TempDir dir_a;
  Simulation short_dt = MakeInitializedSim(dir_a, 0.3F, 0.5F);
  ASSERT_TRUE(short_dt.Step(0.001F));

  TempDir dir_b;
  Simulation long_dt = MakeInitializedSim(dir_b, 0.3F, 0.5F);
  ASSERT_TRUE(long_dt.Step(50.0F));

  EXPECT_TRUE(
      BatchApprox(SpatialAcceleration(short_dt), SpatialAcceleration(long_dt))
  );
}

// Two independently constructed, independently loaded Simulations, given
// byte-identical .arow content, must produce byte-identical results --
// the real full stack (parsing, allocation, layout, Step) has no hidden
// nondeterminism (uninitialized reads, unordered-map iteration leaking
// into results, etc.).
TEST(
    Simulation, TwoIndependentSimulationsFromTheSameArowProduceIdenticalResults
) {
  TempDir dir_a;
  Simulation sim_a = MakeInitializedSim(dir_a, 0.3F, 0.5F);
  ASSERT_TRUE(sim_a.Step(0.1F));

  TempDir dir_b;
  Simulation sim_b = MakeInitializedSim(dir_b, 0.3F, 0.5F);
  ASSERT_TRUE(sim_b.Step(0.1F));

  EXPECT_TRUE(BatchApprox(
      WorldTransform(sim_a).Translation(), WorldTransform(sim_b).Translation()
  ));
  EXPECT_TRUE(BatchApprox(SpatialVelocity(sim_a), SpatialVelocity(sim_b)));
  EXPECT_TRUE(
      BatchApprox(SpatialAcceleration(sim_a), SpatialAcceleration(sim_b))
  );
}

// Step() must be usable more than once in a row -- the shape a real
// simulation loop actually calls it in -- and, since ABA re-derives
// everything from the view's current state each call (nothing carries over
// except what PropagateInertiaOp::Initialize deliberately resets), stepping
// twice with the same unchanged input must reproduce the same result.
TEST(Simulation, StepCanBeCalledRepeatedlyWithConsistentResults) {
  TempDir dir;
  Simulation sim = MakeInitializedSim(dir, 0.3F);

  ASSERT_TRUE(sim.Step(0.1F));
  Transform after_first = WorldTransform(sim);
  Acceleration accel_after_first = SpatialAcceleration(sim);

  ASSERT_TRUE(sim.Step(0.1F));
  EXPECT_TRUE(
      BatchApprox(WorldTransform(sim).Translation(), after_first.Translation())
  );
  EXPECT_TRUE(BatchApprox(SpatialAcceleration(sim), accel_after_first));
}

// LoadArowFile + Init() can be called again on the same Simulation with a
// *different* file -- Init() must rebuild state_ from the newly loaded
// archetypes, not retain anything from the first Init(). The reloaded
// sim's own result must match a fresh Simulation built directly from the
// second file, proving the reload is a genuine rebuild and not stale state
// bleeding through.
TEST(Simulation, ReloadingADifferentArowFileAndReinitializingUsesTheNewData) {
  TempDir dir;
  Simulation sim = MakeInitializedSim(dir, 0.3F);
  ASSERT_TRUE(sim.Step(0.0F));
  Transform first_world = WorldTransform(sim);

  std::filesystem::path second_file =
      dir.Write("second.arow", MakeRevoluteZArow(1.2F));
  ASSERT_TRUE(sim.LoadArowFile(second_file.string()));
  ASSERT_TRUE(sim.Init());
  ASSERT_TRUE(sim.Step(0.0F));

  TempDir fresh_dir;
  Simulation fresh = MakeInitializedSim(fresh_dir, 1.2F);
  ASSERT_TRUE(fresh.Step(0.0F));

  EXPECT_FALSE(achilles::util::AllTrue(
      WorldTransform(sim).Rotation().IsApprox(first_world.Rotation())
  ));
  EXPECT_TRUE(BatchApprox(
      WorldTransform(sim).Translation(), WorldTransform(fresh).Translation()
  ));
}

// A real, *multi-archetype* .arow scenario: "base" (root, one joint,
// "mount") and "child" (one joint, "hub", attached to base's own mount via
// a real `includes`/`attach` pair across two separate files -- the same
// mechanism interface_archetype_loader.cpp's own
// LoadArchetypes.RecursiveIncludeAndBroadcastAttachToSingleParentInstance
// exercises directly). Simulation itself has no way to say "give me the
// View index for this specific joint" (Layout is construction-only -- see
// SimAllocator's own comment -- and Simulation never keeps one past Init()),
// so this test goes around Simulation and builds the SimAllocator directly
// from the same real LoadArchetypes(...) output Simulation::LoadArowFile
// would produce, giving it LayoutFor<TopologicalOrdering>().ToSorted(...)
// to resolve each joint's own real row exactly -- not a guessed index, and
// not a scan. LoadArchetypes' own contract (root archetype first) plus
// ToSorted give an exact, verified row for each of the two real joints.
TEST(
    Simulation,
    MultiArchetypeChainAcrossIncludedArowFilesPropagatesVelocityExactly
) {
  TempDir dir;
  dir.Write(
      "base.arow",
      "archetype: base\n"
      "joints:\n"
      "  - name: mount\n"
      "    fields:\n" +
          RevoluteZFieldsYaml(0.0F, 0.5F)
  );
  std::filesystem::path child_file = dir.Write(
      "child.arow",
      "archetype: child\n"
      "includes: [base.arow]\n"
      "attach: {archetype: base, joint: mount}\n"
      "joints:\n"
      "  - name: hub\n"
      "    fields:\n" +
          RevoluteZFieldsYaml(0.0F, 0.0F)
  );

  std::vector<Archetype> archetypes = LoadArchetypes(child_file);
  ASSERT_EQ(archetypes.size(), 2U);
  ASSERT_EQ(archetypes[0].Name(), "base");
  ASSERT_EQ(archetypes[1].Name(), "child");

  SimAllocator<ABAAlgorithm> sim(archetypes);
  auto view = sim.ViewFor<ABAAlgorithm>();
  const auto& layout = sim.LayoutFor<TopologicalOrdering>();
  std::size_t lane = xsimd::batch<float>::size;
  std::size_t root_group = layout.ToSorted(0, 0) / lane;
  std::size_t child_group = layout.ToSorted(1, 0) / lane;
  ASSERT_NE(root_group, child_group);

  SimConfig sim_config;
  ABAStep::Step(view, sim.SimContext(), sim_config, 0.0F);

  // Same reference as PropagatesThroughTwoJointChain
  // (algorithms_aba_aba_step.cpp): joint 0's own nonzero qd, joint 1's
  // v_parent read as joint 0's real output, identity x_up throughout ->
  // joint 1's v must come out exactly equal to joint 0's.
  Velocity qd0(Vector3(B(0.5F), B(0.0F), B(0.0F)), Vector3::Zero());
  auto reference =
      ComputeAbaVelocity(Transform::Identity(), Velocity::Zero(), B(0.0F), qd0);
  ASSERT_FALSE(achilles::util::AllTrue(reference.v.IsZero()))
      << "Test premise violated: qd0 produced zero velocity.";

  EXPECT_TRUE(Lane0Approx(
      view.Load<ABAField::kSpatialVelocity, B>(root_group), reference.v
  ));
  EXPECT_TRUE(Lane0Approx(
      view.Load<ABAField::kSpatialVelocity, B>(child_group), reference.v
  ));
}
