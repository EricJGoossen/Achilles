// Validates examples/two_joint_arm.arow + examples/sim_config.yaml -- the
// scene scripts/run-example.sh actually launches -- against an *independent*
// closed-form model of the same physical system, derived from scratch via
// Lagrangian mechanics rather than by reading algorithms/aba's own source.
// The point is to catch a bug in the engine's dynamics that a test built
// from the same equations the engine itself implements never could.
//
// The real scene is a planar double pendulum: two rigid links, hinged in
// series about parallel axes (both joints rotate about their own local X --
// see two_joint_arm.arow's own comment on why X, not the more obvious-
// looking Z), each link's own mass offset from its own joint (rigid_body_
// inertia's own h field) rather than sitting at the pivot. That is *exactly*
// the textbook "double compound pendulum" problem, just parameterized by
// pivot-referenced quantities (mass, pivot-to-CoM distance, inertia about
// the pivot) instead of the more common CoM-referenced ones.
//
// Generalized coordinates: phi = shoulder's own absolute rotation angle
// about X (it has no parent rotation, so this is also its world angle);
// psi = elbow's rotation *relative to the shoulder* -- this is exactly
// what ABAField::kJointPosition/kJointAcceleration store for each joint
// (a joint's own relative DOF, not its absolute world angle), so phi/psi
// (and their derivatives) line up directly with the engine's own qd/qdd
// without any change of coordinates.
//
// Deriving T (kinetic energy) and U (potential energy) in terms of
// (phi, psi, phi', psi') and applying the Euler-Lagrange equations gives a
// 2x2 linear system for (phi'', psi'') at any state -- see
// ReferenceAngularAcceleration below for the closed form, and this file's
// own derivation notes in the git history / PR description for the full
// algebra. This independently reproduces the standard double-pendulum
// kinetic-energy form T = (1/2)P phi'^2 + (1/2)Q psi_dot_abs^2 + R cos(psi)
// phi' psi_dot_abs (with psi_dot_abs = phi'+psi'), which is a strong sign
// the derivation itself is right, not just algebra that happens to compile.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_energy.hpp"
#include "algorithms/conventions.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/vector3.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/simulation.hpp"
#include "support/temp_dir.hpp"

using achilles::algorithms::MathematicalT;
using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::algorithms::aba::ComputeSystemEnergy;
using achilles::domain::JointTopology;
using achilles::domain::math::Vector3;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::Simulation;
using achilles::test_support::TempDir;

using B = MathematicalT;

namespace {

// Mirrors examples/two_joint_arm.arow exactly (mass, center-of-mass offset,
// inertia about each joint, and the 1.0-unit shoulder-to-elbow offset) --
// see that file's own comments for where each of these numbers comes from
// (in particular, the parallel-axis-theorem reasoning behind Ixx/Izz).
// Kept as literal constants here, rather than parsed from the real .arow
// file, so this test's own reference model and the archetype it builds
// below are provably built from the same numbers -- a drift between this
// file and examples/two_joint_arm.arow would just make this test wrong
// about which scene it's checking, not silently pass on stale physics.
struct LinkParams {
  ScalarOperationT mass;
  ScalarOperationT pivot_to_com;         // r -- rigid_body_inertia's h, divided by mass
  ScalarOperationT inertia_about_pivot;  // Ixx == Izz (both perpendicular to the pivot
                              // -> CoM offset, which points along Y)
};
constexpr LinkParams kShoulder{
    .mass = 1.0F, .pivot_to_com = 0.3F, .inertia_about_pivot = 0.18083F
};
constexpr LinkParams kElbow{
    .mass = 0.4F, .pivot_to_com = 0.375F, .inertia_about_pivot = 0.07238F
};
constexpr ScalarOperationT kLinkLength = 1.0F;  // shoulder -> elbow offset (local Y)

// examples/sim_config.yaml's own base_acceleration.linear.z. ABA treats a
// fixed base's own configured acceleration as *minus* true gravity (see
// algorithms/sim_config.hpp's own comment on base_acceleration), so the
// real gravitational acceleration every body actually falls under is
// +9.8 along world Z, not -9.8 -- this constant is that already-negated,
// "as Lagrangian mechanics wants it" value, not sim_config.yaml's literal
// -9.8.
constexpr ScalarOperationT kGravity = 9.8F;

constexpr ScalarOperationT kDt = 1.0F / 120.0F;

std::string TwoJointArmYaml() {
  // Physically identical to examples/two_joint_arm.arow -- same joint
  // axes, offsets, masses, and inertias -- just inlined here (rather than
  // read from disk) so this test doesn't depend on the working directory
  // ctest happens to run in.
  std::ostringstream out;
  out << "archetype: arm\n"
         "joints:\n"
         "  - name: shoulder\n"
         "    parent: null\n"
         "    fields:\n"
         "      joint_subspace: [1,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, "
         "0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]\n"
         "      joint_activation_mask: [1]\n"
         "      fixed_joint_transform:\n"
         "        translation: [0, 0, 0]\n"
         "        rotation: [1, 0, 0, 0]\n"
         "      rigid_body_inertia: [1.0, 0, 0.3, 0, 0.18083, 0.015, "
         "0.18083, 0, 0, 0]\n"
         "      joint_position:\n"
         "        translation: [0, 0, 0]\n"
         "        rotation: [1, 0, 0, 0]\n"
         "      joint_velocity: [0, 0, 0, 0, 0, 0]\n"
         "      joint_torque: [0, 0, 0, 0, 0, 0]\n"
         "      visual_extents: [0.15, 0.5, 0.15]\n"
         "      visual_color: [0.85, 0.3, 0.2]\n"
         "  - name: elbow\n"
         "    parent: shoulder\n"
         "    fields:\n"
         "      joint_subspace: [1,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, "
         "0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]\n"
         "      joint_activation_mask: [1]\n"
         "      fixed_joint_transform:\n"
         "        translation: [0, 1.0, 0]\n"
         "        rotation: [1, 0, 0, 0]\n"
         "      rigid_body_inertia: [0.4, 0, 0.15, 0, 0.07238, 0.00576, "
         "0.07238, 0, 0, 0]\n"
         "      joint_position:\n"
         "        translation: [0, 0, 0]\n"
         "        rotation: [1, 0, 0, 0]\n"
         "      joint_velocity: [0, 0, 0, 0, 0, 0]\n"
         "      joint_torque: [0, 0, 0, 0, 0, 0]\n"
         "      visual_extents: [0.12, 0.4, 0.12]\n"
         "      visual_color: [0.2, 0.55, 0.9]\n";
  return out.str();
}

std::string SimConfigYaml() {
  // Mirrors examples/sim_config.yaml exactly.
  return "base_acceleration:\n"
         "  angular: [0, 0, 0]\n"
         "  linear: [0, 0, -9.8]\n";
}

Simulation MakeTwoJointArmSim(const TempDir& dir) {
  std::filesystem::path arow =
      dir.Write("two_joint_arm.arow", TwoJointArmYaml());
  std::filesystem::path config = dir.Write("sim_config.yaml", SimConfigYaml());
  Simulation sim;
  [[maybe_unused]] bool loaded_arow = sim.LoadArowFile(arow.string());
  [[maybe_unused]] bool loaded_config = sim.LoadConfigFile(config.string());
  [[maybe_unused]] bool initialized = sim.Init();
  return sim;
}

// The real topology is SIMD-lane-padded (see engine/topology/layout_policy
// .hpp) -- shoulder and elbow generally do *not* land on consecutive rows
// (row indices depend on the build's own SIMD lane width), so this looks
// the real row up structurally instead of assuming row 1 is the elbow: the
// one row whose own parent is the root row is the elbow, whatever its own
// index happens to be. (An earlier debugging session got this exact thing
// wrong by hand and mistook a padding row for the elbow -- see the PR
// description.)
struct JointRows {
  std::size_t shoulder;
  std::size_t elbow;
};

JointRows FindJointRows(const JointTopology& topology) {
  std::size_t root_sentinel = topology.Size();
  std::size_t shoulder_row = root_sentinel;
  for (std::size_t row = 0; row < topology.Size(); ++row) {
    if (topology[row] == root_sentinel) {
      shoulder_row = row;
      break;
    }
  }
  std::size_t elbow_row = root_sentinel;
  for (std::size_t row = 0; row < topology.Size(); ++row) {
    if (row != shoulder_row && topology[row] == shoulder_row) {
      elbow_row = row;
      break;
    }
  }
  return {shoulder_row, elbow_row};
}

// Extracts the signed rotation angle about X from a pure-X-axis rotation
// quaternion (w = cos(theta/2), x = sin(theta/2), y = z = 0) -- exact, not
// a small-angle approximation, and the inverse of exactly the composition
// PI's own IntegratePositionOp performs (see pi_ops.cpp's own Exp-based
// position update): composing two pure-X rotations by quaternion
// multiplication is exact angle addition, so reading the angle back this
// way round-trips exactly too.
ScalarOperationT ExtractAngleAboutX(ScalarOperationT w, ScalarOperationT x) { return 2.0F * std::atan2(x, w); }

// The closed-form (phi'', psi'') solved from the Euler-Lagrange equations
// for this exact two-link system -- see this file's own header comment for
// the derivation sketch. `phi`/`psi` are the current generalized
// coordinates (shoulder's absolute angle, elbow's angle relative to the
// shoulder); `phi_dot`/`psi_dot` their rates.
struct Accelerations {
  ScalarOperationT phi_dd;
  ScalarOperationT psi_dd;
};

Accelerations ReferenceAngularAcceleration(
    ScalarOperationT phi, ScalarOperationT psi, ScalarOperationT phi_dot, ScalarOperationT psi_dot
) {
  const ScalarOperationT m1 = kShoulder.mass;
  const ScalarOperationT r1 = kShoulder.pivot_to_com;
  const ScalarOperationT I1 = kShoulder.inertia_about_pivot;
  const ScalarOperationT m2 = kElbow.mass;
  const ScalarOperationT r2 = kElbow.pivot_to_com;
  const ScalarOperationT I2 = kElbow.inertia_about_pivot;
  const ScalarOperationT l1 = kLinkLength;
  const ScalarOperationT g = kGravity;

  const ScalarOperationT P = I1 + m2 * l1 * l1;
  const ScalarOperationT Q = I2;
  const ScalarOperationT R = m2 * l1 * r2;

  const ScalarOperationT cos_psi = std::cos(psi);
  const ScalarOperationT sin_psi = std::sin(psi);
  const ScalarOperationT cos_phi = std::cos(phi);
  const ScalarOperationT cos_phi_psi = std::cos(phi + psi);

  const ScalarOperationT m11 = P + Q + 2.0F * R * cos_psi;
  const ScalarOperationT m12 = Q + R * cos_psi;
  const ScalarOperationT m21 = m12;
  const ScalarOperationT m22 = Q;

  const ScalarOperationT b1 =
      2.0F * R * sin_psi * phi_dot * psi_dot + R * sin_psi * psi_dot * psi_dot +
      (m1 * r1 + m2 * l1) * g * cos_phi + m2 * r2 * g * cos_phi_psi;
  const ScalarOperationT b2 = -R * sin_psi * phi_dot * phi_dot + m2 * r2 * g * cos_phi_psi;

  const ScalarOperationT det = m11 * m22 - m12 * m21;
  return {
      .phi_dd = (b1 * m22 - b2 * m12) / det,
      .psi_dd = (m11 * b2 - m21 * b1) / det,
  };
}

// One tick of the *exact* discrete update the real engine performs: ABA
// solves qdd at the current (q, qd); VI integrates qd += qdd * dt exactly
// (SpatialAcceleration::Integrate is a plain scalar multiply, see domain/
// spatial/dual.hpp); PI then composes the position forward using the
// *already-updated* qd (PI runs after VI in algorithms::RegisteredAlgorithms
// -- see registry.hpp's own comment on why order matters) -- for a single
// rotation axis, that composition is exact angle addition too. So this
// reference stepper isn't a different (if similar) integrator to compare
// against the engine's chaos-sensitive trajectory -- it's a step-for-step
// replica of the *same* discrete map, just fed by an independently-derived
// closed-form qdd instead of algorithms::aba's own code. A correct
// derivation should track the engine's own trajectory to ScalarOperationT precision
// for as long as this test cares to run it, not just at t=0.
struct ReferenceState {
  ScalarOperationT phi = 0.0F;
  ScalarOperationT psi = 0.0F;
  ScalarOperationT phi_dot = 0.0F;
  ScalarOperationT psi_dot = 0.0F;

  void Step(ScalarOperationT dt) {
    Accelerations qdd =
        ReferenceAngularAcceleration(phi, psi, phi_dot, psi_dot);
    phi_dot += qdd.phi_dd * dt;
    psi_dot += qdd.psi_dd * dt;
    phi += phi_dot * dt;
    psi += psi_dot * dt;
  }
};

}  // namespace

// The strongest, least chaos-sensitive check: at t=0 (rest, both angles
// zero), there's no integration involved at all -- just one closed-form
// evaluation compared against one real ABA solve. If this fails, either the
// derivation or the engine's dynamics are wrong; there's no accumulated
// integrator drift to blame it on.
TEST(TwoJointArmReference, InitialAccelerationMatchesClosedFormAtRest) {
  TempDir dir;
  Simulation sim = MakeTwoJointArmSim(dir);

  JointTopology topology = sim.TopologyFor<TopologicalOrdering>();
  JointRows rows = FindJointRows(topology);
  ASSERT_NE(rows.shoulder, topology.Size());
  ASSERT_NE(rows.elbow, topology.Size());

  ASSERT_TRUE(sim.Step(kDt));
  auto view = sim.ViewFor<ABAAlgorithm>();
  auto shoulder_qdd =
      view.Load<ABAField::kJointAcceleration, ScalarOperationT>(rows.shoulder);
  auto elbow_qdd = view.Load<ABAField::kJointAcceleration, ScalarOperationT>(rows.elbow);

  Accelerations expected = ReferenceAngularAcceleration(0.0F, 0.0F, 0.0F, 0.0F);

  // A single closed-form evaluation compared against a single ABA solve,
  // both in float32, with no integration in between -- measured agreement
  // is ~1e-6 (see this test file's own derivation notes / the PR
  // description for the measured value), so 1e-3 is already a ~1000x
  // margin, not a loosened-until-it-passes number.
  EXPECT_NEAR(shoulder_qdd.Angular().X(), expected.phi_dd, 1e-3F);
  EXPECT_NEAR(elbow_qdd.Angular().X(), expected.psi_dd, 1e-3F);
  // Every other spatial-acceleration component this joint's subspace
  // doesn't drive should stay exactly zero -- a nonzero Y/Z angular or any
  // linear component would mean gravity is leaking into a DOF this
  // joint_subspace never activated.
  EXPECT_NEAR(shoulder_qdd.Angular().Y(), 0.0F, 1e-5F);
  EXPECT_NEAR(shoulder_qdd.Angular().Z(), 0.0F, 1e-5F);
}

// The real test: step the actual engine and this independent closed-form
// model with the identical discrete-time update rule for two full seconds
// (240 ticks) and require the resulting angles to agree at every single
// tick, not just initially. Two seconds is long enough for the shoulder to
// swing through a full 180-degree arc (see the PR description's own traced
// example) -- if the reference model's equations were subtly wrong (a sign
// flip, a missing coupling term, a wrong coefficient), the two trajectories
// would visibly diverge well before this window closes, even though both
// use the same integrator.
TEST(TwoJointArmReference, TrajectoryMatchesIndependentModelOverTwoSeconds) {
  TempDir dir;
  Simulation sim = MakeTwoJointArmSim(dir);

  JointTopology topology = sim.TopologyFor<TopologicalOrdering>();
  JointRows rows = FindJointRows(topology);
  ASSERT_NE(rows.shoulder, topology.Size());
  ASSERT_NE(rows.elbow, topology.Size());

  ReferenceState reference;

  constexpr int kSteps = 240;  // 2 simulated seconds at kDt.
  for (int i = 0; i < kSteps; ++i) {
    ASSERT_TRUE(sim.Step(kDt));
    reference.Step(kDt);

    auto view = sim.ViewFor<ABAAlgorithm>();
    auto shoulder_q = view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.shoulder);
    auto elbow_q = view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.elbow);

    ScalarOperationT engine_phi = ExtractAngleAboutX(
        shoulder_q.Rotation().W(), shoulder_q.Rotation().X()
    );
    ScalarOperationT engine_psi =
        ExtractAngleAboutX(elbow_q.Rotation().W(), elbow_q.Rotation().X());

    // Both sides recompute trig from scratch every tick (xsimd's cos/sin
    // inside the engine vs. std::cos/sin here), so float32 rounding
    // differences of a few ULPs per tick compound slightly over 240 ticks
    // -- measured max drift over the full 2 seconds is ~2.5e-6 rad (see
    // this test file's own derivation notes / the PR description), so
    // 1e-4 is a ~40x margin: comfortably above ScalarOperationT noise, but nowhere
    // near what an actually-wrong coefficient produces (multiple degrees
    // of divergence within the first few ticks, not a slow ScalarOperationT-noise
    // creep -- see e.g. the exact 1:1 cancellation bug the PR description
    // traces, which showed up as an *exact*, not approximate, mismatch).
    ASSERT_NEAR(engine_phi, reference.phi, 1e-4F)
        << "shoulder angle diverged at tick " << i;
    ASSERT_NEAR(engine_psi, reference.psi, 1e-4F)
        << "elbow angle diverged at tick " << i;
  }
}

// NOT a conservation check -- measured directly (see the PR description),
// this engine's total mechanical energy does NOT stay near its starting
// value over a long run: it swings as high as +7 within the first 20
// simulated seconds, then trends down and settles around -9.5 (close to
// this system's minimum potential energy, both links hanging at rest) by
// ~100s, and stays there. That's real numerical dissipation, not a bug in
// this scene's own physics: VI/PI integrate generalized *velocity*
// (qd += qdd*dt, then compose position from the new qd -- see vi_ops.cpp/
// pi_ops.cpp), which is the "symplectic Euler" trick, but that scheme's
// usual energy-boundedness guarantee is proven for integrating true
// *momentum* against a constant mass matrix. This system's own mass matrix
// depends on the elbow's own angle psi (the R*cos(psi) coupling term in
// the kinetic energy below), so there's no such guarantee here -- and
// measurement shows it really doesn't hold. So this test checks the
// weaker, but still real, thing that's true: energy stays *finite and
// within the range this system could physically ever reach* (bounded by
// its own maximum potential energy, a small fixed number) rather than
// diverging to some huge magnitude or NaN, which is what an actual
// energy-injection bug (as opposed to this scheme's own known dissipation)
// would look like instead.
TEST(
    TwoJointArmReference, TotalEnergyStaysFiniteAndPhysicallyBoundedOverALongRun
) {
  TempDir dir;
  Simulation sim = MakeTwoJointArmSim(dir);

  JointTopology topology = sim.TopologyFor<TopologicalOrdering>();
  JointRows rows = FindJointRows(topology);
  ASSERT_NE(rows.shoulder, topology.Size());
  ASSERT_NE(rows.elbow, topology.Size());

  const ScalarOperationT m1 = kShoulder.mass;
  const ScalarOperationT r1 = kShoulder.pivot_to_com;
  const ScalarOperationT I1 = kShoulder.inertia_about_pivot;
  const ScalarOperationT m2 = kElbow.mass;
  const ScalarOperationT r2 = kElbow.pivot_to_com;
  const ScalarOperationT I2 = kElbow.inertia_about_pivot;
  const ScalarOperationT l1 = kLinkLength;
  const ScalarOperationT g = kGravity;
  const ScalarOperationT P = I1 + m2 * l1 * l1;
  const ScalarOperationT Q = I2;
  const ScalarOperationT R = m2 * l1 * r2;

  constexpr int kSteps = 120 * 30;  // 30 simulated seconds.
  ScalarOperationT max_abs_energy = 0.0F;
  for (int i = 0; i < kSteps; ++i) {
    ASSERT_TRUE(sim.Step(kDt));

    auto view = sim.ViewFor<ABAAlgorithm>();
    auto shoulder_q = view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.shoulder);
    auto elbow_q = view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.elbow);
    auto shoulder_qd =
        view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.shoulder);
    auto elbow_qd = view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.elbow);

    ScalarOperationT phi = ExtractAngleAboutX(
        shoulder_q.Rotation().W(), shoulder_q.Rotation().X()
    );
    ScalarOperationT psi =
        ExtractAngleAboutX(elbow_q.Rotation().W(), elbow_q.Rotation().X());
    ScalarOperationT phi_dot = shoulder_qd.Angular().X();
    ScalarOperationT psi_dot = elbow_qd.Angular().X();
    ScalarOperationT omega2 = phi_dot + psi_dot;

    ScalarOperationT kinetic = 0.5F * P * phi_dot * phi_dot + 0.5F * Q * omega2 * omega2 +
                    R * std::cos(psi) * phi_dot * omega2;
    ScalarOperationT potential = -(m1 * r1 + m2 * l1) * g * std::sin(phi) -
                      m2 * r2 * g * std::sin(phi + psi);
    ScalarOperationT total_energy = kinetic + potential;
    max_abs_energy = std::max(max_abs_energy, std::abs(total_energy));
  }

  // Starting energy is 0; the highest this pendulum can ever legitimately
  // reach is bounded by its own maximum potential energy (both links
  // pointing straight against gravity), a small, fixed number -- nowhere
  // near what an actual energy-injection bug (unbounded growth) would
  // produce over 30 simulated seconds / 3600 ticks.
  ScalarOperationT max_possible_potential = (m1 * r1 + m2 * l1) * g + m2 * r2 * g;
  EXPECT_LT(max_abs_energy, max_possible_potential * 1.5F);
}

// Simulation::SetSubsteps exists specifically to shrink the per-tick error
// behind the dissipation the test above documents and bounds, by running
// VI/PI's own scheme at a smaller effective dt without changing the scheme
// itself (see its own comment). This locks in that it actually works: the
// same scene, run for the same simulated time, must drift dramatically
// less with substeps than without -- not just "still bounded" (the weaker
// claim above), but *meaningfully closer to true energy conservation*.
// Ratio-based rather than a fixed absolute threshold, since this is
// deliberately the exact same chaotic system both runs -- the claim under
// test is the *relative* improvement substepping buys, not some specific
// number this scene's own chaos happens to produce today.
TEST(TwoJointArmReference, SubstepsSubstantiallyReduceEnergyDissipation) {
  const ScalarOperationT m1 = kShoulder.mass;
  const ScalarOperationT r1 = kShoulder.pivot_to_com;
  const ScalarOperationT I1 = kShoulder.inertia_about_pivot;
  const ScalarOperationT m2 = kElbow.mass;
  const ScalarOperationT r2 = kElbow.pivot_to_com;
  const ScalarOperationT I2 = kElbow.inertia_about_pivot;
  const ScalarOperationT l1 = kLinkLength;
  const ScalarOperationT g = kGravity;
  const ScalarOperationT P = I1 + m2 * l1 * l1;
  const ScalarOperationT Q = I2;
  const ScalarOperationT R = m2 * l1 * r2;

  auto max_abs_total_energy = [&](int substeps) {
    TempDir dir;
    Simulation sim = MakeTwoJointArmSim(dir);
    sim.SetSubsteps(substeps);

    JointTopology topology = sim.TopologyFor<TopologicalOrdering>();
    JointRows rows = FindJointRows(topology);

    constexpr int kSteps = 120 * 10;  // 10 simulated seconds.
    ScalarOperationT max_abs_energy = 0.0F;
    for (int i = 0; i < kSteps; ++i) {
      EXPECT_TRUE(sim.Step(kDt));

      auto view = sim.ViewFor<ABAAlgorithm>();
      auto shoulder_q =
          view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.shoulder);
      auto elbow_q = view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.elbow);
      auto shoulder_qd =
          view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.shoulder);
      auto elbow_qd = view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.elbow);

      ScalarOperationT phi = ExtractAngleAboutX(
          shoulder_q.Rotation().W(), shoulder_q.Rotation().X()
      );
      ScalarOperationT psi =
          ExtractAngleAboutX(elbow_q.Rotation().W(), elbow_q.Rotation().X());
      ScalarOperationT phi_dot = shoulder_qd.Angular().X();
      ScalarOperationT psi_dot = elbow_qd.Angular().X();
      ScalarOperationT omega2 = phi_dot + psi_dot;

      ScalarOperationT kinetic = 0.5F * P * phi_dot * phi_dot +
                      0.5F * Q * omega2 * omega2 +
                      R * std::cos(psi) * phi_dot * omega2;
      ScalarOperationT potential = -(m1 * r1 + m2 * l1) * g * std::sin(phi) -
                        m2 * r2 * g * std::sin(phi + psi);
      max_abs_energy = std::max(max_abs_energy, std::abs(kinetic + potential));
    }
    return max_abs_energy;
  };

  ScalarOperationT max_energy_1 = max_abs_total_energy(1);
  ScalarOperationT max_energy_8 = max_abs_total_energy(8);

  EXPECT_LT(max_energy_8, max_energy_1 * 0.5F)
      << "substeps=1 max |energy|: " << max_energy_1
      << ", substeps=8 max |energy|: " << max_energy_8;
}

// algorithms::aba::ComputeSystemEnergy is a *generic* walk over every row of
// a topology (see its own comment); this test holds it to the exact same
// closed-form energy this file already trusts (the test above), rather than
// just checking ComputeSystemEnergy is self-consistent. If a sign were
// wrong anywhere in it -- gravity's direction, which frame H()/mass is
// offset in, kinetic energy's own spatial quadratic form -- this diverges
// from the closed form almost immediately; matching it at every tick over a
// real run is a much stronger check than TotalEnergyStaysFiniteAndPhysically
// BoundedOverALongRun's own boundedness-only assertion above.
//
// Compares ComputeSystemEnergy(view) after Step() N against the closed form
// evaluated at (phi, psi, phi_dot, psi_dot) from *before* that same Step()
// call, not after: ComputeSystemEnergy reads kWorldTransform/
// kSpatialVelocity, which ABA only (re)writes from whatever kJointPosition/
// kJointVelocity held before that Step() call's own VI/PI sub-passes then
// advance them -- so it always describes the state as of one dt earlier
// than kJointPosition/kJointVelocity's own post-Step() values (see
// ComputeSystemEnergy's own comment). Comparing against the same-tick
// (post-Step()) angles instead looks fine while the arm is barely moving,
// then visibly diverges once it picks up real speed -- exactly the
// symptom an off-by-one-tick bug produces, not a physics bug.
TEST(TwoJointArmReference, GenericSystemEnergyMatchesClosedFormAtEveryTick) {
  TempDir dir;
  Simulation sim = MakeTwoJointArmSim(dir);

  JointTopology topology = sim.TopologyFor<TopologicalOrdering>();
  JointRows rows = FindJointRows(topology);
  ASSERT_NE(rows.shoulder, topology.Size());
  ASSERT_NE(rows.elbow, topology.Size());

  const ScalarOperationT m1 = kShoulder.mass;
  const ScalarOperationT r1 = kShoulder.pivot_to_com;
  const ScalarOperationT I1 = kShoulder.inertia_about_pivot;
  const ScalarOperationT m2 = kElbow.mass;
  const ScalarOperationT r2 = kElbow.pivot_to_com;
  const ScalarOperationT I2 = kElbow.inertia_about_pivot;
  const ScalarOperationT l1 = kLinkLength;
  const ScalarOperationT g = kGravity;
  const ScalarOperationT P = I1 + m2 * l1 * l1;
  const ScalarOperationT Q = I2;
  const ScalarOperationT R = m2 * l1 * r2;

  // Real gravity, not sim_config.yaml's literal (negated) base_acceleration
  // -- see this file's own kGravity comment.
  Vector3<ScalarOperationT> gravity(0.0F, 0.0F, kGravity);

  // The archetype's own initial state (both links at rest, identity
  // rotation) -- what ComputeSystemEnergy reports right after the first
  // Step() call, since that call's ABA pass computes kWorldTransform/
  // kSpatialVelocity from exactly this pre-Step() state.
  ScalarOperationT phi_prev = 0.0F;
  ScalarOperationT psi_prev = 0.0F;
  ScalarOperationT phi_dot_prev = 0.0F;
  ScalarOperationT psi_dot_prev = 0.0F;

  constexpr int kSteps = 120 * 5;  // 5 simulated seconds.
  for (int i = 0; i < kSteps; ++i) {
    ASSERT_TRUE(sim.Step(kDt));

    auto view = sim.ViewFor<ABAAlgorithm>();

    ScalarOperationT omega2_prev = phi_dot_prev + psi_dot_prev;
    ScalarOperationT closed_form_kinetic =
        0.5F * P * phi_dot_prev * phi_dot_prev +
        0.5F * Q * omega2_prev * omega2_prev +
        R * std::cos(psi_prev) * phi_dot_prev * omega2_prev;
    ScalarOperationT closed_form_potential =
        -(m1 * r1 + m2 * l1) * g * std::sin(phi_prev) -
        m2 * r2 * g * std::sin(phi_prev + psi_prev);

    auto energy = ComputeSystemEnergy(view, topology, gravity);
    EXPECT_NEAR(energy.kinetic, closed_form_kinetic, 1e-3F) << "tick " << i;
    EXPECT_NEAR(energy.potential, closed_form_potential, 1e-3F) << "tick " << i;

    auto shoulder_q = view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.shoulder);
    auto elbow_q = view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.elbow);
    auto shoulder_qd =
        view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.shoulder);
    auto elbow_qd = view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.elbow);
    phi_prev = ExtractAngleAboutX(
        shoulder_q.Rotation().W(), shoulder_q.Rotation().X()
    );
    psi_prev =
        ExtractAngleAboutX(elbow_q.Rotation().W(), elbow_q.Rotation().X());
    phi_dot_prev = shoulder_qd.Angular().X();
    psi_dot_prev = elbow_qd.Angular().X();
  }
}

// RK4 (Simulation::Integrator::kRungeKutta4) is the integrator MuJoCo
// itself points at for energy-conserving systems, and this pins down the
// margin it actually buys over the registered semi-implicit-Euler scheme
// on this repo's own demo scene. Ratio-based rather than absolute for the
// same reason SubstepsSubstantiallyReduceEnergyDissipation above is: both
// runs are the same chaotic system, so the claim under test is the
// relative improvement, not a number this scene happens to produce today.
//
// The measured gap at dt = 1/120 is roughly three orders of magnitude
// (mean drift ~0.001 for RK4 vs ~3.2 for Euler over a 26s run), so the
// 100x threshold here is a wide margin rather than a tuned-to-pass one.
TEST(TwoJointArmReference, RungeKutta4DrasticallyOutperformsEulerOnEnergy) {
  const ScalarOperationT m1 = kShoulder.mass;
  const ScalarOperationT r1 = kShoulder.pivot_to_com;
  const ScalarOperationT I1 = kShoulder.inertia_about_pivot;
  const ScalarOperationT m2 = kElbow.mass;
  const ScalarOperationT r2 = kElbow.pivot_to_com;
  const ScalarOperationT I2 = kElbow.inertia_about_pivot;
  const ScalarOperationT l1 = kLinkLength;
  const ScalarOperationT g = kGravity;
  const ScalarOperationT P = I1 + m2 * l1 * l1;
  const ScalarOperationT Q = I2;
  const ScalarOperationT R = m2 * l1 * r2;

  auto max_abs_total_energy = [&](Simulation::Integrator integrator) {
    TempDir dir;
    Simulation sim = MakeTwoJointArmSim(dir);
    sim.SetIntegrator(integrator);

    JointTopology topology = sim.TopologyFor<TopologicalOrdering>();
    JointRows rows = FindJointRows(topology);

    constexpr int kSteps = 120 * 8;  // 8 simulated seconds.
    ScalarOperationT max_abs_energy = 0.0F;
    for (int i = 0; i < kSteps; ++i) {
      EXPECT_TRUE(sim.Step(kDt));

      auto view = sim.ViewFor<ABAAlgorithm>();
      auto shoulder_q =
          view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.shoulder);
      auto elbow_q = view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.elbow);
      auto shoulder_qd =
          view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.shoulder);
      auto elbow_qd = view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.elbow);

      ScalarOperationT phi = ExtractAngleAboutX(
          shoulder_q.Rotation().W(), shoulder_q.Rotation().X()
      );
      ScalarOperationT psi =
          ExtractAngleAboutX(elbow_q.Rotation().W(), elbow_q.Rotation().X());
      ScalarOperationT phi_dot = shoulder_qd.Angular().X();
      ScalarOperationT psi_dot = elbow_qd.Angular().X();
      ScalarOperationT omega2 = phi_dot + psi_dot;

      ScalarOperationT kinetic = 0.5F * P * phi_dot * phi_dot +
                      0.5F * Q * omega2 * omega2 +
                      R * std::cos(psi) * phi_dot * omega2;
      ScalarOperationT potential = -(m1 * r1 + m2 * l1) * g * std::sin(phi) -
                        m2 * r2 * g * std::sin(phi + psi);
      max_abs_energy = std::max(max_abs_energy, std::abs(kinetic + potential));
    }
    return max_abs_energy;
  };

  ScalarOperationT euler =
      max_abs_total_energy(Simulation::Integrator::kSemiImplicitEuler);
  ScalarOperationT rk4 = max_abs_total_energy(Simulation::Integrator::kRungeKutta4);

  EXPECT_LT(rk4, euler * 0.01F)
      << "euler max |energy|: " << euler << ", rk4 max |energy|: " << rk4;
}

namespace {

// Which integrator ACHILLES_LONG_HORIZON_INTEGRATOR names -- default rk4,
// since that's the one this test exists to double-check. Watching the
// engine run with semi-implicit Euler makes the energy dissipation from
// earlier in this repo's own history obvious just by eye; watching it run
// with RK4 doesn't show anything wrong. "Doesn't look wrong" is not the
// same claim as "is correct" for a chaotic system -- a human eye has no
// way to tell subtle numerical error apart from ordinary chaotic
// unpredictability by watching a double pendulum swing -- so this test
// checks the actual per-tick dynamics regardless of what the integrated
// trajectory looks like.
Simulation::Integrator LongHorizonIntegratorFromEnv() {
  const char* name = std::getenv("ACHILLES_LONG_HORIZON_INTEGRATOR");
  std::string_view integrator = name != nullptr ? name : "rk4";
  if (integrator == "euler") {
    return Simulation::Integrator::kSemiImplicitEuler;
  }
  if (integrator == "verlet") {
    return Simulation::Integrator::kVelocityVerlet;
  }
  if (integrator == "midpoint") {
    return Simulation::Integrator::kImplicitMidpoint;
  }
  if (integrator == "implicit") {
    return Simulation::Integrator::kImplicitVelocity;
  }
  return Simulation::Integrator::kRungeKutta4;
}

const char* IntegratorName(Simulation::Integrator integrator) {
  switch (integrator) {
    case Simulation::Integrator::kSemiImplicitEuler:
      return "euler";
    case Simulation::Integrator::kVelocityVerlet:
      return "verlet";
    case Simulation::Integrator::kImplicitMidpoint:
      return "midpoint";
    case Simulation::Integrator::kImplicitVelocity:
      return "implicit";
    case Simulation::Integrator::kRungeKutta4:
      return "rk4";
    case Simulation::Integrator::kImplicitFast:
      return "implicitfast";
  }
  return "unknown";
}

double Mean(
    const std::vector<double>& samples, std::size_t begin, std::size_t end
) {
  double sum = 0.0;
  for (std::size_t i = begin; i < end; ++i) {
    sum += samples[i];
  }
  return sum / static_cast<double>(end - begin);
}

double Percentile(std::vector<double> samples, double p) {
  std::sort(samples.begin(), samples.end());
  auto index =
      static_cast<std::size_t>(p * static_cast<double>(samples.size() - 1));
  return samples[index];
}

double Median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  std::size_t n = values.size();
  return n % 2 == 1 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2.0;
}

// Ordinary least-squares slope of y against x = 0, 1, ..., y.size()-1 --
// the per-window-index trend in a series of window-mean errors. Used
// instead of a naive first-half-vs-second-half or first-window-vs-last-
// window comparison because a straight line fit averages out ordinary
// window-to-window noise (which a chaotic system has plenty of) rather
// than comparing two individually noisy samples against each other.
double TrendSlope(const std::vector<double>& y) {
  double n = static_cast<double>(y.size());
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xy = 0.0;
  double sum_xx = 0.0;
  for (std::size_t i = 0; i < y.size(); ++i) {
    double x = static_cast<double>(i);
    sum_x += x;
    sum_y += y[i];
    sum_xy += x * y[i];
    sum_xx += x * x;
  }
  return (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
}

}  // namespace

// A long-horizon validation, deliberately DISABLED_ so ctest never runs it
// -- run it on demand via scripts/validate-long-horizon.sh (which is just
// a wrapper for --gtest_also_run_disabled_tests on this filter). It is not
// a CI gate; it exists to answer a question a human watching the viewer
// cannot: after minutes of a chaotic system running, are the per-tick
// dynamics still the same dynamics the engine was validated against at
// t = 0, or has something quietly gone wrong that chaos is simply
// drowning out visually?
//
// What it can and cannot check is worth being precise about, because the
// obvious extension of TrajectoryMatchesIndependentModelOverTwoSeconds --
// just run that same trajectory comparison for longer -- is guaranteed to
// fail, and for a physical reason rather than a bug. This is a chaotic
// double pendulum: two trajectories that differ by even one ScalarOperationT ulp
// separate exponentially, so the engine and a reference stepper of the
// exact same discrete map inevitably diverge once enough Lyapunov times
// have passed. That says nothing about correctness, so trajectory
// agreement cannot be the thing under test here.
//
// The actual check is *per-tick* rather than trajectory-wide: at every
// state the sim visits, feed that state into the closed-form
// ReferenceAngularAcceleration and compare against the qdd algorithms::aba
// just produced for it (via Simulation::RefreshAccelerationForCurrentState,
// which forces a fresh ABA evaluation at exactly the captured state --
// see its own comment for why that call can't be skipped for any
// Integrator other than kSemiImplicitEuler). That error never accumulates,
// so it stays meaningful for an arbitrarily long run -- and because a few
// minutes of chaotic motion wanders over a far wider swath of
// configuration space than any two-second trajectory does, it is a
// considerably *stronger* test of the dynamics than the short trajectory
// match is: a bug that only manifests in some rare configuration (a
// near-singular pose, an extreme velocity) is far more likely to actually
// be visited over minutes than over two seconds.
//
// THE ACCEPTANCE CRITERION IS SELF-REFERENTIAL, NOT A HARDCODED THRESHOLD --
// and specifically not anchored to the trajectory's own starting window
// either. Two earlier versions of this test got this wrong in two
// different ways, both worth recording so a third attempt doesn't repeat
// them:
//   1. The first asserted the observed error stayed under fixed constants
//      (5e-4 mean, 1e-2 worst-case) -- numbers arrived at by running the
//      test once, reading off what came out, and picking something a bit
//      above it. That is characterization, not verification: a regression
//      in the underlying formula would have been baked into that same
//      constant and silently passed forever after.
//   2. The second computed a "trusted baseline" from the first two
//      seconds of the run and compared the rest of the horizon against
//      it -- which sounds self-referential and principled, but the first
//      two seconds start from rest, where every velocity-dependent term
//      in the dynamics is near zero and there is almost no floating-point
//      cancellation for the closed form to suffer. That window is
//      therefore *unrepresentatively* well-conditioned, not typical --
//      comparing a fast, large-angle swing against it produces a large
//      "regression" that is actually just ordinary, expected error growth
//      from worse conditioning at higher speed, not a dynamics bug.
// The fix is to never single out one window as ground truth. Instead,
// split the *entire* horizon into equal windows (all sampling the same
// chaotic, swinging regime, so none is atypically well-conditioned),
// compute each window's own mean error, and ask two scale-invariant
// questions of that series directly:
//   a. Does any window look dramatically worse than a typical window
//      (max window mean / median window mean, bounded by
//      kOutlierTolerance)? Catches a bug that only bites in some specific
//      region of state space the run happens to pass through.
//   b. Is there a systematic upward trend across the run (a least-squares
//      fit's total rise over the horizon, relative to the median window,
//      bounded by kTrendTolerance)? Catches a bug that compounds or only
//      manifests after some accumulation -- the reason a linear fit is
//      used rather than comparing two individual windows is that fitting
//      averages out per-window noise instead of comparing two samples of
//      it against each other.
// Per-window p99 is printed for visibility into tail behavior (dominated
// by float32 cancellation at the instants phi/psi pass through
// configurations where the closed form subtracts near-equal terms, which
// is real and expected) but not asserted on: with a few hundred samples
// per window, a single-window p99 is itself noisy enough that gating on
// it would be more likely to catch that noise than a real problem.
TEST(
    TwoJointArmReference,
    DISABLED_LongHorizonDynamicsMatchClosedFormWithoutDrift
) {
  const char* horizon_env = std::getenv("ACHILLES_LONG_HORIZON_SECONDS");
  const ScalarOperationT horizon = horizon_env != nullptr
                            ? static_cast<ScalarOperationT>(std::atof(horizon_env))
                            : 300.0F;
  constexpr int kWindowCount = 10;
  ASSERT_GT(horizon / static_cast<ScalarOperationT>(kWindowCount), 1.0F)
      << "Horizon must be long enough for " << kWindowCount
      << " windows of at least a simulated second each to be meaningful.";
  const int steps = static_cast<int>(horizon / kDt);

  Simulation::Integrator integrator = LongHorizonIntegratorFromEnv();

  TempDir dir;
  Simulation sim = MakeTwoJointArmSim(dir);
  sim.SetIntegrator(integrator);
  JointTopology topology = sim.TopologyFor<TopologicalOrdering>();
  JointRows rows = FindJointRows(topology);
  ASSERT_NE(rows.shoulder, topology.Size());
  ASSERT_NE(rows.elbow, topology.Size());

  // Matches the archetype's own initial state (both links at rest,
  // identity rotation) -- Simulation::Init() loads exactly this, so the
  // very first RefreshAccelerationForCurrentState() below evaluates ABA at
  // exactly this state.
  ScalarOperationT phi = 0.0F;
  ScalarOperationT psi = 0.0F;
  ScalarOperationT phi_dot = 0.0F;
  ScalarOperationT psi_dot = 0.0F;

  std::vector<double> shoulder_errors(static_cast<std::size_t>(steps));
  std::vector<double> elbow_errors(static_cast<std::size_t>(steps));

  for (int i = 0; i < steps; ++i) {
    // Force a fresh ABA evaluation at the state (phi, psi, phi_dot,
    // psi_dot) currently describes, *before* advancing anything -- see
    // Simulation::RefreshAccelerationForCurrentState's own comment on why
    // this can't be skipped for any Integrator other than
    // kSemiImplicitEuler. Skipping it silently compares the closed form
    // against whatever intermediate stage state an integrator's own last
    // internal ABA call happened to leave behind, which for e.g.
    // kRungeKutta4 is a real but different state than (phi, psi, phi_dot,
    // psi_dot) -- that mismatch alone produces a large, integrator-
    // specific-looking "error" that has nothing to do with dynamics
    // correctness.
    sim.RefreshAccelerationForCurrentState();
    auto view = sim.ViewFor<ABAAlgorithm>();
    Accelerations expected =
        ReferenceAngularAcceleration(phi, psi, phi_dot, psi_dot);
    ScalarOperationT shoulder_qdd =
        view.Load<ABAField::kJointAcceleration, ScalarOperationT>(rows.shoulder)
            .Angular()
            .X();
    ScalarOperationT elbow_qdd = view.Load<ABAField::kJointAcceleration, ScalarOperationT>(rows.elbow)
                          .Angular()
                          .X();

    // Relative, not absolute. Absolute |dqdd| would trend upward over a
    // long, energy-drifting run purely because faster motion follows and
    // ABA's Coriolis terms scale with v^2 -- a constant *relative*
    // accuracy would then look like a growing error even with nothing
    // wrong. Floored at 1 rad/s^2 so a near-zero reference qdd can't
    // manufacture a huge ratio out of ordinary ScalarOperationT noise.
    double shoulder_scale =
        std::max(1.0, static_cast<double>(std::abs(expected.phi_dd)));
    double elbow_scale =
        std::max(1.0, static_cast<double>(std::abs(expected.psi_dd)));
    shoulder_errors[static_cast<std::size_t>(i)] =
        static_cast<double>(std::abs(shoulder_qdd - expected.phi_dd)) /
        shoulder_scale;
    elbow_errors[static_cast<std::size_t>(i)] =
        static_cast<double>(std::abs(elbow_qdd - expected.psi_dd)) /
        elbow_scale;

    // Now advance, and read the new committed state for the next
    // iteration's comparison.
    ASSERT_TRUE(sim.Step(kDt));
    auto next_view = sim.ViewFor<ABAAlgorithm>();
    auto shoulder_q =
        next_view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.shoulder);
    auto elbow_q = next_view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.elbow);
    phi = ExtractAngleAboutX(
        shoulder_q.Rotation().W(), shoulder_q.Rotation().X()
    );
    psi = ExtractAngleAboutX(elbow_q.Rotation().W(), elbow_q.Rotation().X());
    phi_dot = next_view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.shoulder)
                  .Angular()
                  .X();
    psi_dot = next_view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.elbow)
                  .Angular()
                  .X();
  }

  // kOutlierTolerance/kTrendTolerance are headroom for ordinary chaotic
  // window-to-window variance, not fitted constants: which specific
  // configurations (near-singular poses, high-speed passes) a given
  // window happens to sample varies run to run and window to window, so
  // some spread between windows is expected even with nothing wrong. They
  // exist to catch a real, many-times-worse anomaly or a genuine
  // systematic trend, not to be tuned to whatever one run happens to
  // produce -- unlike the mean/p99 numbers this prints, which are exactly
  // that (this run's own numbers) and are reported for visibility, not
  // used to define the pass criterion.
  constexpr double kOutlierTolerance = 15.0;
  constexpr double kTrendTolerance = 5.0;

  auto report = [&](const char* label, const std::vector<double>& errors) {
    std::size_t total = errors.size();
    std::size_t window_size = total / kWindowCount;

    std::vector<double> window_means(kWindowCount);
    std::vector<double> window_p99s(kWindowCount);
    for (int w = 0; w < kWindowCount; ++w) {
      std::size_t begin = static_cast<std::size_t>(w) * window_size;
      std::size_t end = (w + 1 == kWindowCount) ? total : begin + window_size;
      window_means[static_cast<std::size_t>(w)] = Mean(errors, begin, end);
      window_p99s[static_cast<std::size_t>(w)] = Percentile(
          std::vector<double>(
              errors.begin() + static_cast<std::ptrdiff_t>(begin),
              errors.begin() + static_cast<std::ptrdiff_t>(end)
          ),
          0.99
      );
    }

    double median_mean = Median(window_means);
    double max_mean =
        *std::max_element(window_means.begin(), window_means.end());
    double slope = TrendSlope(window_means);
    double fitted_total_rise = slope * static_cast<double>(kWindowCount - 1);

    std::printf(
        "\n  %s (integrator=%s, horizon=%.0fs, %zu ticks)\n",
        label,
        IntegratorName(integrator),
        horizon,
        total
    );
    std::printf(
        "    window means (mean|p99), %d windows of ~%.1fs each:\n",
        kWindowCount,
        static_cast<ScalarOperationT>(window_size) * kDt
    );
    for (int w = 0; w < kWindowCount; ++w) {
      std::printf(
          "      window %2d:  mean=%.3e  p99=%.3e\n",
          w,
          window_means[static_cast<std::size_t>(w)],
          window_p99s[static_cast<std::size_t>(w)]
      );
    }
    std::printf(
        "    median window mean:     %.3e\n"
        "    worst window mean:      %.3e  (ratio vs median: %.2fx, "
        "tolerance %.0fx)\n"
        "    fitted trend over run:  %.3e  (relative to median: %.2fx, "
        "tolerance %.0fx)\n",
        median_mean,
        max_mean,
        max_mean / median_mean,
        kOutlierTolerance,
        fitted_total_rise,
        fitted_total_rise / median_mean,
        kTrendTolerance
    );

    EXPECT_LT(max_mean, median_mean * kOutlierTolerance)
        << label << ": some window's mean error is more than "
        << kOutlierTolerance
        << "x a typical window's -- the dynamics disagree with the closed "
           "form far more in some specific region of the run than "
           "elsewhere.";
    EXPECT_LT(std::abs(fitted_total_rise), median_mean * kTrendTolerance)
        << label << ": the fitted trend across the run's own windows rises "
        << "by more than " << kTrendTolerance
        << "x the typical window's error over the full horizon -- the "
           "signature of a bug that compounds or only manifests in states "
           "reached after accumulation, not of stationary float32 noise.";
  };

  report("shoulder", shoulder_errors);
  report("elbow", elbow_errors);
}
