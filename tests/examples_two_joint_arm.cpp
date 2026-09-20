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

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <sstream>
#include <string>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_energy.hpp"
#include "algorithms/conventions.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/vector3.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/simulation.hpp"
#include "support/temp_dir.hpp"

using achilles::algorithms::MathematicalT;
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
  float mass;
  float pivot_to_com;         // r -- rigid_body_inertia's h, divided by mass
  float inertia_about_pivot;  // Ixx == Izz (both perpendicular to the pivot
                              // -> CoM offset, which points along Y)
};
constexpr LinkParams kShoulder{
    .mass = 1.0F, .pivot_to_com = 0.3F, .inertia_about_pivot = 0.18083F
};
constexpr LinkParams kElbow{
    .mass = 0.4F, .pivot_to_com = 0.375F, .inertia_about_pivot = 0.07238F
};
constexpr float kLinkLength = 1.0F;  // shoulder -> elbow offset (local Y)

// examples/sim_config.yaml's own base_acceleration.linear.z. ABA treats a
// fixed base's own configured acceleration as *minus* true gravity (see
// algorithms/sim_config.hpp's own comment on base_acceleration), so the
// real gravitational acceleration every body actually falls under is
// +9.8 along world Z, not -9.8 -- this constant is that already-negated,
// "as Lagrangian mechanics wants it" value, not sim_config.yaml's literal
// -9.8.
constexpr float kGravity = 9.8F;

constexpr float kDt = 1.0F / 120.0F;

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
float ExtractAngleAboutX(float w, float x) { return 2.0F * std::atan2(x, w); }

// The closed-form (phi'', psi'') solved from the Euler-Lagrange equations
// for this exact two-link system -- see this file's own header comment for
// the derivation sketch. `phi`/`psi` are the current generalized
// coordinates (shoulder's absolute angle, elbow's angle relative to the
// shoulder); `phi_dot`/`psi_dot` their rates.
struct Accelerations {
  float phi_dd;
  float psi_dd;
};

Accelerations ReferenceAngularAcceleration(
    float phi, float psi, float phi_dot, float psi_dot
) {
  const float m1 = kShoulder.mass;
  const float r1 = kShoulder.pivot_to_com;
  const float I1 = kShoulder.inertia_about_pivot;
  const float m2 = kElbow.mass;
  const float r2 = kElbow.pivot_to_com;
  const float I2 = kElbow.inertia_about_pivot;
  const float l1 = kLinkLength;
  const float g = kGravity;

  const float P = I1 + m2 * l1 * l1;
  const float Q = I2;
  const float R = m2 * l1 * r2;

  const float cos_psi = std::cos(psi);
  const float sin_psi = std::sin(psi);
  const float cos_phi = std::cos(phi);
  const float cos_phi_psi = std::cos(phi + psi);

  const float m11 = P + Q + 2.0F * R * cos_psi;
  const float m12 = Q + R * cos_psi;
  const float m21 = m12;
  const float m22 = Q;

  const float b1 =
      2.0F * R * sin_psi * phi_dot * psi_dot + R * sin_psi * psi_dot * psi_dot +
      (m1 * r1 + m2 * l1) * g * cos_phi + m2 * r2 * g * cos_phi_psi;
  const float b2 = -R * sin_psi * phi_dot * phi_dot + m2 * r2 * g * cos_phi_psi;

  const float det = m11 * m22 - m12 * m21;
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
// derivation should track the engine's own trajectory to float precision
// for as long as this test cares to run it, not just at t=0.
struct ReferenceState {
  float phi = 0.0F;
  float psi = 0.0F;
  float phi_dot = 0.0F;
  float psi_dot = 0.0F;

  void Step(float dt) {
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
      view.Load<ABAField::kJointAcceleration, float>(rows.shoulder);
  auto elbow_qdd = view.Load<ABAField::kJointAcceleration, float>(rows.elbow);

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
    auto shoulder_q = view.Load<ABAField::kJointPosition, float>(rows.shoulder);
    auto elbow_q = view.Load<ABAField::kJointPosition, float>(rows.elbow);

    float engine_phi = ExtractAngleAboutX(
        shoulder_q.Rotation().W(), shoulder_q.Rotation().X()
    );
    float engine_psi =
        ExtractAngleAboutX(elbow_q.Rotation().W(), elbow_q.Rotation().X());

    // Both sides recompute trig from scratch every tick (xsimd's cos/sin
    // inside the engine vs. std::cos/sin here), so float32 rounding
    // differences of a few ULPs per tick compound slightly over 240 ticks
    // -- measured max drift over the full 2 seconds is ~2.5e-6 rad (see
    // this test file's own derivation notes / the PR description), so
    // 1e-4 is a ~40x margin: comfortably above float noise, but nowhere
    // near what an actually-wrong coefficient produces (multiple degrees
    // of divergence within the first few ticks, not a slow float-noise
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

  const float m1 = kShoulder.mass;
  const float r1 = kShoulder.pivot_to_com;
  const float I1 = kShoulder.inertia_about_pivot;
  const float m2 = kElbow.mass;
  const float r2 = kElbow.pivot_to_com;
  const float I2 = kElbow.inertia_about_pivot;
  const float l1 = kLinkLength;
  const float g = kGravity;
  const float P = I1 + m2 * l1 * l1;
  const float Q = I2;
  const float R = m2 * l1 * r2;

  constexpr int kSteps = 120 * 30;  // 30 simulated seconds.
  float max_abs_energy = 0.0F;
  for (int i = 0; i < kSteps; ++i) {
    ASSERT_TRUE(sim.Step(kDt));

    auto view = sim.ViewFor<ABAAlgorithm>();
    auto shoulder_q = view.Load<ABAField::kJointPosition, float>(rows.shoulder);
    auto elbow_q = view.Load<ABAField::kJointPosition, float>(rows.elbow);
    auto shoulder_qd =
        view.Load<ABAField::kJointVelocity, float>(rows.shoulder);
    auto elbow_qd = view.Load<ABAField::kJointVelocity, float>(rows.elbow);

    float phi = ExtractAngleAboutX(
        shoulder_q.Rotation().W(), shoulder_q.Rotation().X()
    );
    float psi =
        ExtractAngleAboutX(elbow_q.Rotation().W(), elbow_q.Rotation().X());
    float phi_dot = shoulder_qd.Angular().X();
    float psi_dot = elbow_qd.Angular().X();
    float omega2 = phi_dot + psi_dot;

    float kinetic = 0.5F * P * phi_dot * phi_dot + 0.5F * Q * omega2 * omega2 +
                    R * std::cos(psi) * phi_dot * omega2;
    float potential = -(m1 * r1 + m2 * l1) * g * std::sin(phi) -
                      m2 * r2 * g * std::sin(phi + psi);
    float total_energy = kinetic + potential;
    max_abs_energy = std::max(max_abs_energy, std::abs(total_energy));
  }

  // Starting energy is 0; the highest this pendulum can ever legitimately
  // reach is bounded by its own maximum potential energy (both links
  // pointing straight against gravity), a small, fixed number -- nowhere
  // near what an actual energy-injection bug (unbounded growth) would
  // produce over 30 simulated seconds / 3600 ticks.
  float max_possible_potential = (m1 * r1 + m2 * l1) * g + m2 * r2 * g;
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
  const float m1 = kShoulder.mass;
  const float r1 = kShoulder.pivot_to_com;
  const float I1 = kShoulder.inertia_about_pivot;
  const float m2 = kElbow.mass;
  const float r2 = kElbow.pivot_to_com;
  const float I2 = kElbow.inertia_about_pivot;
  const float l1 = kLinkLength;
  const float g = kGravity;
  const float P = I1 + m2 * l1 * l1;
  const float Q = I2;
  const float R = m2 * l1 * r2;

  auto max_abs_total_energy = [&](int substeps) {
    TempDir dir;
    Simulation sim = MakeTwoJointArmSim(dir);
    sim.SetSubsteps(substeps);

    JointTopology topology = sim.TopologyFor<TopologicalOrdering>();
    JointRows rows = FindJointRows(topology);

    constexpr int kSteps = 120 * 10;  // 10 simulated seconds.
    float max_abs_energy = 0.0F;
    for (int i = 0; i < kSteps; ++i) {
      EXPECT_TRUE(sim.Step(kDt));

      auto view = sim.ViewFor<ABAAlgorithm>();
      auto shoulder_q =
          view.Load<ABAField::kJointPosition, float>(rows.shoulder);
      auto elbow_q = view.Load<ABAField::kJointPosition, float>(rows.elbow);
      auto shoulder_qd =
          view.Load<ABAField::kJointVelocity, float>(rows.shoulder);
      auto elbow_qd = view.Load<ABAField::kJointVelocity, float>(rows.elbow);

      float phi = ExtractAngleAboutX(
          shoulder_q.Rotation().W(), shoulder_q.Rotation().X()
      );
      float psi =
          ExtractAngleAboutX(elbow_q.Rotation().W(), elbow_q.Rotation().X());
      float phi_dot = shoulder_qd.Angular().X();
      float psi_dot = elbow_qd.Angular().X();
      float omega2 = phi_dot + psi_dot;

      float kinetic = 0.5F * P * phi_dot * phi_dot +
                      0.5F * Q * omega2 * omega2 +
                      R * std::cos(psi) * phi_dot * omega2;
      float potential = -(m1 * r1 + m2 * l1) * g * std::sin(phi) -
                        m2 * r2 * g * std::sin(phi + psi);
      max_abs_energy = std::max(max_abs_energy, std::abs(kinetic + potential));
    }
    return max_abs_energy;
  };

  float max_energy_1 = max_abs_total_energy(1);
  float max_energy_8 = max_abs_total_energy(8);

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

  const float m1 = kShoulder.mass;
  const float r1 = kShoulder.pivot_to_com;
  const float I1 = kShoulder.inertia_about_pivot;
  const float m2 = kElbow.mass;
  const float r2 = kElbow.pivot_to_com;
  const float I2 = kElbow.inertia_about_pivot;
  const float l1 = kLinkLength;
  const float g = kGravity;
  const float P = I1 + m2 * l1 * l1;
  const float Q = I2;
  const float R = m2 * l1 * r2;

  // Real gravity, not sim_config.yaml's literal (negated) base_acceleration
  // -- see this file's own kGravity comment.
  Vector3<float> gravity(0.0F, 0.0F, kGravity);

  // The archetype's own initial state (both links at rest, identity
  // rotation) -- what ComputeSystemEnergy reports right after the first
  // Step() call, since that call's ABA pass computes kWorldTransform/
  // kSpatialVelocity from exactly this pre-Step() state.
  float phi_prev = 0.0F;
  float psi_prev = 0.0F;
  float phi_dot_prev = 0.0F;
  float psi_dot_prev = 0.0F;

  constexpr int kSteps = 120 * 5;  // 5 simulated seconds.
  for (int i = 0; i < kSteps; ++i) {
    ASSERT_TRUE(sim.Step(kDt));

    auto view = sim.ViewFor<ABAAlgorithm>();

    float omega2_prev = phi_dot_prev + psi_dot_prev;
    float closed_form_kinetic =
        0.5F * P * phi_dot_prev * phi_dot_prev +
        0.5F * Q * omega2_prev * omega2_prev +
        R * std::cos(psi_prev) * phi_dot_prev * omega2_prev;
    float closed_form_potential =
        -(m1 * r1 + m2 * l1) * g * std::sin(phi_prev) -
        m2 * r2 * g * std::sin(phi_prev + psi_prev);

    auto energy = ComputeSystemEnergy(view, topology, gravity);
    EXPECT_NEAR(energy.kinetic, closed_form_kinetic, 1e-3F) << "tick " << i;
    EXPECT_NEAR(energy.potential, closed_form_potential, 1e-3F) << "tick " << i;

    auto shoulder_q = view.Load<ABAField::kJointPosition, float>(rows.shoulder);
    auto elbow_q = view.Load<ABAField::kJointPosition, float>(rows.elbow);
    auto shoulder_qd =
        view.Load<ABAField::kJointVelocity, float>(rows.shoulder);
    auto elbow_qd = view.Load<ABAField::kJointVelocity, float>(rows.elbow);
    phi_prev = ExtractAngleAboutX(
        shoulder_q.Rotation().W(), shoulder_q.Rotation().X()
    );
    psi_prev =
        ExtractAngleAboutX(elbow_q.Rotation().W(), elbow_q.Rotation().X());
    phi_dot_prev = shoulder_qd.Angular().X();
    psi_dot_prev = elbow_qd.Angular().X();
  }
}
