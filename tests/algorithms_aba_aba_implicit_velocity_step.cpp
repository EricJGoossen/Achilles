// Validates the joint-space Jacobian J = d(qdd)/d(qd) that
// ImplicitVelocityStep builds (see its header comment) against an
// *independently derived* closed form, rather than against itself.
//
// The reference comes from the same two-link Lagrangian model
// tests/examples_two_joint_arm.cpp already derives and trusts. With
//   c_1 = -(2R sin(psi) phi' psi' + R sin(psi) psi'^2)
//   c_2 = +R sin(psi) phi'^2
// as the velocity-dependent generalized bias forces (read straight off
// that file's own b1/b2), the mass matrix M and D = d(c)/dv give
// J = -M^-1 D. This test builds the same two-link arm through the real
// engine, computes J both ways, and requires them to agree.
//
// This is the check that would have caught the earlier, wrong version of
// this module: it restricted D to a per-joint block diagonal, which makes
// J diagonal and (for 1-DOF joints) identically zero -- a result this
// reference model plainly contradicts, since D11/D12/D21 are all nonzero
// whenever the arm is actually moving.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_implicit_velocity_step.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/pi/pi_step.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "domain/joint_topology.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/archetype_loader.hpp"
#include "support/temp_dir.hpp"

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::SimConfig;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::algorithms::aba::ABAStep;
using achilles::algorithms::aba::ImplicitVelocityStep;
using achilles::domain::Archetype;
using achilles::domain::JointTopology;
using Allocator = achilles::engine::memory::SimAllocatorForT<
    achilles::algorithms::RegisteredAlgorithms>;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::LoadArchetypes;
using achilles::test_support::TempDir;

namespace {

using ScalarVelocity =
    achilles::domain::spatial::SpatialVelocity<ScalarOperationT>;
using ScalarAcceleration =
    achilles::domain::spatial::SpatialAcceleration<ScalarOperationT>;
using Vector6 = achilles::domain::math::Vector6<ScalarOperationT>;

// Mirrors examples/two_joint_arm.arow's own shoulder/elbow numbers, the
// same way tests/examples_two_joint_arm.cpp mirrors them -- see that file
// for where each comes from.
constexpr ScalarOperationT kM1 = 1.0F;
constexpr ScalarOperationT kR1 = 0.3F;
constexpr ScalarOperationT kI1 = 0.18083F;
constexpr ScalarOperationT kM2 = 0.4F;
constexpr ScalarOperationT kR2 = 0.375F;
constexpr ScalarOperationT kI2 = 0.07238F;
constexpr ScalarOperationT kLinkLength = 1.0F;

std::string TwoJointArmYaml(
    ScalarOperationT shoulder_qd, ScalarOperationT elbow_qd
) {
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
         "      joint_velocity: ["
      << shoulder_qd
      << ", 0, 0, 0, 0, 0]\n"
         "      joint_torque: [0, 0, 0, 0, 0, 0]\n"
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
         "      joint_velocity: ["
      << elbow_qd
      << ", 0, 0, 0, 0, 0]\n"
         "      joint_torque: [0, 0, 0, 0, 0, 0]\n";
  return out.str();
}

struct Rows {
  std::size_t shoulder;
  std::size_t elbow;
};

Rows FindRows(const JointTopology& topology) {
  std::size_t sentinel = topology.Size();
  std::size_t shoulder = sentinel;
  for (std::size_t row = 0; row < topology.Size(); ++row) {
    if (topology[row] == sentinel) {
      shoulder = row;
      break;
    }
  }
  std::size_t elbow = sentinel;
  for (std::size_t row = 0; row < topology.Size(); ++row) {
    if (row != shoulder && topology[row] == shoulder) {
      elbow = row;
      break;
    }
  }
  return {shoulder, elbow};
}

// J = -M^-1 D for the two-link arm, at joint angles (phi, psi) and rates
// (phi_dot, psi_dot) -- see this file's own header comment for where D
// comes from. Returned row-major as {J11, J12, J21, J22}.
std::array<ScalarOperationT, 4> ReferenceJacobian(
    ScalarOperationT psi, ScalarOperationT phi_dot, ScalarOperationT psi_dot
) {
  const ScalarOperationT P = kI1 + kM2 * kLinkLength * kLinkLength;
  const ScalarOperationT Q = kI2;
  const ScalarOperationT R = kM2 * kLinkLength * kR2;
  const ScalarOperationT cos_psi = std::cos(psi);
  const ScalarOperationT sin_psi = std::sin(psi);

  // Mass matrix (same M as the reference model's own m11..m22).
  const ScalarOperationT m11 = P + Q + 2.0F * R * cos_psi;
  const ScalarOperationT m12 = Q + R * cos_psi;
  const ScalarOperationT m22 = Q;
  const ScalarOperationT det = m11 * m22 - m12 * m12;

  // G = d(c)/dv, differentiating c_1/c_2 (this file's own header comment)
  // directly. Note the signs: c_1 is itself negative in phi'psi', so
  // dc_1/dphi' comes out negative.
  const ScalarOperationT g11 = -2.0F * R * sin_psi * psi_dot;
  const ScalarOperationT g12 = -2.0F * R * sin_psi * (phi_dot + psi_dot);
  const ScalarOperationT g21 = 2.0F * R * sin_psi * phi_dot;
  const ScalarOperationT g22 = 0.0F;

  // qdd = M^-1 (tau - c), so J = d(qdd)/dv = -M^-1 G, with
  // M^-1 = [m22, -m12; -m12, m11] / det.
  const ScalarOperationT inv11 = m22 / det;
  const ScalarOperationT inv12 = -m12 / det;
  const ScalarOperationT inv21 = -m12 / det;
  const ScalarOperationT inv22 = m11 / det;
  return {
      -(inv11 * g11 + inv12 * g21),
      -(inv11 * g12 + inv12 * g22),
      -(inv21 * g11 + inv22 * g21),
      -(inv21 * g12 + inv22 * g22)
  };
}

// The same finite-difference construction ImplicitVelocityStep::Step does
// internally, run here against a real ABA so the test compares the value
// the integrator actually uses.
std::array<ScalarOperationT, 4> EngineJacobian(
    Allocator& sim, const SimConfig& config, const Rows& rows
) {
  auto view = sim.ViewFor<ABAAlgorithm>();
  std::array<std::size_t, 2> dof_rows{rows.shoulder, rows.elbow};

  std::array<ScalarVelocity, 2> original{
      view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.shoulder),
      view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.elbow)
  };

  ABAStep::Step(sim.SimContext(), config, 0.0F);
  std::array<ScalarOperationT, 2> baseline{
      view.Load<ABAField::kJointAcceleration, ScalarOperationT>(rows.shoulder
      )[0],
      view.Load<ABAField::kJointAcceleration, ScalarOperationT>(rows.elbow)[0]
  };

  std::array<ScalarOperationT, 4> jacobian{};
  for (std::size_t j = 0; j < 2; ++j) {
    ScalarOperationT base = original[j][0];
    ScalarOperationT epsilon =
        ScalarOperationT{1e-3} * std::max(ScalarOperationT{1}, std::abs(base));
    view.Store<ABAField::kJointVelocity, ScalarOperationT>(
        dof_rows[j], ScalarVelocity(Vector6(base + epsilon, 0, 0, 0, 0, 0))
    );
    ABAStep::Step(sim.SimContext(), config, 0.0F);
    for (std::size_t i = 0; i < 2; ++i) {
      ScalarOperationT perturbed =
          view.Load<ABAField::kJointAcceleration, ScalarOperationT>(dof_rows[i]
          )[0];
      jacobian[i * 2 + j] = (perturbed - baseline[i]) / epsilon;
    }
    view.Store<ABAField::kJointVelocity, ScalarOperationT>(
        dof_rows[j], original[j]
    );
  }
  ABAStep::Step(sim.SimContext(), config, 0.0F);
  return jacobian;
}

}  // namespace

// psi = 0 here (both joints start at identity), so sin(psi) == 0 and the
// reference D -- and therefore J -- is exactly zero regardless of the
// rates. Worth pinning separately from the moving case below: it is the
// one configuration where the wrong (block-diagonal) implementation and
// the right one agree, and conflating the two is exactly how the earlier
// version looked plausible for as long as it did.
TEST(ImplicitVelocityStepJacobian, IsZeroAtTheStraightConfiguration) {
  TempDir dir;
  std::filesystem::path file =
      dir.Write("arm.arow", TwoJointArmYaml(0.9F, -0.5F));
  std::vector<Archetype> archetypes = LoadArchetypes(file);

  Allocator sim(archetypes);
  SimConfig config;
  JointTopology topology = sim.TopologyFor<TopologicalOrdering>();
  Rows rows = FindRows(topology);
  ASSERT_NE(rows.shoulder, topology.Size());
  ASSERT_NE(rows.elbow, topology.Size());

  std::array<ScalarOperationT, 4> engine = EngineJacobian(sim, config, rows);
  for (std::size_t k = 0; k < 4; ++k) {
    EXPECT_NEAR(engine[k], 0.0F, 1e-3F) << "entry " << k;
  }
}

// The real check: step the arm forward far enough that psi != 0, then
// require every entry of the engine's own finite-difference J to match the
// independently-derived closed form. A block-diagonal J (the earlier bug)
// fails this on both off-diagonal entries and on J11.
TEST(ImplicitVelocityStepJacobian, MatchesClosedFormOnceTheArmHasMoved) {
  TempDir dir;
  std::filesystem::path file =
      dir.Write("arm.arow", TwoJointArmYaml(0.0F, 0.0F));
  std::vector<Archetype> archetypes = LoadArchetypes(file);

  Allocator sim(archetypes);
  SimConfig config;
  config.base_acceleration = achilles::algorithms::Acceleration(
      achilles::algorithms::Vector3::Zero(),
      achilles::algorithms::Vector3(
          achilles::algorithms::MathematicalT(0.0F),
          achilles::algorithms::MathematicalT(0.0F),
          achilles::algorithms::MathematicalT(-9.8F)
      )
  );

  JointTopology topology = sim.TopologyFor<TopologicalOrdering>();
  Rows rows = FindRows(topology);
  ASSERT_NE(rows.shoulder, topology.Size());
  ASSERT_NE(rows.elbow, topology.Size());

  // Drive the arm away from psi == 0 with real ABA + integration steps, so
  // the state J gets evaluated at is a physically reachable one rather
  // than a hand-set pose.
  auto view = sim.ViewFor<ABAAlgorithm>();
  constexpr ScalarOperationT kDt = 1.0F / 120.0F;
  for (int i = 0; i < 60; ++i) {
    ABAStep::Step(sim.SimContext(), config, kDt);
    for (std::size_t row : {rows.shoulder, rows.elbow}) {
      ScalarVelocity qd =
          view.Load<ABAField::kJointVelocity, ScalarOperationT>(row);
      ScalarAcceleration qdd =
          view.Load<ABAField::kJointAcceleration, ScalarOperationT>(row);
      view.Store<ABAField::kJointVelocity, ScalarOperationT>(
          row, ScalarVelocity(Vector6(qd[0] + kDt * qdd[0], 0, 0, 0, 0, 0))
      );
    }
    achilles::algorithms::pi::PIStep::Step(sim.SimContext(), config, kDt);
  }

  // Read the state J should be evaluated at. psi is the elbow's own
  // relative angle; extracted the same way examples_two_joint_arm.cpp
  // does (pure-X rotation quaternion).
  auto elbow_q =
      view.Load<ABAField::kJointPosition, ScalarOperationT>(rows.elbow);
  ScalarOperationT psi =
      2.0F * std::atan2(elbow_q.Rotation().X(), elbow_q.Rotation().W());
  ScalarOperationT phi_dot =
      view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.shoulder)[0];
  ScalarOperationT psi_dot =
      view.Load<ABAField::kJointVelocity, ScalarOperationT>(rows.elbow)[0];
  ASSERT_GT(std::abs(std::sin(psi)), 0.05F)
      << "Test premise violated: arm never left the straight configuration.";

  std::array<ScalarOperationT, 4> engine = EngineJacobian(sim, config, rows);
  std::array<ScalarOperationT, 4> reference =
      ReferenceJacobian(psi, phi_dot, psi_dot);

  for (std::size_t k = 0; k < 4; ++k) {
    EXPECT_NEAR(engine[k], reference[k], 5e-2F)
        << "entry " << k << " (engine " << engine[k] << " vs reference "
        << reference[k] << ")";
  }
  // And specifically: this must not be a diagonal matrix, or the earlier
  // block-diagonal bug would pass.
  EXPECT_GT(std::abs(reference[1]), 0.05F);
  EXPECT_GT(std::abs(reference[2]), 0.05F);
}
