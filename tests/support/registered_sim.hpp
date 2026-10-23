#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "algorithms/conventions.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "interface/archetype_loader.hpp"
#include "temp_dir.hpp"

namespace achilles::test_support {

// Every hosted Algorithm, carved the way interface::Simulation carves them
// -- for a test about how algorithms share memory or step together, built
// from real .arow text through the real LoadArchetypes path.
using RegisteredSim =
    engine::memory::SimAllocatorForT<algorithms::RegisteredAlgorithms>;

// One revolute-Z joint body (subspace entry (2, 0), DOF 0), as one entry of
// an .arow `joints:` list. `inertia` is rigid_body_inertia's [m, h_x, h_y,
// h_z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]; `extra` is any further field lines,
// each already indented to field depth and newline-terminated (joint
// limits, collision fields, ...).
inline std::string RevoluteZJointArow(
    const std::string& name,
    const std::string& parent,
    const std::string& inertia,
    algorithms::ScalarOperationT q0,
    algorithms::ScalarOperationT qd0,
    const std::string& extra = ""
) {
  std::ostringstream out;
  out << "  - name: " << name << "\n";
  if (!parent.empty()) {
    out << "    parent: " << parent << "\n";
  }
  out << "    fields:\n"
         "      joint_subspace: [0,0,0,0,0,0,0,0,0,0,0,0,1,"
         "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]\n"
         "      joint_activation_mask: [1]\n"
         "      fixed_joint_transform: [0,0,0, 1,0,0,0]\n"
         "      rigid_body_inertia: "
      << inertia
      << "\n"
         "      joint_position: [0,0,0, "
      << std::cos(q0 * 0.5F) << ", 0,0, " << std::sin(q0 * 0.5F)
      << "]\n"
         "      joint_velocity: ["
      << qd0
      << ", 0,0,0,0,0]\n"
         "      joint_torque: [0,0,0,0,0,0]\n"
      << extra;
  return out.str();
}

// joint_limited/range/margin field lines for DOF 0 alone, for `extra`.
inline std::string Dof0LimitFields(
    algorithms::ScalarOperationT lower,
    algorithms::ScalarOperationT upper,
    algorithms::ScalarOperationT margin = 0.0F
) {
  std::ostringstream out;
  out << "      joint_limited: [1]\n"
         "      joint_range_lower: ["
      << lower
      << ", 0,0,0,0,0]\n"
         "      joint_range_upper: ["
      << upper
      << ", 0,0,0,0,0]\n"
         "      joint_limit_margin: ["
      << margin << "]\n";
  return out.str();
}

// Unit mass with its center 0.5 along X, Izz about the joint 0.26 (see
// algorithms_integrators.cpp's kPendulumInertia): under PendulumConfig's
// gravity, qdd = kPendulumK cos(theta) with no constraint.
inline const char* const kPendulumInertia =
    "[1, 0.5,0,0, 0.01,0.26,0.26, 0,0,0]";
inline constexpr double kPendulumIzz = 0.26;
inline constexpr double kPendulumK = 0.5 * 9.8 / kPendulumIzz;

// Gravity +9.8 along Y in the joint's frame (base_acceleration is minus
// gravity, see algorithms::SimConfig).
inline algorithms::SimConfig PendulumConfig() {
  using B = algorithms::MathematicalT;
  algorithms::SimConfig config;
  config.base_acceleration = algorithms::Acceleration(
      algorithms::Vector3::Zero(),
      algorithms::Vector3(B(0.0F), B(-9.8F), B(0.0F))
  );
  return config;
}

// One archetype named "scene" holding `joints` (RevoluteZJointArow
// entries, ...), written to `dir` and loaded through the real
// LoadArchetypes path.
inline RegisteredSim MakeRegisteredSim(
    const TempDir& dir, const std::string& joints
) {
  std::filesystem::path file =
      dir.Write("scene.arow", "archetype: scene\njoints:\n" + joints);
  std::vector<domain::Archetype> archetypes =
      interface::LoadArchetypes(file.string());
  return RegisteredSim(archetypes);
}

}  // namespace achilles::test_support
