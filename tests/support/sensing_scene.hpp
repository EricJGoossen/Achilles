#pragma once

#include <array>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

namespace achilles::test_support {

// One joint of a hand-written .arow scene, for the sensing tests: every
// field the registered algorithms read, with defaults for a massive,
// welded, shapeless body at its parent's origin. Set what a test needs.
struct SceneJoint {
  std::string name;
  std::string parent = "null";
  // Which subspace column 0 entry (row 0..5: roll, pitch, yaw, x, y, z) is
  // this joint's one degree of freedom; -1 welds it, 6 makes it free.
  int dof = -1;
  std::array<float, 3> translation = {0, 0, 0};
  std::array<float, 4> rotation = {1, 0, 0, 0};
  std::array<float, 6> velocity = {0, 0, 0, 0, 0, 0};
  std::array<float, 3> extents = {0, 0, 0};
  float radius = 0.0F;
  int group = 0;
  int mask = 0;
};

inline std::string List(const auto& values) {
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    out << (i == 0 ? "" : ", ") << values[i];
  }
  out << ']';
  return out.str();
}

inline std::string Subspace(int dof) {
  std::array<int, 36> s{};
  if (dof == 6) {
    for (std::size_t i = 0; i < 6; ++i) {
      s[(i * 6) + i] = 1;
    }
  } else if (dof >= 0) {
    s[static_cast<std::size_t>(dof) * 6] = 1;
  }
  return List(s);
}

inline std::string ActivationMask(int dof) {
  if (dof == 6) {
    return "[63]";
  }
  return dof >= 0 ? "[1]" : "[0]";
}

// An archetype of `joints`, with `includes` and, unless it's the root,
// attached to `attach_archetype`'s `attach_joint`.
inline std::string Archetype(
    const std::string& name,
    const std::vector<SceneJoint>& joints,
    const std::vector<std::string>& includes = {},
    const std::string& attach_archetype = "",
    const std::string& attach_joint = ""
) {
  std::ostringstream out;
  out << "archetype: " << name << '\n';
  if (!attach_archetype.empty()) {
    out << "attach: {archetype: " << attach_archetype
        << ", joint: " << attach_joint << "}\n";
  }
  if (!includes.empty()) {
    out << "includes:\n";
    for (const std::string& include : includes) {
      out << "  - " << include << '\n';
    }
  }
  out << "joints:\n";
  for (const SceneJoint& joint : joints) {
    out << "  - name: " << joint.name << "\n    parent: " << joint.parent
        << "\n    fields:\n"
        << "      joint_subspace: " << Subspace(joint.dof) << '\n'
        << "      joint_activation_mask: " << ActivationMask(joint.dof) << '\n'
        << "      fixed_joint_transform: {translation: "
        << List(joint.translation) << ", rotation: " << List(joint.rotation)
        << "}\n"
        << "      rigid_body_inertia: [1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0]\n"
        << "      joint_position: {translation: [0, 0, 0], rotation: [1, 0, "
           "0, 0]}\n"
        << "      joint_velocity: " << List(joint.velocity) << '\n'
        << "      collision_pose: {translation: [0, 0, 0], rotation: [1, 0, "
           "0, 0]}\n"
        << "      collision_extents: " << List(joint.extents) << '\n'
        << "      collision_radius: [" << joint.radius << "]\n"
        << "      collision_margin: [0.005]\n"
        << "      collision_group: [" << joint.group << "]\n"
        << "      collision_mask: [" << joint.mask << "]\n";
  }
  return out.str();
}

// A sensor file: `config` is the body of its config: block, each line
// "key: value".
inline std::string SensorFile(
    const std::string& name,
    const std::string& kind,
    const std::string& archetype,
    const std::string& joint,
    const std::vector<std::string>& config = {}
) {
  std::ostringstream out;
  out << "sensor: " << name << "\nkind: " << kind
      << "\nattach: {archetype: " << archetype << ", joint: " << joint << "}\n";
  if (!config.empty()) {
    out << "config:\n";
    for (const std::string& line : config) {
      out << "  " << line << '\n';
    }
  }
  return out.str();
}

// Gravity along -z.
inline constexpr const char* kZUpConfig =
    "base_acceleration:\n  angular: [0, 0, 0]\n  linear: [0, 0, 9.81]\n";

}  // namespace achilles::test_support
