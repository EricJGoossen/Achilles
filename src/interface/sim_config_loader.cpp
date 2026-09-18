#include "interface/sim_config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <filesystem>

#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "util/yaml.hpp"

namespace achilles::interface {

namespace {

using algorithms::Acceleration;
using algorithms::BatchOperationT;
using algorithms::Quaternion;
using algorithms::SimConfig;
using algorithms::Transform;
using algorithms::Vector3;
using algorithms::Velocity;

// Falls back wholesale, not component-by-component, if `node` isn't a
// plain N-element numeric sequence -- SimConfig's fields are small,
// all-or-nothing vectors/quaternions, so a partially-specified one is
// treated the same as an absent one. Keeps this loader flat: no recursion,
// just one fixed-size pass per field.
//
// IsDefined() must be checked before anything else: a Node produced by
// indexing a missing map key (e.g. node["rotation"] when "rotation" isn't
// present) reports IsDefined() == false safely, but calling almost any
// other query on it -- IsSequence(), size(), even operator[] again --
// throws YAML::InvalidNode instead of just answering "no". Since every
// field in this file is optional, that would turn "the user only wrote
// some of it" into a crash instead of a per-field fallback.
template <std::size_t N>
bool TryParseFloats(const YAML::Node& node, std::array<float, N>& out) {
  if (!node.IsDefined() || !node.IsSequence() || node.size() != N) {
    return false;
  }
  for (std::size_t i = 0; i < N; ++i) {
    if (!util::TryParseNumber(node[i], out[i])) {
      return false;
    }
  }
  return true;
}

Vector3 ParseVector3(const YAML::Node& node, const Vector3& fallback) {
  std::array<float, 3> v{};
  if (!TryParseFloats(node, v)) {
    return fallback;
  }
  return {BatchOperationT(v[0]), BatchOperationT(v[1]), BatchOperationT(v[2])};
}

Quaternion ParseQuaternion(const YAML::Node& node, const Quaternion& fallback) {
  std::array<float, 4> q{};
  if (!TryParseFloats(node, q)) {
    return fallback;
  }
  return {
      BatchOperationT(q[0]),
      BatchOperationT(q[1]),
      BatchOperationT(q[2]),
      BatchOperationT(q[3])
  };
}

// Guards IsDefined() before indexing `node` at all -- unlike TryParseFloats
// above (which only ever reads a node it was handed), this one calls
// node["translation"]/node["rotation"] itself. Indexing into a node that's
// already invalid (e.g. world_base_transform absent entirely, so `node`
// itself came from a failed lookup one level up) throws immediately at the
// `[]` call, before TryParseFloats ever gets a chance to say no.
Transform ParseTransform(const YAML::Node& node, const Transform& fallback) {
  if (!node.IsDefined()) {
    return fallback;
  }
  return {
      ParseVector3(node["translation"], fallback.Translation()),
      ParseQuaternion(node["rotation"], fallback.Rotation())
  };
}

Velocity ParseVelocity(const YAML::Node& node, const Velocity& fallback) {
  if (!node.IsDefined()) {
    return fallback;
  }
  return {
      ParseVector3(node["angular"], fallback.Angular()),
      ParseVector3(node["linear"], fallback.Linear())
  };
}

Acceleration ParseAcceleration(
    const YAML::Node& node, const Acceleration& fallback
) {
  if (!node.IsDefined()) {
    return fallback;
  }
  return {
      ParseVector3(node["angular"], fallback.Angular()),
      ParseVector3(node["linear"], fallback.Linear())
  };
}

}  // namespace

algorithms::SimConfig LoadSimConfig(const std::filesystem::path& path) {
  SimConfig config = SimConfig::Default();

  YAML::Node root;
  try {
    root = YAML::LoadFile(path.string());
    if (!root.IsMap()) {
      return SimConfig::Default();
    }
    config.world_base_transform = ParseTransform(
        root["world_base_transform"], config.world_base_transform
    );
    config.base_velocity =
        ParseVelocity(root["base_velocity"], config.base_velocity);
    config.base_acceleration =
        ParseAcceleration(root["base_acceleration"], config.base_acceleration);
  } catch (const YAML::Exception&) {
    return SimConfig::Default();
  }

  return config;
}

}  // namespace achilles::interface
