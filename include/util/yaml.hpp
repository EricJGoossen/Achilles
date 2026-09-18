#pragma once

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>

namespace achilles::util {

// NaN-sentinel probe: YAML::Node::as<T>(fallback) never throws, so a
// successful numeric parse is distinguished from "used the fallback" by
// checking for the fallback itself, rather than paying a throw/catch per
// scalar -- for a large archetype file, that's the difference between
// parsing being instant and noticeably slow. Shared between LoadArchetypes
// (archetype_loader.cpp, which layers its own true/false string coercion
// and a thrown ArchetypeLoadError on top of a failed parse) and
// LoadSimConfig (sim_config_loader.cpp, which just falls back to that
// field's own default) -- same trick, different T (double vs float) and
// different failure handling, so only this part is actually shared.
template <typename T>
bool TryParseNumber(const YAML::Node& node, T& out) {
  T value = node.as<T>(std::numeric_limits<T>::quiet_NaN());
  if (std::isnan(value)) {
    return false;
  }
  out = value;
  return true;
}

}  // namespace achilles::util
