#pragma once

#include <filesystem>

#include "algorithms/sim_config.hpp"

namespace achilles::interface {

// Loads algorithms::SimConfig from a flat YAML file -- unlike
// interface::LoadArchetypes (ArchetypeLoadError's strict, recursive
// `includes`-following loader), this never throws: a missing file,
// unparsable YAML, or a malformed/absent individual field all just fall
// back to SimConfig::Default(), field by field. Adding a new SimConfig
// field means adding one more `config.new_field = ParseX(...)` line in
// the .cpp, nothing else -- the same "new field, not new plumbing" shape
// SimConfig itself documents.
//
// Expected shape (every key, and every key within it, optional):
//   world_base_transform:
//     translation: [x, y, z]
//     rotation: [w, x, y, z]
//   base_velocity:
//     angular: [x, y, z]
//     linear: [x, y, z]
//   base_acceleration:
//     angular: [x, y, z]
//     linear: [x, y, z]
algorithms::SimConfig LoadSimConfig(const std::filesystem::path& path);

}  // namespace achilles::interface
