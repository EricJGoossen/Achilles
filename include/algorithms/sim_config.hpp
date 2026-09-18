#pragma once

#include "algorithms/conventions.hpp"

namespace achilles::algorithms {

// The whole-simulation, caller-supplied config every hosted Algorithm's Step
// may read (see engine::pass::SimContext::Step/engine::pass::Step, both
// generic over whatever Config type a caller passes -- this is just the one
// this codebase actually uses). One concrete struct for the whole sim, not
// a per-algorithm/templated shape: today only ABA reads
// world_base_transform/base_velocity/base_acceleration (its base-row seed
// state -- base_acceleration is typically -gravity for the common
// fixed-base case), but adding an algorithm that needs its own static
// config means adding a field here, the same "new field, not new plumbing"
// shape Traits<F> already has for a field's own per-tick data.
//
// Lives here, not engine::pass, so engine::pass never has to name
// algorithms::Transform/Velocity/Acceleration -- the generic engine layer
// stays generic, and only algorithms (and whatever composes it, e.g.
// interface::Simulation) needs to know this struct's shape.
struct SimConfig {
  Transform world_base_transform = Transform::Identity();
  Velocity base_velocity = Velocity::Zero();
  Acceleration base_acceleration = Acceleration::Zero();

  // Same values as a default-constructed SimConfig -- named, the same way
  // Transform::Identity()/Velocity::Zero() are, so interface::LoadSimConfig
  // (interface/sim_config_loader.hpp) has an explicit thing to fall back
  // to (a missing file, unparsable YAML, or one bad field) rather than
  // relying on an implicit {}.
  static SimConfig Default() { return SimConfig{}; }
};

}  // namespace achilles::algorithms
