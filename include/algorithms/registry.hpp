#pragma once

#include "algorithms/physics_algorithms.hpp"
#include "algorithms/sensing/sensing_step.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms {

// Every Algorithm the engine hosts: the physics (physics_algorithms.hpp),
// then sensing::SensingAlgorithm (every registered sensor kind, sensing/
// sensor_kinds.hpp). SimAllocator (via engine::memory::SimAllocatorForT)
// picks this list up directly and carves every listed Algorithm's fields.
// Sensing comes last: it names the physics' shared slots for its inputs, so
// they must already own them (the first Algorithm naming a SharedAs tag
// owns that block -- SimAllocator::Carve).
using RegisteredAlgorithms =
    util::ConcatT<PhysicsAlgorithms, util::TypeList<sensing::SensingAlgorithm>>;

}  // namespace achilles::algorithms
