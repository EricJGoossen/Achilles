#pragma once

#include "algorithms/attachment/attachment_step.hpp"
#include "algorithms/physics_algorithms.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms {

// Every Algorithm the engine hosts: the physics (physics_algorithms.hpp), then
// attachment::AttachmentAlgorithm (every registered sensor and actuator kind,
// attachment_kinds/attachment_kinds.hpp). SimAllocator (via
// engine::memory::SimAllocatorForT) picks this list up directly and carves
// every listed Algorithm's fields. Attachments come last: they name the
// physics' shared slots for their inputs, so it must already own them (the
// first Algorithm naming a SharedAs tag owns that block --
// SimAllocator::Carve).
using RegisteredAlgorithms = util::
    ConcatT<PhysicsAlgorithms, util::TypeList<attachment::AttachmentAlgorithm>>;

}  // namespace achilles::algorithms
