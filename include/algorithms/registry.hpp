#pragma once

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/pi/pi_step.hpp"
#include "algorithms/vi/vi_step.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms {

// Every Algorithm the engine hosts, registered once here. SimAllocator
// (via engine::memory::SimAllocatorForT) picks this list up directly, and
// engine::pass::Step's own Algorithms... pack is deduced from whichever
// SimAllocator it's given -- so nothing else ever has to restate this pack.
// Adding a new algorithm to the sim is exactly one line here.
//
// Order matters: SimContext::Step runs this pack left to right
// (sim_context.hpp), and each algorithm's own fields are SharedAs-aliased
// onto the next one's inputs (see shared_slots.hpp) -- ABA solves for
// kJointAcceleration from the current kJointPosition/kJointVelocity, VI
// integrates that into kJointVelocity, and PI integrates the now-updated
// kJointVelocity into kJointPosition for ABA's next tick to read. Running
// VI/PI before ABA in a given tick would integrate stale acceleration/
// velocity instead of this tick's own.
using RegisteredAlgorithms =
    util::TypeList<aba::ABAAlgorithm, vi::VIAlgorithm, pi::PIAlgorithm>;

}  // namespace achilles::algorithms
