#pragma once

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/midpoint/midpoint_step.hpp"
#include "algorithms/pi/pi_step.hpp"
#include "algorithms/rk4/rk4_step.hpp"
#include "algorithms/verlet/verlet_step.hpp"
#include "algorithms/vi/vi_step.hpp"
#include "algorithms/viz/viz_step.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms {

// Every Algorithm the engine hosts, registered once here. SimAllocator
// (via engine::memory::SimAllocatorForT) picks this list up directly and
// carves every listed Algorithm's fields -- so nothing else ever has to
// restate this pack. Adding a new algorithm to the sim is one line here.
//
// Hosting is not stepping: this list says what memory exists, and the
// integrator lists in integrators.hpp say what one tick runs
// (SimContext::Step<List>).
// Every integrator is hosted side by side -- each one's fields are
// SharedAs-aliased onto ABA's own q/qd/qdd (see shared_slots.hpp), plus
// whatever private per-tick scratch it needs -- so switching integrators at
// runtime is just stepping a different list over the same state.
//
// ABA is first because the first Algorithm naming a SharedAs tag owns that
// block (SimAllocator::Carve). viz::VizAlgorithm has no Step: it only ever
// supplies memory (its kWorldTransform aliases ABA's, via
// WorldTransformSlot) for a renderer to read after a real Step() call.
using RegisteredAlgorithms = util::TypeList<
    aba::ABAAlgorithm,
    vi::VIAlgorithm,
    pi::PIAlgorithm,
    verlet::VerletAlgorithm,
    midpoint::MidpointAlgorithm,
    rk4::RK4Algorithm,
    viz::VizAlgorithm>;

}  // namespace achilles::algorithms
