#pragma once

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/broad_phase/broad_phase_step.hpp"
#include "algorithms/contact_solver/contact_solver_step.hpp"
#include "algorithms/joint_limit/joint_limit_step.hpp"
#include "algorithms/midpoint/midpoint_step.hpp"
#include "algorithms/narrow_phase/narrow_phase_step.hpp"
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
//
// broad_phase::BroadPhaseAlgorithm comes before
// narrow_phase::NarrowPhaseAlgorithm for the same reason: the narrow phase
// only reads what the broad phase owns (its shape, world shape pose and
// candidate bitmask, via their SharedAs slots). When each runs in a tick is
// the integrator lists' business (integrators.hpp).
using RegisteredAlgorithms = util::TypeList<
    aba::ABAAlgorithm,
    vi::VIAlgorithm,
    broad_phase::BroadPhaseAlgorithm,
    narrow_phase::NarrowPhaseAlgorithm,
    joint_limit::JointLimitAlgorithm,
    contact_solver::ContactSolverAlgorithm,
    pi::PIAlgorithm,
    verlet::VerletAlgorithm,
    midpoint::MidpointAlgorithm,
    rk4::RK4Algorithm,
    viz::VizAlgorithm>;

}  // namespace achilles::algorithms
