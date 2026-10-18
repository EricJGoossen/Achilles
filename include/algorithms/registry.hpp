#pragma once

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/broad_phase/broad_phase_step.hpp"
#include "algorithms/narrow_phase/narrow_phase_step.hpp"
#include "algorithms/pi/pi_step.hpp"
#include "algorithms/vi/vi_step.hpp"
#include "algorithms/viz/viz_step.hpp"
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
//
// broad_phase::BroadPhaseAlgorithm and then narrow_phase::NarrowPhaseAlgorithm
// sit between VI and PI. Both read only kWorldTransform (via
// WorldTransformSlot), which only ABA writes and VI/PI never touch -- so
// they see this tick's pose wherever they sit after ABA -- and the narrow
// phase reads the broad phase's own results (via its SharedAs slots), so it
// runs second. They sit before PI, not after it, so that this tick's
// contacts exist while VI's updated kJointVelocity hasn't yet been
// integrated into kJointPosition: a contact solver correcting that velocity
// slots in right after the narrow phase, and PI then integrates the
// corrected velocity instead of one that ignores the contacts.
//
// viz::VizAlgorithm is last, and Step-order-irrelevant: it names
// engine::NoStep (viz_step.hpp), so SimContext::Step skips it entirely --
// it only ever supplies memory (its own kWorldTransform aliases ABA's,
// via WorldTransformSlot) for a renderer to read after a real Step() call,
// never something this pack itself steps.
using RegisteredAlgorithms = util::TypeList<
    aba::ABAAlgorithm,
    vi::VIAlgorithm,
    broad_phase::BroadPhaseAlgorithm,
    narrow_phase::NarrowPhaseAlgorithm,
    pi::PIAlgorithm,
    viz::VizAlgorithm>;

}  // namespace achilles::algorithms
