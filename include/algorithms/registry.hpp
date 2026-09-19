#pragma once

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/vi/vi_step.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms {

// Every Algorithm the engine hosts, registered once here. SimAllocator
// (via engine::memory::SimAllocatorForT) picks this list up directly, and
// engine::pass::Step's own Algorithms... pack is deduced from whichever
// SimAllocator it's given -- so nothing else ever has to restate this pack.
// Adding a new algorithm to the sim is exactly one line here.
using RegisteredAlgorithms = util::TypeList<aba::ABAAlgorithm, vi::VIAlgorithm>;

}  // namespace achilles::algorithms
