#pragma once

#include "algorithms/viz/viz_data.hpp"
#include "engine/algorithm_contract.hpp"

namespace achilles::algorithms::viz {

// VizAlgorithm has no Step of its own -- engine::NoStep, the same sentinel
// a field-pool-only Algorithm uses (see its own comment,
// engine/algorithm_contract.hpp). kWorldTransform is written by ABA (via
// the shared WorldTransformSlot); kVisualExtents/kVisualColor are written
// once, at load time, by SimAllocator's Populate pass from a joint's own
// archetype data. Nothing about drawing a frame belongs in
// engine::pass::Step's per-tick loop -- rendering is an ordinary read-only
// consumer of the already-stepped sim state (see interface::Simulation::
// ViewFor/TopologyFor), the same way a test inspects a View after Step()
// without itself being a Step.
using VizAlgorithm = engine::Algorithm<VizField, VizFieldTraits>;
static_assert(engine::AlgorithmLike<VizAlgorithm>);

}  // namespace achilles::algorithms::viz
