#pragma once

#include "algorithms/physics_algorithms.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "algorithms/sensing/truth_sensor.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::sensing {

// Every sensor kind the sim hosts, registered once here. All of them run in
// the one sensing algorithm (sensing_data.hpp): each kind's instances are
// one segment of its rows, packed at that kind's own record sizes. A kind is
// a type; what varies between instances of one kind (where it's mounted,
// its error model, its rate) is data, declared in the .arow as attachments
// under the kind's kListName. A kind with no instances costs one empty
// loop per pass, nothing more.
//
// Today that's one truth sensor per named field the physics can be sensed
// at (TruthSensorsOf), each attached under the field's own name.
using SensorKinds = TruthSensorsOf<PhysicsAlgorithms>;

// Every sensor's row: one per attachment the archetypes declare under any
// registered kind's kListName, kind by kind -- each kind's rows one segment
// -- each linked to its mount joint's row.
using SensingOrdering = engine::topology::
    ListOrdering<engine::topology::TopologicalOrdering, SensorKinds>;

// Every sim field any registered kind reads, once each.
using SensorInputs = util::UniqueT<
    util::ApplyT<util::ConcatT, util::TransformT<KindInputs, SensorKinds>>>;

// Every registered kind's state and output, as the sensing algorithm
// stores them (one record type per kind's segment).
using SensorStates = util::TransformT<KindState, SensorKinds>;
using SensorOutputs = util::TransformT<KindOutput, SensorKinds>;

}  // namespace achilles::algorithms::sensing
