#pragma once

#include <memory>

#include "control/plant_state.hpp"
#include "math/distributions/i_distribution.hpp"
#include "math/drift_processes/i_drift.hpp"
#include "math/filters/i_filter.hpp"

namespace achilles::control::sensors {

class ISensor {
  public:
    ISensor() = default;
    ISensor(const ISensor&) = delete;
    ISensor(ISensor&&) = default;
    ISensor& operator=(const ISensor&) = delete;
    ISensor& operator=(ISensor&&) = default;
    virtual ~ISensor() = default;

    virtual void Update(const PlantState& plant_state, double dt) = 0;
};
}  // namespace achilles::control::sensors