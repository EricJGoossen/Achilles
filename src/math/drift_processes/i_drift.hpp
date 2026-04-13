#pragma once

#include "math/distributions/i_distribution.hpp"

namespace achilles::math::drift_processes {
class IDrift {
  public:
    IDrift() = default;
    IDrift(const IDrift&) = default;
    IDrift(IDrift&&) = default;
    IDrift& operator=(const IDrift&) = default;
    IDrift& operator=(IDrift&&) = default;
    virtual ~IDrift() = default;

    virtual double sample(double dt) = 0;
};

}  // namespace achilles::math::drift_processes