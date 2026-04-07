#pragma once

#include "control/actuators/actuator.hpp"
#include "math/vector.hpp"

namespace achilles::control::actuators {

class LinearActuator : public Actuator {
  public:
    LinearActuator(
        const math::Vector& dof, dynamics::joints::AbstractJoint& joint
    )
      : Actuator({dof, math::Vector::zero()}, joint) {}
};
}  // namespace achilles::control::actuators