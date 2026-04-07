#pragma once

#include "control/actuators/actuator.hpp"
#include "math/vector.hpp"

namespace achilles::control::actuators {

class AngularActuator : public Actuator {
  public:
    AngularActuator(
        const math::Vector& dof, dynamics::joints::AbstractJoint& joint
    )
      : Actuator({math::Vector::zero(), dof}, joint) {}
};
}  // namespace achilles::control::actuators