#pragma once

#include "dynamics/joints/abstract_joint.hpp"
#include "dynamics/link.hpp"
#include "spatial/inertia.hpp"
#include "spatial/jerk.hpp"
#include "spatial/wrench.hpp"

namespace achilles::control::actuators {

class Actuator {
  public:
    Actuator(const spatial::Wrench& dof, dynamics::joints::AbstractJoint& joint)
      : dof_(dof.normalized()), joint_(&joint) {}

    void actuate(const spatial::Inertia& composite_inertia) {
        spatial::Jerk acceleration{
            spatial::Jerk::fromWrench(dof_, composite_inertia)
        };

        joint_->applyAcceleration(acceleration);
        effort_ = 0;
    }

    const dynamics::Link& childLink() { return joint_->childLink(); }

    void applyEffort(double effort) { effort_ = effort; }

  private:
    spatial::Wrench dof_;

    dynamics::joints::AbstractJoint* joint_;
    double effort_ = 0;
};  // class actuator

}  // namespace achilles::control::actuators