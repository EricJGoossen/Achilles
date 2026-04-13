#pragma once

#include "dynamics/joints/abstract_joint.hpp"
#include "dynamics/link.hpp"
#include "spatial/inertia.hpp"
#include "spatial/surge.hpp"
#include "spatial/wrench.hpp"

namespace achilles::control::actuators {

class Actuator {
  public:
    using Frame = geometry::Frame<Actuator>;

    Actuator(const spatial::Wrench& dof, dynamics::joints::AbstractJoint& joint)
      : dof_(dof.normalized()), joint_(&joint) {}

    void actuate(const spatial::Inertia& composite_inertia) {
        spatial::Surge acceleration{
            spatial::Surge::fromWrench(dof_, composite_inertia)};

        joint_->applyAcceleration(acceleration * effort_);
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