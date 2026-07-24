#pragma once

#include "dynamics/joint_tree.hpp"
#include "dynamics/joints/abstract_joint.hpp"
#include "dynamics/link.hpp"
#include "spatial/inertia.hpp"
#include "spatial/surge.hpp"
#include "spatial/wrench.hpp"

namespace achilles::control::actuators {

class Actuator {
  public:
    using Frame = geometry::Frame<Actuator>;

    Actuator(
        dynamics::joints::AbstractJoint::Frame joint, const spatial::Wrench& dof
    )
      : joint_(joint), dof_(dof.normalized()) {}

    void actuate(
        const dynamics::JointTree& joints,
        const spatial::Inertia& composite_inertia
    ) {
        spatial::Surge acceleration{
            spatial::Surge::fromWrench(dof_, composite_inertia)};

        joints.getJoint(joint_).applyAcceleration(acceleration * effort_);
        effort_ = 0;
    }

    dynamics::joints::AbstractJoint::Frame joint() const { return joint_; }
    void applyEffort(double effort) { effort_ = effort; }

  private:
    const dynamics::joints::AbstractJoint::Frame joint_;
    const spatial::Wrench dof_;

    double effort_ = 0;
};  // class actuator

}  // namespace achilles::control::actuators