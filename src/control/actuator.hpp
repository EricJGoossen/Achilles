#pragma once

#include "dynamics/joints/abstract_joint.hpp"
#include "dynamics/link.hpp"
#include "math/dual.hpp"
#include "spatial/inertia.hpp"
#include "spatial/jerk.hpp"
#include "spatial/wrench.hpp"

namespace achilles::control {

class Actuator {
  protected:
    struct WrenchBasis : math::Dual<WrenchBasis> {
        using Base = math::Dual<WrenchBasis>;

        WrenchBasis(const math::Vector& linear, const math::Vector& angular)
          : Base(linear, angular) {}

        spatial::Wrench toWrench(double effort) const {
            return (*this * effort).dual();
        }
    };

  public:
    Actuator(const WrenchBasis& dof, dynamics::joints::AbstractJoint& joint)
      : dof_(dof.normalized()), joint_(&joint) {}

    void actuate(const spatial::Inertia& composite_inertia) {
        spatial::Jerk acceleration{spatial::Jerk::zero()};
        acceleration.propagate(dof_.toWrench(effort_), composite_inertia);

        joint_->applyAcceleration(acceleration);
        effort_ = 0;
    }

    const dynamics::Link& childLink() { return joint_->childLink(); }

    void applyEffort(double effort) { effort_ = effort; }

  private:
    WrenchBasis dof_;

    dynamics::joints::AbstractJoint* joint_;
    double effort_ = 0;
};  // class actuator

}  // namespace achilles::control