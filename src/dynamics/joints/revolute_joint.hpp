#pragma once

#include <Eigen/Dense>

#include "dynamics/joints/base_joint.hpp"
#include "math/unit_vector.hpp"

namespace achilles::dynamics::joints {

class RevoluteJoint : public BaseJoint<RevoluteJoint, 1> {
    static constexpr int DOF = 1;

  public:
    RevoluteJoint(
        const geometry::Frame& frame,
        const Link& parent_link,
        const Link& child_link,
        const math::UnitVector& axis,
        spatial::Pose initial_position,
        spatial::Twist initial_velocity
    );

    constexpr static int dof() { return DOF; }

  private:
    friend BaseJoint<RevoluteJoint, DOF>;

    struct RevoluteBasis {
        RevoluteBasis(const math::UnitVector& axis);

        Eigen::Matrix3d k;
        Eigen::Matrix3d k2;
        Eigen::Vector3d n;
    };

    spatial::Pose makeChildPose(const Eigen::Matrix<double, DOF, 1>& q);
    Eigen::Matrix<double, DOF, 1> makeJointPose(const spatial::Pose& pose);

    static Eigen::Matrix<double, 6, DOF> makeMotionSubspace(
        const math::UnitVector& axis
    );

    RevoluteBasis b_;
};  // class RevoluteJoint
}  // namespace achilles::dynamics::joints