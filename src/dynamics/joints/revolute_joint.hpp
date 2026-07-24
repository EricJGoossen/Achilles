#pragma once

#include <Eigen/Dense>

#include "dynamics/joints/base_joint.hpp"
#include "math/unit_vector.hpp"

namespace achilles::dynamics::joints {

class RevoluteJoint : public BaseJoint<RevoluteJoint, 1> {
    static constexpr int kDOF = 1;

  public:
    RevoluteJoint(
        const char* frame,
        Link::Frame parent_link,
        Link::Frame child_link,
        const math::UnitVector& axis,
        spatial::Pose initial_position,
        spatial::Twist initial_velocity
    );

    constexpr static int dof() { return kDOF; }

  private:
    friend BaseJoint<RevoluteJoint, kDOF>;

    struct RevoluteBasis {
        RevoluteBasis(const math::UnitVector& axis);

        Eigen::Matrix3d k;
        Eigen::Matrix3d k2;
        Eigen::Vector3d n;
    };

    spatial::Pose makeChildPose(const Eigen::Matrix<double, kDOF, 1>& q);
    Eigen::Matrix<double, kDOF, 1> makeJointPose(const spatial::Pose& pose);

    static Eigen::Matrix<double, 6, kDOF> makeMotionSubspace(
        const math::UnitVector& axis
    );

    RevoluteBasis b_;
};  // class RevoluteJoint
}  // namespace achilles::dynamics::joints