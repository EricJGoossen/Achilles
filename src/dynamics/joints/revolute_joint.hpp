#pragma once

#include <Eigen/Dense>

#include "dynamics/joints/joint.hpp"
#include "math/unit_vector.hpp"

namespace achilles::dynamics::joints {

class RevoluteJoint : public Joint<1> {
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

  private:
    struct PlanarBasis {
        Eigen::Matrix3d k;
        Eigen::Matrix3d k2;
    };

    static PlanarBasis makeBasis(const math::UnitVector& axis);

    static spatial::Pose makeChildPose(
        const Eigen::Matrix<double, DOF, 1>& q, const PlanarBasis& b
    );

    static Eigen::Matrix<double, DOF, 1> makeJointPose(
        const spatial::Pose& pose, const Eigen::Vector3d& a
    );

    static Eigen::Matrix<double, 6, DOF> makeMotionSubspace(
        const math::UnitVector& axis
    );
};  // class RevoluteJoint
}  // namespace achilles::dynamics::joints