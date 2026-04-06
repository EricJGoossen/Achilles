#pragma once

#include <Eigen/Dense>

#include "dynamics/joints/joint.hpp"
#include "dynamics/link.hpp"
#include "math/unit_vector.hpp"

namespace achilles::dynamics::joints {

class PlanarJoint : public Joint<3> {
    static constexpr int DOF = 3;

  public:
    PlanarJoint(
        const geometry::Frame& frame,
        const Link& parent_link,
        const Link& child_link,
        const math::UnitVector& normal,
        spatial::Pose initial_position,
        spatial::Twist initial_velocity
    );

  private:
    struct PlanarBasis {
        Eigen::Matrix3d k;
        Eigen::Matrix3d k2;
        Eigen::Vector3d n;
        Eigen::Vector3d t1;
        Eigen::Vector3d t2;
    };

    static PlanarBasis makeBasis(const math::UnitVector& normal);

    static spatial::Pose makeChildPose(
        const Eigen::Matrix<double, DOF, 1>& q, const PlanarBasis& b
    );
    static Eigen::Matrix<double, DOF, 1> makeJointPose(
        const spatial::Pose& pose, const PlanarBasis& b
    );

    static Eigen::Matrix<double, 6, DOF> makeMotionSubspace(
        const math::UnitVector& normal
    );
};
}  // namespace achilles::dynamics::joints