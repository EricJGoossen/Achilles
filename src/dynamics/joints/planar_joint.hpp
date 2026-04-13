#pragma once

#include <Eigen/Dense>

#include "dynamics/joints/base_joint.hpp"
#include "dynamics/link.hpp"
#include "math/unit_vector.hpp"

namespace achilles::dynamics::joints {

class PlanarJoint : public BaseJoint<PlanarJoint, 3> {
    static constexpr int DOF = 3;

  public:
    PlanarJoint(
        const char* frame,
        const Link& parent_link,
        const Link& child_link,
        const math::UnitVector& normal,
        spatial::Pose initial_position,
        spatial::Twist initial_velocity
    );

  private:
    friend BaseJoint<PlanarJoint, DOF>;

    struct PlanarBasis {
        PlanarBasis(const math::UnitVector& normal);

        Eigen::Matrix3d k;
        Eigen::Matrix3d k2;
        Eigen::Vector3d n;
        Eigen::Vector3d t1;
        Eigen::Vector3d t2;
    };

    spatial::Pose makeChildPose(const Eigen::Matrix<double, DOF, 1>& q);
    Eigen::Matrix<double, DOF, 1> makeJointPose(const spatial::Pose& pose);

    static Eigen::Matrix<double, 6, DOF> makeMotionSubspace(const PlanarBasis& b
    );

    PlanarBasis b_;
};
}  // namespace achilles::dynamics::joints