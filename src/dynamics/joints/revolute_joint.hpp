#pragma once

#include "dynamics/joints/joint.hpp"
#include "math/unit_vector.hpp"
#include "math/vector.hpp"

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
    ) {
        double angle = q[0];

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity() +
                            std::sin(angle) * b.k +
                            (1.0 - std::cos(angle)) * b.k2;

        return spatial::Pose(R, Eigen::Vector3d::Zero());
    }

    static Eigen::Matrix<double, DOF, 1> makeJointPose(
        const spatial::Pose& pose, const Eigen::Vector3d& a
    );

    static Eigen::Matrix<double, 6, DOF> makeMotionSubspace(
        const math::UnitVector& axis
    );
};  // class RevoluteJoint
}  // namespace achilles::dynamics::joints