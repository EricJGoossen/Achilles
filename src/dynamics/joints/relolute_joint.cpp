#include "revolute_joint.hpp"

namespace achilles::dynamics::joints {

RevoluteJoint(
    const geometry::Frame& frame,
    const Link& parent_link,
    const Link& child_link,
    const math::UnitVector& axis,
    spatial::Pose initial_position,
    spatial::Twist initial_velocity
)
  : Joint<DOF>(
        frame,
        parent_link,
        child_link,
        makeMotionSubspace(axis),
        [b = makeBasis(axis)](const Eigen::Matrix<double, DOF, 1>& q) {
            return makeChildPose(q, b);
        },
        [a = axis.mat()](const spatial::Pose& pose) {
            return makeJointPose(pose, a);
        },
        initial_position,
        initial_velocity
    ) {}

RevoluteJoint::PlanarBasis RevoluteJoint::makeBasis(const math::UnitVector& axis
) {
    Eigen::Matrix3d k = axis.skew();
    return {k, k * k};
}

spatial::Pose RevoluteJoint::makeChildPose(
    const Eigen::Matrix<double, DOF, 1>& q, const PlanarBasis& b
) {
    double angle = q[0];

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + std::sin(angle) * b.k +
                        (1.0 - std::cos(angle)) * b.k2;

    return spatial::Pose(R, Eigen::Vector3d::Zero());
}

Eigen::Matrix<double, DOF, 1> RevoluteJoint::makeJointPose(
    const spatial::Pose& pose, const Eigen::Vector3d& a
) {
    Eigen::AngleAxisd aa(pose.orientation().mat().toRotationMatrix());
    double angle = aa.angle() * aa.axis().dot(a);
    return Eigen::Matrix<double, DOF, 1>(angle);
}

Eigen::Matrix<double, 6, DOF> RevoluteJoint::makeMotionSubspace(
    const math::UnitVector& axis
) {
    Eigen::Matrix<double, 6, DOF> motion_subspace;
    motion_subspace << axis.x(), axis.y(), axis.z(), 0, 0, 0;
    return motion_subspace;
}

}  // namespace achilles::dynamics::joints