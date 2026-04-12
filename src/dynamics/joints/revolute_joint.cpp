#include "revolute_joint.hpp"

namespace achilles::dynamics::joints {

RevoluteJoint::RevoluteJoint(
    const geometry::Frame& frame,
    const Link& parent_link,
    const Link& child_link,
    const math::UnitVector& axis,
    spatial::Pose initial_position,
    spatial::Twist initial_velocity
)
  : BaseJoint<RevoluteJoint, RevoluteJoint::DOF>(
        frame,
        parent_link,
        child_link,
        makeMotionSubspace(axis),
        std::move(initial_position),
        std::move(initial_velocity),
        makeJointPose(initial_position)
    ),
    b_(axis) {}

RevoluteJoint::RevoluteBasis::RevoluteBasis(const math::UnitVector& axis)
  : k(axis.skew()), k2(k * k), n(axis.mat()) {}

spatial::Pose RevoluteJoint::makeChildPose(
    const Eigen::Matrix<double, RevoluteJoint::DOF, 1>& q
) {
    // clang-format off
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() 
                      + std::sin(q(0)) * b_.k 
                      + (1.0 - std::cos(q(0))) * b_.k2;
    // clang-format on

    return {R, Eigen::Vector3d::Zero()};
}

Eigen::Matrix<double, RevoluteJoint::DOF, 1> RevoluteJoint::makeJointPose(
    const spatial::Pose& pose
) {
    Eigen::AngleAxisd aa(pose.orientation().mat().toRotationMatrix());

    return {Eigen::Matrix<double, RevoluteJoint::DOF, 1>(
        aa.angle() * aa.axis().dot(b_.n)
    )};
}

Eigen::Matrix<double, 6, RevoluteJoint::DOF> RevoluteJoint::makeMotionSubspace(
    const math::UnitVector& axis
) {
    Eigen::Matrix<double, 6, RevoluteJoint::DOF> motion_subspace;
    motion_subspace << Eigen::Vector3d::Zero(), axis.mat();
    return motion_subspace;
}

}  // namespace achilles::dynamics::joints