#include "planar_joint.hpp"

namespace achilles::dynamics::joints {

PlanarJoint::PlanarJoint(
    const geometry::Frame& frame,
    const Link& parent_link,
    const Link& child_link,
    const math::UnitVector& normal,
    spatial::Pose initial_position,
    spatial::Twist initial_velocity
)
  : BaseJoint<PlanarJoint, PlanarJoint::DOF>(
        frame,
        parent_link,
        child_link,
        makeMotionSubspace(normal),
        std::move(initial_position),
        std::move(initial_velocity)
    ),
    b_(normal) {}

PlanarJoint::PlanarBasis::PlanarBasis(const math::UnitVector& normal)
  : k(normal.skew()),
    k2(k * k),
    n(normal.mat()),
    t1(n.unitOrthogonal()),
    t2(n.cross(t1)) {}

spatial::Pose PlanarJoint::makeChildPose(
    const Eigen::Matrix<double, PlanarJoint::DOF, 1>& q
) {
    // clang-format off
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() 
                      + std::sin(q(2)) * b_.k 
                      + (1.0 - std::cos(q(2))) * b_.k2;
    // clang-format on

    Eigen::Vector3d t = q(0) * b_.t1 + q(1) * b_.t2;

    return {R, t};
}

Eigen::Matrix<double, PlanarJoint::DOF, 1> PlanarJoint::makeJointPose(
    const spatial::Pose& pose
) {
    Eigen::AngleAxisd aa(pose.orientation().mat().toRotationMatrix());

    return {
        pose.position().dot(b_.t1),
        pose.position().dot(b_.t2),
        aa.angle() * aa.axis().dot(b_.n)
    };
}

Eigen::Matrix<double, 6, PlanarJoint::DOF> PlanarJoint::makeMotionSubspace(
    const PlanarBasis& b
) {
    Eigen::Matrix<double, 6, PlanarJoint::DOF> S;
    S.col(0) << b.t1, Eigen::Vector3d::Zero();
    S.col(1) << b.t2, Eigen::Vector3d::Zero();
    S.col(2) << Eigen::Vector3d::Zero(), b.n;

    return S;
}

}  // namespace achilles::dynamics::joints