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
  : Joint<PlanarJoint::DOF>(
        frame,
        parent_link,
        child_link,
        makeMotionSubspace(normal),
        [b = makeBasis(normal)](
            const Eigen::Matrix<double, PlanarJoint::DOF, 1>& q
        ) { return makeChildPose(q, b); },
        [b = makeBasis(normal)](const spatial::Pose& pose) {
            return makeJointPose(pose, b);
        },
        std::move(initial_position),
        std::move(initial_velocity)
    ) {}

PlanarJoint::PlanarBasis PlanarJoint::makeBasis(const math::UnitVector& normal
) {
    Eigen::Matrix3d k = normal.skew();
    Eigen::Vector3d n = normal.mat();
    Eigen::Vector3d t1 = n.unitOrthogonal();
    return {k, k * k, n, t1, n.cross(t1)};
}

spatial::Pose PlanarJoint::makeChildPose(
    const Eigen::Matrix<double, PlanarJoint::DOF, 1>& q, const PlanarBasis& b
) {
    double x = q(0);
    double y = q(1);
    double theta = q(2);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + std::sin(theta) * b.k;
    +(1.0 - std::cos(theta)) * b.k2;

    Eigen::Vector3d t = x * b.t1 + y * b.t2;

    return {R, t};
}

Eigen::Matrix<double, PlanarJoint::DOF, 1> PlanarJoint::makeJointPose(
    const spatial::Pose& pose, const PlanarBasis& b
) {
    Eigen::Matrix3d R = pose.orientation().mat().toRotationMatrix();
    Eigen::Vector3d t = pose.position().mat();

    double x = t.dot(b.t1);
    double y = t.dot(b.t2);

    Eigen::AngleAxisd aa(R);
    double theta = aa.angle() * aa.axis().dot(b.n);

    return (Eigen::Matrix<double, PlanarJoint::DOF, 1>() << x, y, theta)
        .finished();
}

Eigen::Matrix<double, 6, PlanarJoint::DOF> PlanarJoint::makeMotionSubspace(
    const math::UnitVector& normal
) {
    Eigen::Vector3d n = normal.mat();
    Eigen::Vector3d t1 = n.unitOrthogonal();
    Eigen::Vector3d t2 = n.cross(t1);

    Eigen::Matrix<double, 6, PlanarJoint::DOF> S;
    S.col(0) << 0, 0, 0, t1.x(), t1.y(), t1.z();
    S.col(1) << 0, 0, 0, t2.x(), t2.y(), t2.z();
    S.col(2) << n.x(), n.y(), n.z(), 0, 0, 0;

    return S;
}

}  // namespace achilles::dynamics::joints