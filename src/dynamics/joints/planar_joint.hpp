#pragma once

#include <Eigen/Dense>

#include "dynamics/joints/joint.hpp"
#include "math/unit_vector.hpp"
#include "dynamics/link.hpp"

namespace achilles::dynamics::joints {

class PlanarJoint : public Joint<3> {
    static constexpr int DOF = 3;

public:
    PlanarJoint(
        const geometry::Frame frame,
        const Link& parent_link,
        const Link& child_link,
        const math::UnitVector& normal,
        const spatial::Pose initial_position,
        const spatial::Twist initial_velocity) :
        Joint<DOF>(
            frame,
            parent_link,
            child_link,
            makeMotionSubspace(normal),
            [b = makeBasis(normal)](const Eigen::Matrix<double, DOF, 1>& q) {
                return makeChildPose(q, b);
            },
            [b = makeBasis(normal)](const spatial::Pose& pose) {
                return makeJointPose(pose, b);
            },
            initial_position,
            initial_velocity
        ) {}

private:
    struct PlanarBasis {
        Eigen::Matrix3d k;
        Eigen::Matrix3d k2;
        Eigen::Vector3d n;
        Eigen::Vector3d t1;
        Eigen::Vector3d t2;
    };

    static PlanarBasis makeBasis(const math::UnitVector& normal) {
        Eigen::Matrix3d k = normal.skew();
        Eigen::Vector3d n = normal.mat();
        Eigen::Vector3d t1 = n.unitOrthogonal();
        return { k, k * k, n, t1, n.cross(t1) };
    }

    static spatial::Pose makeChildPose(const Eigen::Matrix<double, DOF, 1>& q, const PlanarBasis& b) {
        double x     = q[0];
        double y     = q[1];
        double theta = q[2];

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity()
            + std::sin(theta) * b.k;
            + (1.0 - std::cos(theta)) * b.k2;

        Eigen::Vector3d t = x * b.t1 + y * b.t2;

        return spatial::Pose(R, t);
    }

    static Eigen::Matrix<double, DOF, 1> makeJointPose(const spatial::Pose& pose, const PlanarBasis& b) {
        Eigen::Matrix3d R = pose.orientation().mat().toRotationMatrix();
        Eigen::Vector3d t = pose.position().mat();

        double x = t.dot(b.t1);
        double y = t.dot(b.t2);

        Eigen::AngleAxisd aa(R);
        double theta = aa.angle() * aa.axis().dot(b.n);

        return (Eigen::Matrix<double, DOF, 1>() << x, y, theta).finished();
    }

    static Eigen::Matrix<double, 6, DOF> makeMotionSubspace(const math::UnitVector& normal) {
        Eigen::Vector3d n = normal.mat();
        Eigen::Vector3d t1 = n.unitOrthogonal();
        Eigen::Vector3d t2 = n.cross(t1);

        Eigen::Matrix<double, 6, DOF> S;
        S.col(0) << 0,     0,     0,     t1.x(), t1.y(), t1.z(); 
        S.col(1) << 0,     0,     0,     t2.x(), t2.y(), t2.z(); 
        S.col(2) << n.x(), n.y(), n.z(), 0,      0,      0;     

        return S;
    }
};
}