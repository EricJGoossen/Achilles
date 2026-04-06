#pragma once

#include <Eigen/Dense>
#include <functional>
#include <utility>
#include <vector>

#include "dynamics/joints/i_joint.hpp"
#include "dynamics/link.hpp"
#include "geometry/frame.hpp"
#include "geometry/transform.hpp"
#include "math/dual.hpp"
#include "spatial/jerk.hpp"
#include "spatial/pose.hpp"
#include "spatial/twist.hpp"


namespace achilles::dynamics::joints {

template <int DOF>
class Joint : public IJoint {
    using ChildPoseFunc =
        std::function<spatial::Pose(const Eigen::Matrix<double, DOF, 1>&)>;
    using JointPoseFunc =
        std::function<Eigen::Matrix<double, DOF, 1>(const spatial::Pose&)>;

  public:
    Joint(
        const geometry::Frame& frame,
        const Link& parent_link,
        const Link& child_link,
        Eigen::Matrix<double, 6, DOF> motion_subspace,
        ChildPoseFunc make_child_pose,
        JointPoseFunc make_joint_pose,
        spatial::Pose initial_position,
        spatial::Twist initial_velocity
    );

    inline const geometry::Frame& frame() override { return frame_; }
    inline const Link& parent_link() override { return parent_link_; }
    inline const Link& child_link() override { return child_link_; }
    inline const spatial::Pose& position() override { return position_cache_; }
    inline const spatial::Twist& velocity() override { return velocity_cache_; }
    inline const spatial::Jerk& acceleration() override {
        return acceleration_cache_;
    }

    spatial::Inertia solveInertia(const spatial::Inertia& child_inertia
    ) const override;
    void applyAcceleration(const spatial::Jerk& delta_acceleration) override;
    void integrate(double dt) override;

  private:
    void updatePositionCache();
    void updateVelocityCache();
    void updateAccelerationCache();

    Eigen::Matrix<double, DOF, 1> projectAcceleration(const spatial::Jerk& force
    ) const;

    const geometry::Frame& frame_;
    const Link& parent_link_;
    const Link& child_link_;

    Eigen::Matrix<double, 6, DOF> motion_subspace_;
    Eigen::Matrix<double, DOF, 6> projection_matrix_;

    ChildPoseFunc make_child_pose_;
    JointPoseFunc make_joint_pose_;

    Eigen::Matrix<double, DOF, 1> q_;
    Eigen::Matrix<double, DOF, 1> q_dot_;
    Eigen::Matrix<double, DOF, 1> q_ddot_;

    spatial::Pose position_cache_;
    spatial::Twist velocity_cache_;
    spatial::Jerk acceleration_cache_;

};  // class Joint
}  // namespace achilles::dynamics::joints