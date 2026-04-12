#pragma once

#include <Eigen/Dense>

#include "dynamics/joints/abstract_joint.hpp"
#include "spatial/inertia.hpp"
#include "spatial/pose.hpp"
#include "spatial/twist.hpp"

namespace achilles::dynamics::joints {

template <typename Derived, int DOF>
class BaseJoint : public AbstractJoint {
  public:
    BaseJoint(
        const geometry::Frame& frame,
        const Link& parent_link,
        const Link& child_link,
        Eigen::Matrix<double, 6, DOF> motion_subspace,
        spatial::Pose initial_position,
        spatial::Twist initial_velocity
    )
      : AbstractJoint(
            frame,
            parent_link,
            child_link,
            std::move(initial_position),
            std::move(initial_velocity)
        ),
        motion_subspace_(std::move(motion_subspace)),
        projection_matrix_(
            (motion_subspace_.transpose() * motion_subspace_).inverse() *
            motion_subspace_.transpose()
        ),
        q_(static_cast<Derived*>(this)->makeJointPose(position_cache_)),
        q_dot_(projection_matrix_ * velocity_cache_.mat()),
        q_ddot_(Eigen::Matrix<double, DOF, 1>::Zero()) {}

    spatial::Inertia solveInertia(const spatial::Inertia& child_inertia
    ) const override {
        Eigen::Matrix<double, 6, DOF> IS =
            child_inertia.mat() * motion_subspace_;
        Eigen::Matrix<double, DOF, DOF> SIS = motion_subspace_.transpose() * IS;

        return spatial::Inertia(
            child_inertia.mat() - IS * SIS.ldlt().solve(IS.transpose())
        );
    }

    void applyAcceleration(const spatial::Surge& delta_acceleration) override {
        q_ddot_ += projection_matrix_ * delta_acceleration.mat();

        updateAccelerationCache();
    }

    void integrate(double dt) override {
        q_dot_ += q_ddot_ * dt;
        q_ += q_dot_ * dt;
        q_ddot_ = Eigen::Matrix<double, DOF, 1>::Zero();

        updatePositionCache();
        updateVelocityCache();

        acceleration_cache_ = spatial::Surge::zero();
    }

  private:
    void updatePositionCache() {
        position_cache_ = derived()->makeChildPose(q_);
    }
    void updateVelocityCache() {
        velocity_cache_ = {motion_subspace_ * q_dot_};
    }
    void updateAccelerationCache() {
        acceleration_cache_ = {motion_subspace_ * q_ddot_};
    }

    Derived* derived() { return static_cast<Derived*>(this); }
    const Derived* derived() const { return static_cast<const Derived*>(this); }

    Eigen::Matrix<double, 6, DOF> motion_subspace_;
    Eigen::Matrix<double, DOF, 6> projection_matrix_;

    Eigen::Matrix<double, DOF, 1> q_;
    Eigen::Matrix<double, DOF, 1> q_dot_;
    Eigen::Matrix<double, DOF, 1> q_ddot_;
};
}  // namespace achilles::dynamics::joints