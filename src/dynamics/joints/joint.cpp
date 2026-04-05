#include "joint.hpp"

namespace achilles::dynamics::joints {

template <int N>
Joint<N>::Joint(
    const geometry::Frame frame,
    const Link& parent_link,
    const Link& child_link,
    const Eigen::Matrix<double, 6, N> motion_subspace,
    const Joint<N>::ChildPoseFunc make_child_pose,
    const Joint<N>::JointPoseFunc make_joint_pose,
    const spatial::Pose initial_position,
    const spatial::Twist initial_velocity) : 
    frame_(std::move(frame)),
    parent_link_(parent_link),
    child_link_(child_link),
    motion_subspace_(std::move(motion_subspace)),
    projection_matrix_((motion_subspace_.transpose() * motion_subspace_).inverse() * motion_subspace_.transpose()),
    make_child_pose_(std::move(make_child_pose)),
    make_joint_pose_(std::move(make_joint_pose)),
    q_(make_joint_pose(initial_position)),
    q_dot_(projection_matrix_ * initial_velocity.mat()),
    q_ddot_(Eigen::Matrix<double, N, 1>::Zero()),
    position_cache_(std::move(initial_position)),
    velocity_cache_(std::move(initial_velocity)),
    acceleration_cache_(spatial::Jerk::identity()) {}

template <int N>
spatial::Inertia Joint<N>::solveInertia(const spatial::Inertia& child_inertia) const {
    Eigen::Matrix<double, 6, N> IS = child_inertia.mat() * motion_subspace_;
    Eigen::Matrix<double, N, N> SIS = motion_subspace_.transpose() * IS;

    return spatial::Inertia(child_inertia.mat() - IS * SIS.inverse() * IS.transpose());
}

template <int N>
void Joint<N>::applyAcceleration(const spatial::Jerk& delta_acceleration) {
    q_ddot_ += projectAcceleration(delta_acceleration); 

    updateAccelerationCache();
}

template <int N>
void Joint<N>::integrate(double dt) {
    q_dot_ += q_ddot_ * dt;
    q_ += q_dot_ * dt;
    q_ddot_ = Eigen::Matrix<double, N, 1>::Zero();

    updatePositionCache();
    updateVelocityCache();
    updateAccelerationCache();
}  

template <int N>
void Joint<N>::updateVelocityCache() {
    velocity_cache_ = spatial::Twist(motion_subspace_ * q_dot_);
}

template <int N>
void Joint<N>::updateAccelerationCache() {
    acceleration_cache_ = spatial::Jerk(motion_subspace_ * q_ddot_);
}

template <int N>
void Joint<N>::updatePositionCache() {
    position_cache_ = make_child_pose_(q_);
}

template <int N>
Eigen::Matrix<double, N, 1> Joint<N>::projectAcceleration(const spatial::Jerk& acceleration) const {
    return projection_matrix_ * acceleration.mat();
}

template class Joint<1>;
template class Joint<3>;
template class Joint<6>;

} // namespace achilles::dynamics::joints