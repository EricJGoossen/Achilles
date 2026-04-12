#pragma once

#include <Eigen/Dense>

#include "dynamics/joints/base_joint.hpp"
#include "dynamics/link.hpp"
#include "geometry/frame.hpp"
#include "spatial/pose.hpp"
#include "spatial/twist.hpp"

namespace achilles::dynamics::joints {

template <int DOF, typename ChildPoseGenerator, typename JointPoseGenerator>
concept Generators = requires(
    ChildPoseGenerator make_child_pose,
    JointPoseGenerator make_joint_pose,
    const spatial::Pose& pose,
    const Eigen::Matrix<double, DOF, 1>& q
) {
    { make_child_pose(q) } -> std::convertible_to<spatial::Pose>;
    {
        make_joint_pose(pose)
        } -> std::convertible_to<Eigen::Matrix<double, DOF, 1>>;
};

template <int DOF, typename ChildPoseGenerator, typename JointPoseGenerator>
requires Generators<DOF, ChildPoseGenerator, JointPoseGenerator>
class GeneralJoint
  : public BaseJoint<
        GeneralJoint<DOF, ChildPoseGenerator, JointPoseGenerator>,
        DOF> {
    using Base = BaseJoint<
        GeneralJoint<DOF, ChildPoseGenerator, JointPoseGenerator>,
        DOF>;

  public:
    GeneralJoint(
        const geometry::Frame& frame,
        const Link& parent_link,
        const Link& child_link,
        spatial::Pose initial_position,
        spatial::Twist initial_velocity,
        Eigen::Matrix<double, 6, DOF> motion_subspace,
        ChildPoseGenerator make_child_pose,
        JointPoseGenerator make_joint_pose
    )
      : Base(
            frame,
            parent_link,
            child_link,
            make_joint_pose(initial_position),
            (motion_subspace.transpose() * motion_subspace).inverse() *
                motion_subspace.transpose() * initial_velocity.mat(),
            std::move(initial_position),
            std::move(initial_velocity)
        ),
        make_child_pose_(std::move(make_child_pose)),
        make_joint_pose_(std::move(make_joint_pose)) {}

    constexpr static int dof() { return DOF; }

  private:
    friend Base;

    spatial::Pose makeChildPose(const Eigen::Matrix<double, DOF, 1>& q) {
        return make_child_pose_(q);
    }
    Eigen::Matrix<double, DOF, 1> makeJointPose(const spatial::Pose& q_dot) {
        return make_joint_pose_(q_dot);
    }

    ChildPoseGenerator make_child_pose_;
    JointPoseGenerator make_joint_pose_;
};
}  // namespace achilles::dynamics::joints