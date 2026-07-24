#include <Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "control/actuators/actuator.hpp"
#include "control/actuators/angular_actuator.hpp"
#include "control/actuators/linear_actuator.hpp"
#include "control/plant.hpp"
#include "dynamics/joint_tree.hpp"
#include "dynamics/joints/planar_joint.hpp"
#include "dynamics/joints/revolute_joint.hpp"
#include "dynamics/link.hpp"
#include "geometry/frame.hpp"
#include "geometry/transform.hpp"
#include "geometry/transform_tree.hpp"
#include "spatial/inertia.hpp"
#include "spatial/pose.hpp"
#include "spatial/twist.hpp"

namespace achilles {
int main() {
    static std::string F_world = "world";
    static std::string F_odomJ = "odom_joint";
    static std::string F_base = "base";
    static std::string F_armJ = "arm_joint";
    static std::string F_arm = "arm";
    static std::string F_handJ = "hand_joint";
    static std::string F_hand = "hand";
    static std::string F_plant = "plant";
    static std::string A_odom_x_name = "actuator_odom_x";
    static std::string A_odom_y_name = "actuator_odom_y";
    static std::string A_odom_theta_name = "actuator_odom_theta";

    // clang-format off
    Eigen::Matrix3d M_base;
    M_base << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;

    Eigen::Matrix3d M_arm;
    M_arm << 0.001, 0, 0,
             0, 1, 0,
             0, 0, 1;

    Eigen::Matrix3d M_hand;
    M_hand << 1.0/12.0,     0,        0,
                0,        3.0,  -1.0/2.0,
                0,    -1.0/2.0,     3.0;
    // clang-format on

    auto L_world = std::make_unique<dynamics::Link>(
        F_world.data(), spatial::Inertia::zero()
    );
    auto L_base = std::make_unique<dynamics::Link>(
        F_base.data(), spatial::Inertia(M_base, math::Vector::zero(), 1.0)
    );
    auto L_arm = std::make_unique<dynamics::Link>(
        F_arm.data(), spatial::Inertia(M_arm, math::Vector::zero(), 1.0 / 2.0)
    );
    auto L_hand = std::make_unique<dynamics::Link>(
        F_hand.data(),
        spatial::Inertia(M_hand, math::Vector(0.02, 0, 0), 1.0 / 4.0)
    );

    auto T_world_odomJ = std::make_unique<geometry::Transform>(
        L_world->frame(),
        dynamics::joints::AbstractJoint::Frame(F_odomJ.c_str()),
        spatial::Pose::identity()
    );
    auto T_odomJ_base = std::make_unique<geometry::Transform>(
        dynamics::joints::AbstractJoint::Frame(F_odomJ.c_str()),
        L_base->frame(),
        spatial::Pose(math::Vector(0.0, 0.0, 1), math::Quaternion::identity())
    );
    auto T_base_armJ = std::make_unique<geometry::Transform>(
        L_base->frame(),
        dynamics::joints::AbstractJoint::Frame(F_armJ.c_str()),
        spatial::Pose(math::Vector(0.0, 0.0, 1), math::Quaternion::identity())
    );
    auto T_armJ_arm = std::make_unique<geometry::Transform>(
        dynamics::joints::AbstractJoint::Frame(F_armJ.c_str()),
        L_arm->frame(),
        spatial::Pose(
            math::Vector(0.0, 0.0, 0.25), math::Quaternion::identity()
        )
    );
    auto T_arm_handJ = std::make_unique<geometry::Transform>(
        L_arm->frame(),
        dynamics::joints::AbstractJoint::Frame(F_handJ.c_str()),
        spatial::Pose(
            math::Vector(0.0, 0.0, 0.25), math::Quaternion::identity()
        )
    );
    auto T_handJ_hand = std::make_unique<geometry::Transform>(
        dynamics::joints::AbstractJoint::Frame(F_handJ.c_str()),
        L_hand->frame(),
        spatial::Pose(math::Vector(0.0, 0.0, 0.5), math::Quaternion::identity())
    );

    auto J_odom = std::make_unique<dynamics::joints::PlanarJoint>(
        F_odomJ.c_str(),
        *L_world,
        *L_base,
        math::Vector(0.0, 0.0, 1.0),
        T_odomJ_base->pose(),
        spatial::Twist::identity()
    );

    auto J_arm = std::make_unique<dynamics::joints::RevoluteJoint>(
        F_armJ.c_str(),
        *L_base,
        *L_arm,
        math::Vector(0.0, 1.0, 0.0),
        T_armJ_arm->pose(),
        spatial::Twist::identity()
    );

    auto J_hand = std::make_unique<dynamics::joints::RevoluteJoint>(
        F_handJ.c_str(),
        *L_arm,
        *L_hand,
        math::Vector(0.0, 1.0, 0.0),
        T_handJ_hand->pose(),
        spatial::Twist::identity()
    );

    auto A_odom_x = std::make_unique<control::actuators::LinearActuator>(
        math::Vector(1.0, 0.0, 0.0), *J_odom
    );
    auto A_odom_y = std::make_unique<control::actuators::LinearActuator>(
        math::Vector(0.0, 1.0, 0.0), *J_odom
    );
    auto A_odom_theta = std::make_unique<control::actuators::AngularActuator>(
        math::Vector(0.0, 0.0, 1.0), *J_odom
    );

    geometry::TransformTree transform_tree;
    transform_tree.addTransform(std::move(T_base_armJ));
    transform_tree.addTransform(std::move(T_armJ_arm));
    transform_tree.addTransform(std::move(T_arm_handJ));
    transform_tree.addTransform(std::move(T_handJ_hand));

    dynamics::JointTree joint_tree;
    joint_tree.addJoint(std::move(J_odom));
    joint_tree.addJoint(std::move(J_arm));
    joint_tree.addJoint(std::move(J_hand));

    std::unordered_map<dynamics::Link::Frame, std::unique_ptr<dynamics::Link>>
        links;
    links.emplace(L_world->frame(), std::move(L_world));
    links.emplace(L_base->frame(), std::move(L_base));
    links.emplace(L_arm->frame(), std::move(L_arm));
    links.emplace(L_hand->frame(), std::move(L_hand));

    std::unordered_map<
        control::actuators::Actuator::Frame,
        std::unique_ptr<control::actuators::Actuator>>
        actuators;
    actuators.emplace(
        control::actuators::Actuator::Frame(A_odom_x_name.c_str()),
        std::move(A_odom_x)
    );
    actuators.emplace(
        control::actuators::Actuator::Frame(A_odom_y_name.c_str()),
        std::move(A_odom_y)
    );
    actuators.emplace(
        control::actuators::Actuator::Frame(A_odom_theta_name.c_str()),
        std::move(A_odom_theta)
    );

    control::Plant plant(
        F_plant.c_str(),
        std::move(transform_tree),
        std::move(joint_tree),
        std::move(T_odomJ_base),
        std::move(T_world_odomJ),
        std::move(actuators),
        std::move(links)
    );

    std::vector<control::actuators::Actuator*> actuator_ptrs =
        plant.getActuators();

    while (true) {
        actuator_ptrs[0]->applyEffort(1.0);
        plant.update(0.01);
    }

    return 0;
}
}  // namespace achilles

int main() { return achilles::main(); }