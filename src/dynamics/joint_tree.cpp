#include "joint_tree.hpp"

#include "dynamics/joints/abstract_joint.hpp"

namespace achilles::dynamics {
using InertiaMap = std::unordered_map<Link::Id, spatial::Inertia>;

void JointTree::addJoint(std::unique_ptr<joints::AbstractJoint> joint) {
    Link::Id child = joint->childLink().id();
    Link::Id parent = joint->parentLink().id();

    auto [it, inserted] = child_to_joint_.emplace(child, std::move(joint));

    if (!inserted) {
        throw std::runtime_error("Joint already exists for link");
    }

    parent_to_children_.emplace(child, std::vector<Link::Id>());
    parent_to_children_[parent].push_back(child);
}

const joints::AbstractJoint& JointTree::getJoint(const Link& child_link) const {
    return *child_to_joint_.at(child_link.id());
}

InertiaMap JointTree::computeCompositeInertias(const dynamics::Link& root
) const {
    InertiaMap result;
    recursiveInertia(root, result);
    return result;
}

void JointTree::propagateAccelerations(const Link& root) {
    for (Link::Id child : parent_to_children_[root.id()]) {
        joints::AbstractJoint& joint = *child_to_joint_.at(child);
        recursiveAcceleration(joint, spatial::Jerk::zero());
    }
}

void JointTree::integrate(double dt) {
    for (auto& [_, joint] : child_to_joint_) {
        joint->integrate(dt);
    }
}

void JointTree::updateTransforms(geometry::TransformTree& transform_tree) {
    for (auto& [_, joint] : child_to_joint_) {
        const geometry::Frame& joint_frame = joint->frame();
        const geometry::Frame& child_frame = joint->childLink().frame();
        transform_tree.updateTransform(
            joint_frame, child_frame, joint->position()
        );
    }
}

void JointTree::recursiveInertia(
    const Link& link, InertiaMap& composite_inertias
) const {
    spatial::Inertia composite_inertia = link.inertia();

    for (Link::Id child : parent_to_children_.at(link.id())) {
        auto& joint = *child_to_joint_.at(child);
        const Link& child_link = joint.childLink();

        recursiveInertia(child_link, composite_inertias);
        composite_inertia += joint.solveInertia(composite_inertias.at(child));
    }

    composite_inertias.emplace(link.id(), composite_inertia);
}

void JointTree::recursiveAcceleration(
    joints::AbstractJoint& joint, const spatial::Jerk& parent_acceleration
) {
    joint.applyAcceleration(parent_acceleration);

    for (Link::Id child : parent_to_children_[joint.childLink().id()]) {
        joints::AbstractJoint& child_joint = *child_to_joint_.at(child);
        recursiveAcceleration(child_joint, joint.acceleration());
    }
}

}  // namespace achilles::dynamics