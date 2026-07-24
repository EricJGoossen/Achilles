#include "joint_tree.hpp"

#include "dynamics/joints/abstract_joint.hpp"

namespace achilles::dynamics {
using InertiaMap =
    std::unordered_map<joints::AbstractJoint::Frame, spatial::Inertia>;
using LinkMap = std::unordered_map<Link::Frame, std::unique_ptr<Link>>;

void JointTree::addJoint(std::unique_ptr<joints::AbstractJoint> joint) {
    Link::Frame child = joint->childLink();

    Link::Frame parent = joint->parentLink();

    auto [it, inserted] = child_to_joint_.emplace(child, std::move(joint));

    if (!inserted) {
        throw std::runtime_error("Joint already exists for link");
    }

    parent_to_children_.emplace(child, std::vector<Link::Frame>());
    parent_to_children_[parent].push_back(child);
}

const joints::AbstractJoint& JointTree::getJoint(Link::Frame child_link) const {
    return *child_to_joint_.at(child_link);
}

InertiaMap JointTree::computeCompositeInertias(
    const LinkMap& link_map, Link::Frame root
) const {
    InertiaMap result;
    recursiveInertia(link_map, root, result);
    return result;
}

void JointTree::propagateAccelerations(Link::Frame root) {
    const auto& children = parent_to_children_[root];
    assert(children.size() == 1 && "Root must have exactly one child");

    joints::AbstractJoint& joint = *child_to_joint_.at(children.front());
    recursiveAcceleration(joint, spatial::Surge::zero());
}

void JointTree::integrate(double dt) {
    for (auto& [_, joint] : child_to_joint_) {
        joint->integrate(dt);
    }
}

void JointTree::updateTransforms(geometry::TransformTree& transform_tree) {
    for (auto& [_, joint] : child_to_joint_) {
        const joints::AbstractJoint::Frame& joint_frame = joint->frame();
        const Link::Frame& child_frame = joint->childLink();

        transform_tree.updateTransform(
            joint_frame, child_frame, joint->position()
        );
    }
}

void JointTree::recursiveInertia(
    const LinkMap& link_map,
    joints::AbstractJoint::Frame joint,
    InertiaMap& composite_inertias
) const {
    spatial::Inertia composite_inertia = link_map.at(link)->inertia();

    for (const Link::Frame& child : parent_to_children_.at(link)) {
        auto& joint = *child_to_joint_.at(child);

        recursiveInertia(link_map, joint.childLink(), composite_inertias);
        composite_inertia += joint.solveInertia(composite_inertias.at(child));
    }

    composite_inertias.emplace(link, composite_inertia);
}

void JointTree::recursiveAcceleration(
    joints::AbstractJoint& joint, const spatial::Surge& parent_acceleration
) {
    joint.applyAcceleration(parent_acceleration);

    for (const Link::Frame& child : parent_to_children_[joint.childLink()]) {
        joints::AbstractJoint& child_joint = *child_to_joint_.at(child);
        recursiveAcceleration(child_joint, joint.acceleration());
    }
}

}  // namespace achilles::dynamics