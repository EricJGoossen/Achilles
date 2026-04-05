#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>

#include "geometry/frame.hpp"
#include "dynamics/joints/i_joint.hpp"
#include "dynamics/link.hpp"
#include "spatial/inertia.hpp"
#include "geometry/transform_tree.hpp"

namespace achilles::dynamics {

class JointTree {
public:
    JointTree() = default;

    void addJoint(std::unique_ptr<joints::IJoint> joint) {
        Link::Id child  = joint->child_link().id();
        Link::Id parent = joint->parent_link().id();

        auto [it, inserted] = child_to_joint_.emplace(child, std::move(joint));

        if (!inserted) {
            throw std::runtime_error("Joint already exists for link");
        }

        parent_to_children_.emplace(child, std::vector<Link::Id>());
        parent_to_children_[parent].push_back(child);
    }

    const joints::IJoint& getJoint(const Link& child_link) const {
        return *child_to_joint_.at(child_link.id());
    }

    std::unordered_map<Link::Id, spatial::Inertia> computeCompositeInertias(const dynamics::Link& root) const {
        std::unordered_map<Link::Id, spatial::Inertia> result;
        recursiveInertia(root, result);
        return result;
    }

    void propagateAccelerations(const Link& root) {
        for (Link::Id child : parent_to_children_[root.id()]) {
            joints::IJoint& joint = *child_to_joint_.at(child);
            recursiveAcceleration(joint, spatial::Jerk::identity());
        }
    }

    void integrate(double dt) {
        for (auto& [_, joint] : child_to_joint_) {
            joint->integrate(dt);
        }
    }

    void updateTransforms(geometry::TransformTree& transform_tree) {
        for (auto& [_, joint] : child_to_joint_) {
            const geometry::Frame& joint_frame = joint->frame();
            const geometry::Frame& child_frame = joint->child_link().frame();
            transform_tree.updateTransform(joint_frame, child_frame, joint->position());
        }
    }

private:
    void recursiveInertia(const Link& link, std::unordered_map<Link::Id, spatial::Inertia>& composite_inertias) const {
        spatial::Inertia composite_inertia = link.inertia();

        for (Link::Id child : parent_to_children_.at(link.id())) {
            auto& joint = *child_to_joint_.at(child);
            const Link& child_link = joint.child_link();   

            recursiveInertia(child_link, composite_inertias);
            composite_inertia += joint.solveInertia(composite_inertias[child]);
        }

        composite_inertias.emplace(link.id(), composite_inertia);
    }

    void recursiveAcceleration(joints::IJoint& joint, const spatial::Jerk& parent_acceleration) {
        joint.applyAcceleration(parent_acceleration);

        for (Link::Id child : parent_to_children_[joint.child_link().id()]) {
            joints::IJoint& child_joint = *child_to_joint_.at(child);
            recursiveAcceleration(child_joint, joint.acceleration());
        }
    }

    std::unordered_map<Link::Id, std::unique_ptr<joints::IJoint>> child_to_joint_;
    std::unordered_map<Link::Id, std::vector<Link::Id>> parent_to_children_;
};
}