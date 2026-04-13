#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "dynamics/joints/abstract_joint.hpp"
#include "dynamics/link.hpp"
#include "geometry/transform_tree.hpp"
#include "spatial/inertia.hpp"

namespace achilles::dynamics {

class JointTree {
    using InertiaMap = std::unordered_map<Link::Frame, spatial::Inertia>;

  public:
    JointTree() = default;

    void addJoint(std::unique_ptr<joints::AbstractJoint> joint);
    const joints::AbstractJoint& getJoint(const Link& child_link) const;

    InertiaMap computeCompositeInertias(const Link& root) const;
    void propagateAccelerations(const Link& root);

    void integrate(double dt);
    void updateTransforms(geometry::TransformTree& transform_tree);

  private:
    void recursiveInertia(const Link& link, InertiaMap& composite_inertias)
        const;

    void recursiveAcceleration(
        joints::AbstractJoint& joint, const spatial::Surge& parent_acceleration
    );

    std::unordered_map<Link::Frame, std::unique_ptr<joints::AbstractJoint>>
        child_to_joint_;
    std::unordered_map<Link::Frame, std::vector<Link::Frame>>
        parent_to_children_;
};
}  // namespace achilles::dynamics