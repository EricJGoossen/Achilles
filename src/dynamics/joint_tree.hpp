#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dynamics/joints/i_joint.hpp"
#include "dynamics/link.hpp"
#include "geometry/frame.hpp"
#include "geometry/transform_tree.hpp"
#include "spatial/inertia.hpp"

namespace achilles::dynamics {

class JointTree {
    using InertiaMap = std::unordered_map<Link::Id, spatial::Inertia>;

  public:
    JointTree() = default;

    void addJoint(std::unique_ptr<joints::IJoint> joint);
    const joints::IJoint& getJoint(const Link& child_link) const;

    InertiaMap computeCompositeInertias(const dynamics::Link& root) const;
    void propagateAccelerations(const Link& root);

    void integrate(double dt);
    void updateTransforms(geometry::TransformTree& transform_tree);

  private:
    void recursiveInertia(const Link& link, InertiaMap& composite_inertias)
        const;

    void recursiveAcceleration(
        joints::IJoint& joint, const spatial::Jerk& parent_acceleration
    );

    std::unordered_map<Link::Id, std::unique_ptr<joints::IJoint>>
        child_to_joint_;
    std::unordered_map<Link::Id, std::vector<Link::Id>> parent_to_children_;
};
}  // namespace achilles::dynamics