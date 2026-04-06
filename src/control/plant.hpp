#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "control/actuator.hpp"
#include "dynamics/joint_tree.hpp"
#include "dynamics/link.hpp"
#include "geometry/frame.hpp"
#include "geometry/transform_tree.hpp"

namespace achilles::control {

class Plant {
    using InertiaMap = std::unordered_map<dynamics::Link::Id, spatial::Inertia>;

  public:
    Plant(
        geometry::TransformTree transform_tree,
        dynamics::JointTree joint_tree,
        std::vector<dynamics::Link> links,
        std::vector<geometry::Frame> frames,
        std::vector<Actuator> actuators
    )
      : transform_tree_(std::move(transform_tree)),
        joint_tree_(std::move(joint_tree)),
        links_(std::move(links)),
        frames_(std::move(frames)),
        actuators_(std::move(actuators)) {}

    void update(double dt) {
        InertiaMap composite_inertias =
            joint_tree_.computeCompositeInertias(links_[0]);

        for (Actuator& actuator : actuators_) {
            actuator.actuate(composite_inertias.at(actuator.childLink().id()));
        }

        joint_tree_.propagateAccelerations(links_[0]);
        joint_tree_.integrate(dt);
        joint_tree_.updateTransforms(transform_tree_);
    }

  private:
    geometry::TransformTree transform_tree_;
    dynamics::JointTree joint_tree_;
    std::vector<dynamics::Link> links_;
    std::vector<geometry::Frame> frames_;
    std::vector<Actuator> actuators_;
};
}  // namespace achilles::control