#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "control/actuators/actuator.hpp"
#include "control/plant_state.hpp"
#include "dynamics/joint_tree.hpp"
#include "dynamics/link.hpp"
#include "geometry/frame.hpp"
#include "geometry/transform.hpp"
#include "geometry/transform_tree.hpp"

namespace achilles::control {

class Plant {
    using InertiaMap =
        std::unordered_map<dynamics::Link::Frame, spatial::Inertia>;

  public:
    using Frame = geometry::Frame<Plant>;

    Plant(
        const char* frame,
        geometry::TransformTree transform_tree,
        dynamics::JointTree joint_tree,
        std::unique_ptr<geometry::Transform> base_link_transform,
        std::unique_ptr<geometry::Transform> world_transform,
        std::unordered_map<
            actuators::Actuator::Frame,
            std::unique_ptr<actuators::Actuator>> actuators,
        std::unordered_map<
            dynamics::Link::Frame,
            std::unique_ptr<dynamics::Link>> links
    )
      : frame_(frame),
        state_{
            std::move(transform_tree),
            std::move(joint_tree),
            std::move(world_transform)},
        actuators_(std::move(actuators)),
        links_(std::move(links)),
        base_link_(
            *links_.at(base_link_transform->childFrame().as<dynamics::Link>())
        ) {
        state_.transform_tree.addTransform(std::move(base_link_transform));
    }

    const Frame& frame() const { return frame_; }

    void update(double dt) {
        InertiaMap composite_inertias =
            state_.joint_tree.computeCompositeInertias(base_link_);

        for (auto& [_, actuator] : actuators_) {
            actuator->actuate(composite_inertias.at(actuator->childLink().frame(
            )));
        }

        state_.joint_tree.propagateAccelerations(base_link_);
        state_.joint_tree.integrate(dt);
        state_.joint_tree.updateTransforms(state_.transform_tree);
    }

    std::vector<control::actuators::Actuator*> getActuators() {
        std::vector<control::actuators::Actuator*> actuator_ptrs;
        actuator_ptrs.reserve(actuators_.size());

        for (const auto& [_, actuator] : actuators_) {
            actuator_ptrs.push_back(actuator.get());
        }

        return actuator_ptrs;
    }

  private:
    Frame frame_;

    PlantState state_;

    std::unordered_map<
        actuators::Actuator::Frame,
        std::unique_ptr<actuators::Actuator>>
        actuators_;
    std::unordered_map<dynamics::Link::Frame, std::unique_ptr<dynamics::Link>>
        links_;

    const dynamics::Link& base_link_;
};
}  // namespace achilles::control