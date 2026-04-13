#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "control/actuators/actuator.hpp"
#include "dynamics/joint_tree.hpp"
#include "dynamics/link.hpp"
#include "geometry/transform.hpp"
#include "geometry/transform_tree.hpp"

namespace achilles::control {

struct PlantState {
    geometry::TransformTree transform_tree;
    dynamics::JointTree joint_tree;
    std::unique_ptr<geometry::Transform> world_transform;
};
}  // namespace achilles::control