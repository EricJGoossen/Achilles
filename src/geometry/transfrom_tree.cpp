#include "transform_tree.hpp"

namespace achilles::geometry {

void TransformTree::addTransform(std::unique_ptr<Transform> transform) {
    geometry::Frame child_frame = transform->childFrame();

    auto [it, inserted] =
        tree_.emplace(transform->childFrame().id(), std::move(transform));

    if (!inserted) {
        throw std::runtime_error("Transform already exists for frame");
    }
}

void TransformTree::updateTransform(
    const Frame& parent_frame,
    const Frame& child_frame,
    const spatial::Pose& new_pose
) {
    Transform& transform = *tree_.at(child_frame.id());

    if (transform.parentFrame().id() != parent_frame.id()) {
        throw std::runtime_error(
            "No frame exists for the given parent-child pair"
        );
    }

    transform.update(new_pose);
}

const Transform& TransformTree::getTransform(
    const Frame& parent_frame, const Frame& child_frame
) const {
    Transform& transform = *tree_.at(child_frame.id());

    if (transform.parentFrame().id() != parent_frame.id()) {
        throw std::runtime_error(
            "No frame exists for the given parent-child pair"
        );
    }

    return transform;
}
}  // namespace achilles::geometry