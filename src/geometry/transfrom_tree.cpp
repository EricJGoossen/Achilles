#include "geometry/frame.hpp"
#include "transform_tree.hpp"

namespace achilles::geometry {

void TransformTree::addTransform(std::unique_ptr<Transform> transform) {
    geometry::AbstractFrame child_frame = transform->childFrame();

    auto [it, inserted] =
        tree_.emplace(transform->childFrame(), std::move(transform));

    if (!inserted) {
        throw std::runtime_error("Transform already exists for frame");
    }
}

void TransformTree::updateTransform(
    const AbstractFrame& parent_frame,
    const AbstractFrame& child_frame,
    const spatial::Pose& new_pose
) {
    Transform& transform = *tree_.at(child_frame);

    if (transform.parentFrame() != parent_frame) {
        throw std::runtime_error(
            "No frame exists for the given parent-child pair"
        );
    }

    transform.update(new_pose);
}

const Transform& TransformTree::getTransform(
    const AbstractFrame& parent_frame, const AbstractFrame& child_frame
) const {
    Transform& transform = *tree_.at(child_frame);

    if (transform.parentFrame() != parent_frame) {
        throw std::runtime_error(
            "No frame exists for the given parent-child pair"
        );
    }

    return transform;
}
}  // namespace achilles::geometry