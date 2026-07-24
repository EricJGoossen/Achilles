#include "geometry/frame.hpp"
#include "transform_tree.hpp"

namespace achilles::geometry {

void TransformTree::addTransform(Transform transform) {
    auto [it, inserted] = 
        tree_.emplace(transform.childFrame(), std::move(transform));

    if (!inserted) {
        throw std::runtime_error("Transform already exists for frame");
    }
}

void TransformTree::updateTransform(
    AbstractFrame parent_frame,
    AbstractFrame child_frame,
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
    AbstractFrame parent_frame, AbstractFrame child_frame
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