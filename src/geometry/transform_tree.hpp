#pragma once

#include <memory>
#include <unordered_map>

#include "frame.hpp"
#include "spatial/pose.hpp"
#include "transform.hpp"

namespace achilles::geometry {

class TransformTree {
  public:
    TransformTree() = default;

    void addTransform(std::unique_ptr<Transform> transform);

    void updateTransform(
        const AbstractFrame& parent_frame,
        const AbstractFrame& child_frame,
        const spatial::Pose& new_pose
    );
    const Transform& getTransform(
        const AbstractFrame& parent_frame, const AbstractFrame& child_frame
    ) const;

  private:
    std::unordered_map<AbstractFrame, std::unique_ptr<Transform>> tree_;
};  // class TransformTree

}  // namespace achilles::geometry