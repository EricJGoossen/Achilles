#pragma once

#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "frame.hpp"
#include "spatial/pose.hpp"
#include "transform.hpp"

namespace achilles::geometry {

class TransformTree {
  public:
    TransformTree() = default;

    void addTransform(std::unique_ptr<Transform> transform);

    void updateTransform(
        const Frame& parent_frame,
        const Frame& child_frame,
        const spatial::Pose& new_pose
    );
    const Transform& getTransform(
        const Frame& parent_frame, const Frame& child_frame
    ) const;

  private:
    std::unordered_map<Frame::Id, std::unique_ptr<Transform>> tree_;
};  // class TransformTree

}  // namespace achilles::geometry