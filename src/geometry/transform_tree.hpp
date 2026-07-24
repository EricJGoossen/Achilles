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

    void addTransform(Transform transform);

    void updateTransform(
        AbstractFrame parent_frame,
        AbstractFrame child_frame,
        const spatial::Pose& new_pose
    );
    const Transform& getTransform(
        AbstractFrame parent_frame, AbstractFrame child_frame
    ) const;

  private:
    std::unordered_map<AbstractFrame, std::unique_ptr<Transform>> tree_;
};  // class TransformTree

}  // namespace achilles::geometry