#pragma once

#include "frame.hpp"
#include "spatial/pose.hpp"

namespace achilles::geometry {

class Transform {
    using Pose = spatial::Pose;

  public:
    Transform(
        const AbstractFrame& parent_frame,
        const AbstractFrame& child_frame,
        Pose pose
    )
      : parent_frame_(&parent_frame),
        child_frame_(&child_frame),
        pose_(std::move(pose)) {}

    inline const Pose& pose() const { return pose_; }
    inline const AbstractFrame& parentFrame() const { return *parent_frame_; }
    inline const AbstractFrame& childFrame() const { return *child_frame_; }

    inline void update(const Pose& pose) { pose_ = pose; }
    inline void add(const Pose& delta) { pose_ = pose_ * delta; }

  private:
    const AbstractFrame* parent_frame_;
    const AbstractFrame* child_frame_;

    Pose pose_;
};  // class Transform

}  // namespace achilles::geometry