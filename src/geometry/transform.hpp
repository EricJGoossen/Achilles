#pragma once

#include "frame.hpp"
#include "spatial/pose.hpp"

namespace achilles::geometry {

class Transform {
    using Pose = spatial::Pose;

  public:
    Transform(AbstractFrame parent_frame, AbstractFrame child_frame, Pose pose)
      : parent_frame_(parent_frame),
        child_frame_(child_frame),
        pose_(std::move(pose)) {}

    const Pose& pose() const { return pose_; }
    const AbstractFrame parentFrame() const { return parent_frame_; }
    const AbstractFrame childFrame() const { return child_frame_; }

    void update(const Pose& pose) { pose_ = pose; }
    void add(const Pose& delta) { pose_ = pose_ * delta; }

  private:
    AbstractFrame parent_frame_;
    AbstractFrame child_frame_;

    Pose pose_;
};  // class Transform

}  // namespace achilles::geometry