#pragma once

#include "spatial/pose.hpp"
#include "frame.hpp"

namespace achilles::geometry 
{

class Transform 
{
    using Pose = spatial::Pose;

public:
    Transform(
        const Frame& parent_frame,
        const Frame& child_frame,
        const Pose pose) : 
        parent_frame_(parent_frame), 
        child_frame_(child_frame),
        pose_(pose)
        {}

    inline const Pose& pose() const { return pose_; }
    inline const Frame& parent_frame() const { return parent_frame_; }
    inline const Frame& child_frame() const { return child_frame_; }

    inline void update(const Pose& pose) { pose_ = pose; }
    inline void add(const Pose& delta) { pose_ = pose_ * delta; }
private:
    const Frame& parent_frame_;
    const Frame& child_frame_;

    Pose pose_;
}; // class Transform

} // namespace achilles::geometry