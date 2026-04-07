#pragma once

#include "dynamics/link.hpp"
#include "geometry/frame.hpp"
#include "spatial/inertia.hpp"
#include "spatial/jerk.hpp"
#include "spatial/pose.hpp"
#include "spatial/twist.hpp"

namespace achilles::dynamics::joints {

class AbstractJoint {
  public:
    AbstractJoint(const AbstractJoint&) = delete;
    AbstractJoint(AbstractJoint&&) = default;
    AbstractJoint& operator=(const AbstractJoint&) = delete;
    AbstractJoint& operator=(AbstractJoint&&) = default;
    virtual ~AbstractJoint() = default;

    AbstractJoint(
        const geometry::Frame& frame,
        const Link& parent_link,
        const Link& child_link,
        spatial::Pose initial_position,
        spatial::Twist initial_velocity
    )
      : frame_(&frame),
        parent_link_(&parent_link),
        child_link_(&child_link),
        position_cache_(std::move(initial_position)),
        velocity_cache_(std::move(initial_velocity)),
        acceleration_cache_(spatial::Jerk::zero()) {}

    inline const geometry::Frame& frame() const { return *frame_; }
    inline const Link& parentLink() const { return *parent_link_; }
    inline const Link& childLink() const { return *child_link_; }

    inline const spatial::Pose& position() const { return position_cache_; }
    inline const spatial::Twist& velocity() const { return velocity_cache_; }
    inline const spatial::Jerk& acceleration() const {
        return acceleration_cache_;
    }

    virtual spatial::Inertia solveInertia(const spatial::Inertia& child_inertia
    ) const = 0;
    virtual void applyAcceleration(const spatial::Jerk& delta_acceleration) = 0;
    virtual void integrate(double dt) = 0;

  protected:
    const geometry::Frame* frame_;
    const Link* parent_link_;
    const Link* child_link_;

    spatial::Pose position_cache_;
    spatial::Twist velocity_cache_;
    spatial::Jerk acceleration_cache_;
};
}  // namespace achilles::dynamics::joints