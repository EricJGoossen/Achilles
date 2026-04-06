#pragma once

#include "dynamics/link.hpp"
#include "geometry/frame.hpp"
#include "geometry/transform.hpp"
#include "spatial/inertia.hpp"
#include "spatial/jerk.hpp"
#include "spatial/pose.hpp"
#include "spatial/twist.hpp"

namespace achilles::dynamics::joints {

class IJoint {
  public:
    virtual const geometry::Frame& frame() = 0;
    virtual const Link& parent_link() = 0;
    virtual const Link& child_link() = 0;

    virtual const spatial::Pose& position() = 0;
    virtual const spatial::Twist& velocity() = 0;
    virtual const spatial::Jerk& acceleration() = 0;

    virtual spatial::Inertia solveInertia(
        const spatial::Inertia& child_relative_inertia
    ) const = 0;
    virtual void applyAcceleration(const spatial::Jerk& delta_acceleration) = 0;
    virtual void integrate(double dt) = 0;
};
}  // namespace achilles::dynamics::joints