#pragma once

#include "math/vector.hpp"
#include "math/Id.hpp"
#include "geometry/frame.hpp"
#include "spatial/inertia.hpp"

namespace achilles::dynamics {

class Link {
public:
    using Id = math::Id<Link>;

    Link(const geometry::Frame frame,
        const spatial::Inertia inertia,
        const int id) : 
        frame_(frame),
        inertia_(inertia),
        id_(Id(id)) {}

    inline const geometry::Frame& frame() const { return frame_; }
    inline const spatial::Inertia& inertia() const { return inertia_; }
    inline const Id id() const { return id_;}

private:
    const geometry::Frame& frame_;
    const spatial::Inertia inertia_;
    const Id id_;
}; // class Link

} // namespace achilles::dynamics