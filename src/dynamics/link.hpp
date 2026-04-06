#pragma once

#include <utility>

#include "geometry/frame.hpp"
#include "math/id.hpp"
#include "math/vector.hpp"
#include "spatial/inertia.hpp"

namespace achilles::dynamics {

class Link {
  public:
    using Id = math::Id<Link>;

    Link(const geometry::Frame& frame, spatial::Inertia inertia, Id id)
      : frame_(frame), inertia_(std::move(inertia)), id_(std::move(id)) {}

    inline const geometry::Frame& frame() const { return frame_; }
    inline const spatial::Inertia& inertia() const { return inertia_; }
    inline const Id id() const { return id_; }

  private:
    const geometry::Frame& frame_;
    const spatial::Inertia inertia_;
    const Id id_;
};  // class Link

}  // namespace achilles::dynamics