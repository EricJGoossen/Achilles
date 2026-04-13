#pragma once

#include <utility>

#include "geometry/frame.hpp"
#include "spatial/inertia.hpp"

namespace achilles::dynamics {

class Link {
  public:
    using Frame = geometry::Frame<Link>;

    Link(char* frame, spatial::Inertia inertia)
      : frame_(frame), inertia_(std::move(inertia)) {}

    inline const Frame& frame() const { return frame_; }
    inline const spatial::Inertia& inertia() const { return inertia_; }

  private:
    Frame frame_;
    spatial::Inertia inertia_;
};

}  // namespace achilles::dynamics