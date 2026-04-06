#pragma once

#include <string>
#include <utility>

#include "math/id.hpp"

namespace achilles::geometry {

class Frame {
  public:
    using Id = math::Id<Frame>;

    Frame(std::string name, Id id) : name_(name), id_(std::move(id)) {}

    inline std::string name() const { return name_; }
    inline Id id() const { return id_; }

  private:
    std::string name_;
    Id id_;
};  // class Frame

}  // namespace achilles::geometry