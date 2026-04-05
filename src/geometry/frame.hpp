#pragma once

#include <string>

#include "math/Id.hpp"

namespace achilles::geometry 
{

class Frame 
{
public:
    using Id = math::Id<Frame>;

    Frame(
        std::string name,
        int id) : 
        name_(name),
        id_(Id(id)) {}

    inline std::string name() const { return name_; }
    inline Id id() const { return id_; }
private:
    std::string name_;
    Id id_;
}; // class Frame

} // namespace achilles::geometry