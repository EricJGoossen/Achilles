#pragma once

#include <cassert>
#include <cstring>
#include <functional>
#include <optional>
#include <string>
#include <typeindex>

namespace achilles::geometry {

template <typename Tag>
class Frame;

class AbstractFrame {
  public:
    AbstractFrame(const char* name)
      : name_(name) {}

    const char* name() const noexcept { return name_; }

    bool operator==(const AbstractFrame& other) const noexcept {
        return name_ == other.name_ ;
    }

    bool operator!=(const AbstractFrame& other) const noexcept {
        return !(*this == other);
    }

    template <typename Tag>
    Frame<Tag> as() const {
        assert(std::type_index(typeid(Tag)) == std::type_index(typeid(Tag)));
        return static_cast<Frame<Tag>>(*this);
    }

  private:
    const char* name_;

    template <typename Tag>
    friend struct std::hash;
};

template <typename Tag>
class Frame : public AbstractFrame {
  public:
    explicit Frame(const char* name) : AbstractFrame(name) {}
};

}  // namespace achilles::geometry

namespace std {

template <>
struct hash<achilles::geometry::AbstractFrame> {
    std::size_t operator()(const achilles::geometry::AbstractFrame& frame
    ) const noexcept {
        std::size_t seed = std::hash<const char*>{}(frame.name());
        std::size_t type_hash = std::type_index(typeid(frame)).hash_code();
        seed ^= type_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

template <typename Tag>
struct hash<achilles::geometry::Frame<Tag>> {
    std::size_t operator()(const achilles::geometry::Frame<Tag>& frame
    ) const noexcept {
        return hash<achilles::geometry::AbstractFrame>{}(frame);
    }
};

}  // namespace std