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

/**
 * AbstractFrame stores a pointer to a string with static or guaranteed-longer
 * lifetime (e.g. a string literal or a static registry entry). The pointer is
 * used for identity: two frames are equal iff they point to the same interned
 * string AND share the same Tag type.
 *
 * Never construct a Frame from a temporary or stack-allocated string.
 */
class AbstractFrame {
  public:
    AbstractFrame(const char* name, std::type_index type_index)
      : name_(name), type_index_(type_index) {}

    const char* name() const noexcept { return name_; }

    bool operator==(const AbstractFrame& other) const noexcept {
        return name_ == other.name_ && type_index_ == other.type_index_;
    }

    bool operator!=(const AbstractFrame& other) const noexcept {
        return !(*this == other);
    }

    template <typename Tag>
    Frame<Tag> as() const {
        assert(type_index_ == std::type_index(typeid(Tag)));
        return Frame<Tag>(name_);
    }

  private:
    std::size_t hash() const noexcept {
        std::size_t seed = std::hash<const char*>{}(name_);
        const std::size_t type_hash = type_index_.hash_code();
        seed ^= type_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }

    const char* name_;
    std::type_index type_index_;

    template <typename Tag>
    friend struct std::hash;
};

template <typename Tag>
class Frame : public AbstractFrame {
  public:
    explicit Frame(const char* name) : AbstractFrame(name, typeid(Tag)) {}

    bool operator==(const Frame& other) const noexcept {
        return AbstractFrame::operator==(other);
    }

    bool operator!=(const Frame& other) const noexcept {
        return !(*this == other);
    }
};

}  // namespace achilles::geometry

namespace std {

template <>
struct hash<achilles::geometry::AbstractFrame> {
    std::size_t operator()(const achilles::geometry::AbstractFrame& frame
    ) const noexcept {
        std::size_t seed = std::hash<const char*>{}(frame.name());
        const std::size_t type_hash = frame.type_index_.hash_code();
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