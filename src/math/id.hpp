#pragma once

#include <functional>

namespace achilles::math {

template <typename tag>
struct Id {
public:
    Id(int id) : id_(id) {}
    bool operator==(const Id& other) const { return id_ == other.id_; }
    bool operator!=(const Id& other) const { return id_ != other.id_; }
    constexpr int id() const { return id_; }
private:
    int id_;
}; // class Id
} // namespace achilles::math

namespace std {

template <typename tag>
struct hash<achilles::math::Id<tag>> {
    std::size_t operator()(const achilles::math::Id<tag>& id) const noexcept {
        return std::hash<int>{}(id.id());
    }
}; // struct hash
} // namespace std