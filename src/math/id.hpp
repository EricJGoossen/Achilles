#pragma once

#include <functional>

namespace achilles::math {

template <typename Tag>
struct Id {
  public:
    class IdManager {
      public:
        IdManager() = default;

      private:
        friend class Id;
        size_t nextId() { return next_id_++; }

        size_t next_id_ = 0;
    };

    Id(const Id&) = default;
    Id(Id&&) = default;
    Id& operator=(const Id&) = default;
    Id& operator=(Id&&) = default;
    ~Id() = default;

    Id(IdManager& manager) : id_(manager.nextId()) {}

    bool operator==(const Id& other) const { return id_ == other.id_; }
    bool operator!=(const Id& other) const { return id_ != other.id_; }

    constexpr size_t id() const { return id_; }

  private:
    size_t id_;
};
}  // namespace achilles::math

namespace std {

template <typename Tag>
struct hash<achilles::math::Id<Tag>> {
    std::size_t operator()(const achilles::math::Id<Tag>& id) const noexcept {
        return std::hash<int>{}(id.id());
    }
};  // struct hash
}  // namespace std