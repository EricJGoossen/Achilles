#pragma once

#include <Eigen/Dense>
#include <utility>

#include "inertia.hpp"
#include "math/dual.hpp"
#include "math/vector.hpp"
#include "wrench.hpp"

namespace achilles::spatial {

class Jerk : public math::Dual<Jerk> {
    using Base = math::Dual<Jerk>;
    using Storage = math::DualStorage;

  public:
    Jerk(const Jerk&) = default;
    Jerk(Jerk&&) = default;
    Jerk& operator=(const Jerk&) = default;
    Jerk& operator=(Jerk&&) = default;
    ~Jerk() = default;

    Jerk(math::Vector linear, math::Vector angular)
      : Base(std::move(linear), std::move(angular)) {}
    Jerk(Eigen::Matrix<double, 6, 1> vec) : Base(vec) {}
    Jerk(const Wrench& wrench, const Inertia& inertia)
      : Base(inertia.mat() * wrench.mat()) {}
    Jerk(const Storage& storage) : Base(storage) {}

    static Jerk zero() { return Jerk(math::Vector(), math::Vector()); }

    inline void propagate(const Wrench& wrench, const Inertia& inertia) {
        *this += Jerk(inertia.mat() * wrench.mat());
    }
};  // class Jerk
}  // namespace achilles::spatial