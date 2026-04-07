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
    Jerk(const Eigen::Matrix<double, 6, 1>& vec) : Base(vec) {}
    Jerk(const Storage& storage) : Base(storage) {}

    static Jerk zero() { return {math::Vector(), math::Vector()}; }
    static Jerk fromWrench(const Wrench& wrench, const Inertia& inertia) {
        return {inertia.mat() * wrench.mat()};
    }

    inline void propagate(const Wrench& wrench, const Inertia& inertia) {
        *this += fromWrench(wrench, inertia);
    }
};
}  // namespace achilles::spatial