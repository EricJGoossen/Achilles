#pragma once

#include <Eigen/Dense>
#include <utility>

#include "inertia.hpp"
#include "math/dual.hpp"
#include "math/vector.hpp"
#include "wrench.hpp"

namespace achilles::spatial {

class Surge : public math::Dual<Surge> {
    using Base = math::Dual<Surge>;
    using Storage = math::DualStorage;

  public:
    Surge(const Surge&) = default;
    Surge(Surge&&) = default;
    Surge& operator=(const Surge&) = default;
    Surge& operator=(Surge&&) = default;
    ~Surge() = default;

    Surge(math::Vector linear, math::Vector angular)
      : Base(std::move(linear), std::move(angular)) {}
    Surge(const Eigen::Matrix<double, 6, 1>& vec) : Base(vec) {}
    Surge(const Storage& storage) : Base(storage) {}

    static Surge zero() { return {math::Vector(), math::Vector()}; }
    static Surge fromWrench(const Wrench& wrench, const Inertia& inertia) {
        return {inertia.mat() * wrench.mat()};
    }

    inline void propagate(const Wrench& wrench, const Inertia& inertia) {
        *this += fromWrench(wrench, inertia);
    }
};
}  // namespace achilles::spatial