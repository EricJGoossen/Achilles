#pragma once

#include <Eigen/Dense>
#include <utility>

#include "jerk.hpp"
#include "math/dual.hpp"
#include "math/vector.hpp"

namespace achilles::spatial {

class Twist : public math::Dual<Twist> {
    using Base = math::Dual<Twist>;
    using Storage = math::DualStorage;

  public:
    Twist(const Twist&) = default;
    Twist(Twist&&) = default;
    Twist& operator=(const Twist&) = default;
    Twist& operator=(Twist&&) = default;
    ~Twist() = default;

    Twist(math::Vector linear, math::Vector angular)
      : Base(std::move(linear), std::move(angular)) {}
    Twist(Eigen::Matrix<double, 6, 1> vec) : Base(vec) {}
    Twist(const Storage& storage) : Base(storage) {}

    static Twist identity() { return Twist(math::Vector(), math::Vector()); }

    inline void propagate(const Jerk& derivative, double dt) {
        Storage::linear.propagate(derivative.linear(), dt);
        Storage::angular.propagate(derivative.angular(), dt);
    }
};  // class Twist

}  // namespace achilles::spatial