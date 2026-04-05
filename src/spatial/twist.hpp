#pragma once

#include "math/vector.hpp"
#include "math/dual.hpp"
#include "jerk.hpp"

namespace achilles::spatial 
{

class Twist : public math::Dual<Twist>
{
    using Base = math::Dual<Twist>;
    using Storage = math::DualStorage;

public:
    Twist(const math::Vector& linear, const math::Vector& angular)
        : Base(linear, angular) {}

    Twist(Eigen::Matrix<double, 6, 1> vec) : Base(vec) {}

    Twist(const Storage& storage) : Base(storage) {}

    Twist(const Twist& other) = default;

    static Twist identity() { return Twist(math::Vector(), math::Vector()); }

    inline void propagate(const Jerk& derivative, double dt) { 
        Storage::linear.propagate(derivative.linear(), dt); 
        Storage::angular.propagate(derivative.angular(), dt); 
    }
}; // class Twist

} // namespace achilles::spatial