#pragma once

#include <Eigen/Dense>

#include "math/vector.hpp"
#include "math/dual.hpp"
#include "wrench.hpp"
#include "inertia.hpp"

namespace achilles::spatial {

class Jerk : public math::Dual<Jerk> {
    using Base = math::Dual<Jerk>;
    using Storage = math::DualStorage;

public:
    Jerk(const math::Vector& linear, const math::Vector& angular)
        : Base(linear, angular) {}
    Jerk(const Storage& storage) : Base(storage) {}
    Jerk(Eigen::Matrix<double, 6, 1> vec) : Base(vec) {}
    Jerk(const Wrench& wrench, const Inertia& inertia) : Base(inertia.mat() * wrench.mat()) {}
    Jerk(const Jerk& other) = default;

    static Jerk identity() { return Jerk(math::Vector(), math::Vector()); }

    inline void propagate(const Wrench& wrench, const Inertia& inertia) {
        *this += Jerk(inertia.mat() * wrench.mat());
    }
    
}; // class Jerk

} // namespace achilles::spatial