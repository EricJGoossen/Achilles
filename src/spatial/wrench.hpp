#pragma once

#include "math/vector.hpp"
#include "math/dual.hpp"

namespace achilles::spatial {

class Wrench : public math::Dual<Wrench> {
    using Base = math::Dual<Wrench>;

public:
    Wrench(const math::Vector& linear, const math::Vector& angular)
        : Base(linear, angular) {}

    Wrench(Eigen::Matrix<double, 6, 1> vec) : Base(vec) {}
    

    Wrench(const math::DualStorage& storage) : Base(storage) {}

    Wrench(const Wrench& other) = default;
}; // class Wrench
} // namespace achilles::spatial
