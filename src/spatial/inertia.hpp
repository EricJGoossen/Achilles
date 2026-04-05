#pragma once

#include <Eigen/Dense>

#include "math/vector.hpp"

namespace achilles::spatial {

class Inertia {
    using Inertia3d = Eigen::Matrix<double, 6, 6>;

public:
    Inertia() = default;
    Inertia(const Inertia& other) = default;
    Inertia(const Inertia3d& inrt) : data_(inrt) {}

    static Inertia identity() { return Inertia(Eigen::Matrix<double, 6, 6>::Identity()); }

    inline Inertia3d mat() const { return data_; }

    inline Inertia operator+=(const Inertia& other) {
        data_ += other.data_;
        return *this;
    }

    inline Inertia operator-=(const Inertia& other) {
        data_ -= other.data_;
        return *this;
    }

    inline Inertia operator+(const Inertia& other) const {
        return Inertia(*this) += other;
    }

    inline Inertia operator-(const Inertia& other) const {
        return Inertia(*this) -= other;
    }

private:
    Inertia3d data_;
}; // class Inertia

} // namespace achilles::spatial