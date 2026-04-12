#pragma once

#include <Eigen/Dense>

#include "math/vector.hpp"

namespace achilles::spatial {

class Inertia {
  public:
    Inertia(const Inertia&) = default;
    Inertia(Inertia&&) = default;
    Inertia& operator=(const Inertia&) = default;
    Inertia& operator=(Inertia&&) = default;
    ~Inertia() = default;

    Inertia(
        const Eigen::Matrix3d& inertia,
        const math::Vector& com_offset,
        double mass
    )
      : data_() {
        Eigen::Matrix3d k = com_offset.skew();
        Eigen::Matrix3d kT = k.transpose();

        // Dual vectors are ordered [linear; angular], so inertia follows
        // that same block layout.
        data_.block<3, 3>(0, 0) = mass * Eigen::Matrix3d::Identity();
        data_.block<3, 3>(0, 3) = mass * kT;
        data_.block<3, 3>(3, 0) = mass * k;
        data_.block<3, 3>(3, 3) = inertia + mass * k * kT;
    }

    Inertia(Eigen::Matrix<double, 6, 6> inertia) : data_(std::move(inertia)) {}

    inline Eigen::Matrix<double, 6, 6> mat() const { return data_; }

    static Inertia identity() {
        return {Eigen::Matrix<double, 6, 6>::Identity()};
    }
    static Inertia zero() { return {Eigen::Matrix<double, 6, 6>::Zero()}; }

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
    Eigen::Matrix<double, 6, 6> data_;
};  // class Inertia

}  // namespace achilles::spatial