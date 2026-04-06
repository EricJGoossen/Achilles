#pragma once

#include <Eigen/Geometry>
#include <utility>

#include "vector.hpp"

namespace achilles::math {
class Quaternion {
    static constexpr double EPSILON = 1e-9;

  public:
    Quaternion() = default;
    Quaternion(const Quaternion&) = default;
    Quaternion(Quaternion&&) = default;
    Quaternion& operator=(const Quaternion&) = default;
    Quaternion& operator=(Quaternion&&) = default;
    ~Quaternion() = default;

    Quaternion(double w, double x, double y, double z) : data_(w, x, y, z) {}
    Quaternion(Eigen::Quaterniond quat) : data_(std::move(quat)) {}
    Quaternion(const Eigen::Matrix3d& rot) : data_(rot) {}

    inline static Quaternion identity() { return {1, 0, 0, 0}; }

    inline const Eigen::Quaterniond& mat() const { return data_; }
    inline double w() const { return data_.w(); }
    inline double x() const { return data_.x(); }
    inline double y() const { return data_.y(); }
    inline double z() const { return data_.z(); }

    inline Quaternion& operator+=(const Quaternion& other) {
        data_.coeffs() += other.data_.coeffs();
        return *this;
    }
    inline Quaternion& operator*=(const Quaternion& other) {
        data_ *= other.data_;
        return *this;
    }

    inline Quaternion operator+(const Quaternion& other) const {
        return Quaternion(*this) += other;
    }
    inline Quaternion operator*(const Quaternion& other) const {
        return Quaternion(*this) *= other;
    }

    inline Quaternion conjugate() const { return data_.conjugate(); }
    Quaternion inverse() const { return data_.inverse(); }

    Vector rotate(const Vector& vec) const;
    void propagate(const Vector& angular_velocity, double dt);

    inline double mag() const { return data_.norm(); }
    inline void normalize() { data_.normalize(); }
    inline Quaternion normalized() const { return data_.normalized(); }

  private:
    Eigen::Quaterniond data_;
};  // class Quaternion

}  // namespace achilles::math