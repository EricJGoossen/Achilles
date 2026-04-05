#pragma once

#include <Eigen/Geometry>

#include "vector.hpp"

namespace achilles::math 
{
class Quaternion 
{
public:
    Quaternion() = default;
    Quaternion(double w, double x, double y, double z) : data_(w, x, y, z) {}
    Quaternion(const Quaternion& other) = default;
    Quaternion(const Eigen::Quaterniond& quat) : data_(quat) {}
    Quaternion(const Eigen::Matrix3d& rot) : data_(rot) {}

    inline static Quaternion identity() { return Quaternion(1, 0, 0, 0); }

    inline Eigen::Quaterniond mat() const { return data_; }
    inline double w() const { return data_.w(); }
    inline double x() const { return data_.x(); }
    inline double y() const { return data_.y(); }
    inline double z() const { return data_.z(); }

    inline Quaternion& operator+=(const Quaternion& other) { data_.coeffs() += other.data_.coeffs(); return *this; }
    Quaternion& operator*=(const Quaternion& other);
    inline Quaternion operator+(const Quaternion& other) const { return Quaternion(*this) += other; }
    inline Quaternion operator*(const Quaternion& other) const { return Quaternion(*this) *= other; }

    inline Quaternion conjugate() const { return Quaternion(data_.conjugate()); }
    Quaternion inverse() const;

    Vector rotate(const Vector& vec) const;
    void propagate(const Vector& angular_velocity, double dt);

    inline double mag() const { return data_.norm(); }
    inline void normalize() { data_.normalize(); }
    inline Quaternion normalized() const { return Quaternion(data_.normalized()); }
private:
    Eigen::Quaterniond data_;
    static constexpr double epsilon = 1e-9;
}; // class Quaternion

} // namespace achilles::math