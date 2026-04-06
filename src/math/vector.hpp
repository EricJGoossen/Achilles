#pragma once

#include <Eigen/Dense>
#include <utility>

namespace achilles::math {
class Vector {
  public:
    Vector() = default;
    Vector(const Vector&) = default;
    Vector(Vector&&) = default;
    Vector& operator=(const Vector&) = default;
    Vector& operator=(Vector&&) = default;
    ~Vector() = default;

    Vector(double x, double y, double z) : data_(x, y, z) {}
    Vector(Eigen::Vector3d vec) : data_(std::move(vec)) {}

    inline static Vector zero() { return Vector(0, 0, 0); }

    inline const Eigen::Vector3d& mat() const { return data_; }
    inline double x() const { return data_.x(); }
    inline double y() const { return data_.y(); }
    inline double z() const { return data_.z(); }

    inline Vector& operator+=(const Vector& other) {
        data_ += other.data_;
        return *this;
    }
    inline Vector& operator-=(const Vector& other) {
        data_ -= other.data_;
        return *this;
    }
    inline Vector& operator*=(double scalar) {
        data_ *= scalar;
        return *this;
    }
    inline Vector& operator/=(double scalar) {
        data_ /= scalar;
        return *this;
    }

    inline Vector operator+(const Vector& other) const { return Vector(*this) += other; }
    inline Vector operator-(const Vector& other) const { return Vector(*this) -= other; }
    inline Vector operator*(double scalar) const { return Vector(*this) *= scalar; }
    inline Vector operator/(double scalar) const { return Vector(*this) /= scalar; }

    inline double dot(const Vector& other) const { return data_.dot(other.data_); }
    inline Vector cross(const Vector& other) const { return Vector(data_.cross(other.data_)); }

    inline Eigen::Matrix3d skew() const {
        Eigen::Matrix3d m;
        m << 0, -data_.z(), data_.y(), data_.z(), 0, -data_.x(), -data_.y(), data_.x(), 0;
        return m;
    }

    inline double mag() const { return data_.norm(); }
    inline void normalize() { data_.normalize(); }
    inline Vector normalized() const { return Vector(data_.normalized()); }

    inline Vector inverse() const { return Vector(-data_); }
    inline void propagate(const Vector& derivative, double dt) { (*this) += derivative * dt; }

  private:
    Eigen::Vector3d data_;
};  // class Vector
}  // namespace achilles::math