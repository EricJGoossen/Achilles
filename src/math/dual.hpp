#pragma once

#include <Eigen/Dense>
#include <utility>

#include "vector.hpp"

namespace achilles::math {

struct DualStorage {
    DualStorage(const DualStorage&) = default;
    DualStorage(DualStorage&&) = default;
    DualStorage& operator=(const DualStorage&) = default;
    DualStorage& operator=(DualStorage&&) = default;
    ~DualStorage() = default;

    DualStorage(Vector linear, Vector angular)
      : linear(std::move(linear)), angular(std::move(angular)) {}

    Vector linear;
    Vector angular;
};

template <typename Derived>
class Dual : protected DualStorage {
    using Base = DualStorage;

  public:
    Dual(Vector linear, Vector angular)
      : Base(std::move(linear), std::move(angular)) {}
    Dual(const Eigen::Matrix<double, 6, 1>& vec)
      : Base(Vector(vec.template head<3>()), Vector(vec.template tail<3>())) {}
    Dual(Base other) : Base(std::move(other)) {}

    inline const math::Vector& linear() const { return Base::linear; }
    inline const math::Vector& angular() const { return Base::angular; }
    inline const Base& dual() const { return static_cast<const Base&>(*this); }

    inline Derived& operator+=(const Derived& other) {
        Base::linear += other.linear();
        Base::angular += other.angular();
        return static_cast<Derived&>(*this);
    }
    inline Derived& operator-=(const Derived& other) {
        Base::linear -= other.linear();
        Base::angular -= other.angular();
        return static_cast<Derived&>(*this);
    }
    inline Derived& operator*=(double scalar) {
        Base::linear *= scalar;
        Base::angular *= scalar;
        return static_cast<Derived&>(*this);
    }
    inline Derived& operator/=(double scalar) {
        Base::linear /= scalar;
        Base::angular /= scalar;
        return static_cast<Derived&>(*this);
    }

    inline Derived operator+(const Derived& other) const {
        Derived out(static_cast<const Derived&>(*this));
        return out += other;
    }
    inline Derived operator-(const Derived& other) const {
        Derived out(static_cast<const Derived&>(*this));
        return out -= other;
    }
    inline Derived operator*(double scalar) const {
        Derived out(static_cast<const Derived&>(*this));
        return out *= scalar;
    }
    inline Derived operator/(double scalar) const {
        Derived out(static_cast<const Derived&>(*this));
        return out /= scalar;
    }

    inline Eigen::Matrix<double, 6, 1> mat() const {
        Eigen::Matrix<double, 6, 1> mat;
        mat << Base::linear.mat(), Base::angular.mat();
        return mat;
    }

    inline double norm() const {
        double linear_mag = Base::linear.mag();
        double angular_mag = Base::angular.mag();

        return std::sqrt(linear_mag * linear_mag + angular_mag * angular_mag);
    }
    inline void normalize() {
        double n = norm();
        if (n > 0) {
            *this /= n;
        }
    }
    inline Derived normalized() const {
        Derived out(static_cast<const Derived&>(*this));
        out.normalize();
        return out;
    }
};  // class Dual

}  // namespace achilles::math