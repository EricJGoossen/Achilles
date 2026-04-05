#pragma once

#include <concepts>
#include <Eigen/Dense>

#include "vector.hpp"

namespace achilles::math
{

struct DualStorage {
    DualStorage(const Vector& linear, const Vector& angular) : linear(linear), angular(angular) {}
    DualStorage(const DualStorage& other) : linear(other.linear), angular(other.angular) {}

    Vector linear;
    Vector angular;
};

template<typename T>
class Dual : protected DualStorage
{
    using Base = DualStorage;

public:
    Dual(const Vector& linear, const Vector& angular) : Base(linear, angular) {}
    Dual(Eigen::Matrix<double, 6, 1> vec) : Base(Vector(vec.template head<3>()), Vector(vec.template tail<3>())) {}
    Dual(const Base& other) : Base(other) {}
    Dual(const Dual& other) = default;
    Dual() = default;

    inline const math::Vector& linear() const { return Base::linear; }
    inline const math::Vector& angular() const { return Base::angular; }
    inline const Base& dual() const { return static_cast<const Base&>(*this); }
    
    inline const Eigen::Matrix<double, 6, 1> mat() const {
        Eigen::Matrix<double, 6, 1> mat;
        mat << Base::linear.mat(), Base::angular.mat();
        return mat;
    }

    inline T& operator+=(const T& other) {
        Base::linear += other.linear();
        Base::angular += other.angular();
        return static_cast<T&>(*this);
    }
    inline T& operator-=(const T& other) {
        Base::linear -= other.linear();
        Base::angular -= other.angular();
        return static_cast<T&>(*this);
    }
    inline T& operator*=(double scalar) {
        Base::linear *= scalar;
        Base::angular *= scalar;
        return static_cast<T&>(*this);
    }
    inline T& operator/=(double scalar) {
        Base::linear /= scalar;
        Base::angular /= scalar;
        return static_cast<T&>(*this);
    }

    inline T operator+(const T& other) const { T out(static_cast<const T&>(*this)); return out += other; }
    inline T operator-(const T& other) const { T out(static_cast<const T&>(*this)); return out -= other; }
    inline T operator*(double scalar) const { T out(static_cast<const T&>(*this)); return out *= scalar; }
    inline T operator/(double scalar) const { T out(static_cast<const T&>(*this)); return out /= scalar; }

    inline double norm() const { return std::sqrt(Base::linear.mag() * Base::linear.mag() + Base::angular.mag() * Base::angular.mag()); }
    inline void normalize() { 
        double n = norm();
        if (n > 0) {
            *this /= n;
        }
    }
    inline T normalized() const { 
        T out(static_cast<const T&>(*this)); 
        out.normalize(); 
        return out; 
    }
}; // class Dual

} // namespace achilles::math