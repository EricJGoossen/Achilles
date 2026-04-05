#include "quaternion.hpp"

namespace achilles::math 
{

Quaternion& Quaternion::operator*=(const Quaternion& other) 
{
    data_ *= other.data_;
    return *this;
}

Quaternion Quaternion::inverse() const 
{
    return Quaternion(data_.inverse());
}

Vector Quaternion::rotate(const Vector& vec) const 
{
    Quaternion vq{ 0, vec.x(), vec.y(), vec.z() };
    Quaternion result = (*this) * vq * this->conjugate();
    return { result.x(), result.y(), result.z() };
}

void Quaternion::propagate(const Vector& angular_velocity, double dt)
{
    double omega_mag = angular_velocity.mag();
    double half_theta = omega_mag * dt * 0.5;

    Quaternion dq;
    if (omega_mag > epsilon) {
        double sin = std::sin(half_theta) / omega_mag;
        dq = Quaternion(
            std::cos(half_theta),
            angular_velocity.x() * sin,
            angular_velocity.y() * sin,
            angular_velocity.z() * sin
        );
    }

   this->operator*=(dq);
   this->normalize();
}

} // namespace achilles::math