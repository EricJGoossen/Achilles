#include "pose.hpp"

namespace achilles::spatial {

using Vector = math::Vector;
using Quaternion = math::Quaternion;

Pose& Pose::operator*=(const Pose& other) {
    position_ += orientation_.rotate(other.position_);
    orientation_ *= other.orientation_;
    return *this;
}

Pose Pose::operator*(const Pose& other) const { return Pose(*this) *= other; }

Vector Pose::operator*(const Vector& point) const {
    return orientation_.rotate(point) + position_;
}

double Pose::angle() const { return 2.0 * std::acos(orientation_.w()); }

Pose Pose::inverse() const {
    Quaternion R_inv = orientation_.inverse();
    return {R_inv.rotate(position_) * -1, R_inv};
}

void Pose::propagate(const Twist& tx, double dt) {
    position_ += orientation_.rotate(tx.linear()) * dt;
    orientation_.propagate(tx.angular(), dt);
}

}  // namespace achilles::spatial