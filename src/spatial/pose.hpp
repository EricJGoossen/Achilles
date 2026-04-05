#pragma once

#include <Eigen/Geometry>

#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include "twist.hpp"

namespace achilles::spatial {

class Pose {
public:
    Pose(const math::Vector& position, 
        const math::Quaternion& orientation) : 
        position_(position), 
        orientation_(orientation) {}
    
    Pose(const Pose& other) = default;
    Pose(const Eigen::Matrix3d& rot, 
        const Eigen::Vector3d& pos) : 
        position_(pos), 
        orientation_(rot) {}


    static Pose identity() { return Pose(math::Vector(0, 0, 0), math::Quaternion(1, 0, 0, 0)); }

    inline const math::Vector& position() const { return position_; }
    inline const math::Quaternion& orientation() const { return orientation_; }

    Pose& operator*=(const Pose& other);
    Pose operator*(const Pose& other) const;
    math::Vector operator*(const math::Vector& point) const;

    double angle() const { return 2.0 * std::acos(orientation_.w()); }

    Pose inverse() const;
    void propagate(const Twist& tx, double dt);

private:
    math::Vector position_;
    math::Quaternion orientation_;
}; // class Pose

} // namespace achilles::spatial