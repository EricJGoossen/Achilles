#pragma once

#include <Eigen/Geometry>

#include "math/quaternion.hpp"
#include "math/vector.hpp"
#include "twist.hpp"

namespace achilles::spatial {

class Pose {
  public:
    Pose(const Pose&) = default;
    Pose(Pose&&) = default;
    Pose& operator=(const Pose&) = default;
    Pose& operator=(Pose&&) = default;
    ~Pose() = default;

    Pose(math::Vector position, math::Quaternion orientation)
      : position_(std::move(position)), orientation_(std::move(orientation)) {}
    Pose(const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos)
      : position_(pos), orientation_(rot) {}

    static Pose identity() {
        return {math::Vector::zero(), math::Quaternion::identity()};
    }

    inline const math::Vector& position() const { return position_; }
    inline const math::Quaternion& orientation() const { return orientation_; }

    Pose& operator*=(const Pose& other);
    Pose operator*(const Pose& other) const;
    math::Vector operator*(const math::Vector& point) const;

    double angle() const;

    Pose inverse() const;
    void propagate(const Twist& tx, double dt);

  private:
    math::Vector position_;
    math::Quaternion orientation_;
};

}  // namespace achilles::spatial