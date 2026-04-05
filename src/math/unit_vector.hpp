#pragma once

#include "math/vector.hpp"

namespace achilles::math {

class UnitVector : public Vector {
public:
    UnitVector() = default;
    UnitVector(const UnitVector& other) = default;
    UnitVector(double x, double y, double z) : Vector(x, y, z) { normalize(); }
    UnitVector(const Vector& vec) : Vector(vec) { normalize(); }
    UnitVector(const Eigen::Vector3d& vec) : Vector(vec) { normalize(); }
}; // class UnitVector
} // namespace achilles::math