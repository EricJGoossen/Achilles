#pragma once

#include <utility>

#include "math/vector.hpp"

namespace achilles::math {

class UnitVector : public Vector {
  public:
    UnitVector() = default;
    UnitVector(const UnitVector&) = default;
    UnitVector(UnitVector&&) = default;
    UnitVector& operator=(const UnitVector&) = default;
    UnitVector& operator=(UnitVector&&) = default;
    ~UnitVector() = default;

    UnitVector(double x, double y, double z) : Vector(x, y, z) { normalize(); }
    UnitVector(Vector vec) : Vector(std::move(vec)) { normalize(); }
    UnitVector(Eigen::Vector3d vec) : Vector(std::move(vec)) { normalize(); }
};  // class UnitVector
}  // namespace achilles::math