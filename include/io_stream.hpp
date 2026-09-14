#pragma once

#include <ostream>

#include "domain/math/matrix.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/math/vector6.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles {
namespace math = achilles::domain::math;
namespace spatial = achilles::domain::spatial;

// Math IO stream
template <size_t M, size_t N>
std::ostream& operator<<(std::ostream& os, const math::Matrix<float, M, N>& m) {
  os << "Matrix" << M << "x" << N << "(";
  for (std::size_t i = 0; i < M; ++i) {
    os << "(";
    for (std::size_t j = 0; j < N; ++j) {
      os << m(i, j);
      if (j < N - 1) os << ", ";
    }
    os << ")";
    if (i < M - 1) os << ", ";
  }
  os << ")";
  return os;
}
std::ostream& operator<<(std::ostream& os, const math::Quaternion<float>& q) {
  os << "Quaternion(" << q.W() << ", " << q.X() << ", " << q.Y() << ", "
     << q.Z() << ")";
  return os;
}
std::ostream& operator<<(std::ostream& os, const math::Vector3<float>& v) {
  os << "Vector3(" << v.X() << ", " << v.Y() << ", " << v.Z() << ")";
  return os;
}
std::ostream& operator<<(std::ostream& os, const math::Vector6<float>& v) {
  os << "Vector6(" << v.A() << ", " << v.B() << ", " << v.C() << ", " << v.D()
     << ", " << v.E() << ", " << v.F() << ")";
  return os;
}

// Spatial IO stream
template <template <typename> class Derived>
std::ostream& operator<<(
    std::ostream& os, const spatial::Dual<Derived, float>& d
) {
  os << "Dual(" << d.Linear() << ", " << d.Angular() << ")";
  return os;
}
std::ostream& operator<<(std::ostream& os, const spatial::Inertia<float>& i) {
  os << "Inertia(" << i.AsMatrix() << ")";
  return os;
}

}  // namespace achilles