#pragma once

#include "math/dual.hpp"
#include "math/vector.hpp"

namespace achilles::spatial {

class Wrench : public math::Dual<Wrench> {
    using Base = math::Dual<Wrench>;
    using Storage = math::DualStorage;

  public:
    Wrench(const Wrench&) = default;
    Wrench(Wrench&&) = default;
    Wrench& operator=(const Wrench&) = default;
    Wrench& operator=(Wrench&&) = default;
    ~Wrench() = default;

    Wrench(math::Vector linear, math::Vector angular)
      : Base(std::move(linear), std::move(angular)) {}
    Wrench(const Eigen::Matrix<double, 6, 1>& vec) : Base(vec) {}
    Wrench(const Storage& storage) : Base(storage) {}
};  // class Wrench
}  // namespace achilles::spatial
