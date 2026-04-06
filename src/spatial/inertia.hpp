#pragma once

#include <Eigen/Dense>
#include <utility>

namespace achilles::spatial {

class Inertia {
    using Inertia3d = Eigen::Matrix<double, 6, 6>;

  public:
    Inertia(const Inertia&) = default;
    Inertia(Inertia&&) = default;
    Inertia& operator=(const Inertia&) = default;
    Inertia& operator=(Inertia&&) = default;
    ~Inertia() = default;

    Inertia(Inertia3d inertia) : data_(std::move(inertia)) {}

    inline Inertia3d mat() const { return data_; }

    static Inertia identity() { return {Inertia3d::Identity()}; }

    inline Inertia operator+=(const Inertia& other) {
        data_ += other.data_;
        return *this;
    }
    inline Inertia operator-=(const Inertia& other) {
        data_ -= other.data_;
        return *this;
    }

    inline Inertia operator+(const Inertia& other) const {
        return Inertia(*this) += other;
    }
    inline Inertia operator-(const Inertia& other) const {
        return Inertia(*this) -= other;
    }

  private:
    Inertia3d data_;
};  // class Inertia

}  // namespace achilles::spatial