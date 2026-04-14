#pragma once

#include <random>

#include "i_distribution.hpp"

namespace achilles::math::distributions {

class Gaussian : public IDistribution {
  public:
    Gaussian(
        double mean, double stddev, std::uint32_t seed = std::random_device{}()
    )
      : dist_(mean, stddev), gen_(seed) {}

    Gaussian(const Gaussian& other)
      : dist_(other.dist_), gen_(std::random_device{}()) {}

    Gaussian& operator=(const Gaussian& other) {
        dist_ = other.dist_;
        gen_ = std::mt19937(std::random_device{}());
        return *this;
    }

    Gaussian(Gaussian&&) = default;
    Gaussian& operator=(Gaussian&&) = default;
    ~Gaussian() override = default;

    double mean() const override { return dist_.mean(); }
    double stddev() const override { return dist_.stddev(); }

    double sample() const override { return dist_(gen_); }

    static double unitSample() {
        static std::mt19937 gen(std::random_device{}());
        return std::normal_distribution<double>(0, 1)(gen);
    }

  private:
    mutable std::normal_distribution<double> dist_;
    std::mt19937 gen_;
};
}  // namespace achilles::math::distributions