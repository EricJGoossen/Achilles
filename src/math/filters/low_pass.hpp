#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>

#include "math/filters/i_filter.hpp"

namespace achilles::math::filters {

class LowPass : public IFilter {
  public:
    LowPass(double cutoff) : cutoff_(cutoff) {}

    double compute(double measurement, double dt) override {
        if (first_measurement_) {
            prev_output_ = measurement;
            first_measurement_ = false;
            return measurement;
        }

        double alpha = std::exp(-kTwoPi * cutoff_ * dt);
        prev_output_ = (1 - alpha) * measurement + alpha * prev_output_;

        return prev_output_;
    }

  private:
    static constexpr double kTwoPi = std::numbers::pi * 2;

    double cutoff_;
    double prev_output_ = 0;
    bool first_measurement_ = true;
};
}  // namespace achilles::math::filters