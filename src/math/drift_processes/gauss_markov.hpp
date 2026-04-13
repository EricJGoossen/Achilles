#pragma once

#include <cstdint>

#include "math/distributions/gaussian.hpp"
#include "math/drift_processes/i_drift.hpp"

namespace achilles::math::drift_processes {

class GaussMarkov : public IDrift {
  public:
    GaussMarkov(
        double initialization_mean,
        double initialization_stddev,
        double stddev,
        double correlation_time,
        std::uint32_t seed = std::random_device{}()
    )
      : unit_dist_(0.0, 1.0, seed),
        stddev_(stddev),
        correlation_time_(correlation_time),
        prev_sample_(
            initialization_mean + initialization_stddev * unit_dist_.sample()
        ) {}

    double sample(double dt) override {
        double alpha = std::exp(-dt / correlation_time_);

        double noise_stddev = stddev_ * std::sqrt(1.0 - alpha * alpha);
        double noise = noise_stddev * unit_dist_.sample();

        prev_sample_ = alpha * prev_sample_ + noise;
        return prev_sample_;
    }

  private:
    math::distributions::Gaussian unit_dist_;
    double stddev_;
    double correlation_time_;
    double prev_sample_;
};

}  // namespace achilles::math::drift_processes