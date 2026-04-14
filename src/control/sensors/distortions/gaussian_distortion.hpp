#pragma once

#include "control/sensors/distortions/i_distortion.hpp"
#include "math/distributions/gaussian.hpp"
#include "math/distributions/i_distribution.hpp"
#include "math/drift_processes/gauss_markov.hpp"
#include "math/drift_processes/i_drift.hpp"
#include "math/filters/i_filter.hpp"
#include "math/filters/low_pass.hpp"

namespace achilles::control::sensors::distortions {

class GaussianDistortion : public IDistortion {
  public:
    GaussianDistortion(
        double noise_mean,
        double noise_stddev,
        double drift_init_mean,
        double drift_init_stddev,
        double drift_stddev,
        double drift_correlation_time,
        double filter_cutoff,
        double scale_factor,
        double scale_factor_mean,
        double scale_factor_stddev
    )
      : noise_(noise_mean, noise_stddev),
        drift_(
            drift_init_mean,
            drift_init_stddev,
            drift_stddev,
            drift_correlation_time
        ),
        filter_(filter_cutoff),
        scale_factor_(scale_factor),
        scale_factor_error_(
            math::distributions::Gaussian::unitSample() * scale_factor_stddev +
            scale_factor_mean
        ) {}

    double AddDistortion(double measurement, double dt) override {
        double filtered_measurement = filter_.compute(measurement, dt);
        return (scale_factor_ + scale_factor_error_) * filtered_measurement +
               drift_.sample(dt) + noise_.sample();
    }

  private:
    math::distributions::Gaussian noise_;
    math::drift_processes::GaussMarkov drift_;
    math::filters::LowPass filter_;
    double scale_factor_;
    double scale_factor_error_;
};
}  // namespace achilles::control::sensors::distortions