#pragma once

#include "control/sensors/distortions/i_distortion.hpp"

namespace achilles::control::sensors::distortions {

class NullDistortion : public IDistortion {
  public:
    double AddDistortion(double measurement, double dt) override {
        return measurement;
    }
};
}  // namespace achilles::control::sensors::distortions