#pragma once

namespace achilles::control::sensors::distortions {

class IDistortion {
  public:
    IDistortion() = default;
    IDistortion(const IDistortion&) = delete;
    IDistortion(IDistortion&&) = default;
    IDistortion& operator=(const IDistortion&) = delete;
    IDistortion& operator=(IDistortion&&) = default;
    virtual ~IDistortion() = default;

    virtual double AddDistortion(double measurement, double dt) = 0;
};
}  // namespace achilles::control::sensors::distortions