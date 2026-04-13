#pragma once

namespace achilles::math::distributions {

class IDistribution {
  public:
    IDistribution() = default;
    IDistribution(const IDistribution&) = default;
    IDistribution(IDistribution&&) = default;
    IDistribution& operator=(const IDistribution&) = default;
    IDistribution& operator=(IDistribution&&) = default;
    virtual ~IDistribution() = default;

    virtual double mean() const = 0;
    virtual double stddev() const = 0;

    virtual double sample() const = 0;
};

}  // namespace achilles::math::distributions