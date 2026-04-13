#pragma once

namespace achilles::math::filters {

class IFilter {
  public:
    IFilter() = default;
    IFilter(const IFilter&) = default;
    IFilter(IFilter&&) = default;
    IFilter& operator=(const IFilter&) = default;
    IFilter& operator=(IFilter&&) = default;
    virtual ~IFilter() = default;

    virtual double compute(double measurement, double dt) = 0;
};
}  // namespace achilles::math::filters