#pragma once

#include <cstdint>
#include <random>

namespace achilles::domain::random {

// A seed drawn from the system's random source, for a run that wasn't
// given one.
inline std::uint64_t GenerateSeed() {
  std::random_device device;
  return (static_cast<std::uint64_t>(device()) << 32U) | device();
}

}  // namespace achilles::domain::random
