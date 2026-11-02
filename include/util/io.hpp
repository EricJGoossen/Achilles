#pragma once

#include <iostream>

namespace achilles::util {

// inline: defined in a header that interface/simulation.hpp includes, so a
// program of more than one translation unit would otherwise define these
// once per unit and fail to link.
inline bool Warning(const char* message) {
  std::cerr << "[WARNING] " << message << "\n";
  return false;
}

inline bool Warning(const char* message, const char* error) {
  std::cerr << "[WARNING] " << message << ": " << error << "\n";
  return false;
}
}  // namespace achilles::util