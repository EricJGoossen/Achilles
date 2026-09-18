#pragma once

#include <iostream>

namespace achilles::util {

bool Warning(const char* message) {
  std::cerr << "[WARNING] " << message << "\n";
  return false;
}

bool Warning(const char* message, const char* error) {
  std::cerr << "[WARNING] " << message << ": " << error << "\n";
  return false;
}
}  // namespace achilles::util