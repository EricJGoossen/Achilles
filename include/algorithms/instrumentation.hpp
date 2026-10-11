#pragma once

#include "algorithms/log/log_data.hpp"
#include "util/logger.hpp"

namespace achilles::algorithms {

// Everything that observes a run without affecting it, passed to every
// Step alongside SimConfig: the logger now; a profiler and the like later.
// A Step that doesn't observe anything ignores it.
//
// Owned and refreshed by whoever drives the steps (interface::Simulation);
// a Step only reads it. `logger` is a pointer so a const Instrumentation
// still hands out a writable logger.
template <util::LoggerLike LoggerT>
struct Instrumentation {
  using Logger = LoggerT;

  // nullptr: nothing is logged.
  LoggerT* logger = nullptr;
  // Which fields log::LogStep writes, changeable between steps.
  log::LogSelection log_fields;
  // Sim time of the state being stepped, written with each logged step.
  double time = 0.0;
};

using NoInstrumentation = Instrumentation<util::NullLogger>;
inline constexpr NoInstrumentation kNoInstrumentation{};

}  // namespace achilles::algorithms
