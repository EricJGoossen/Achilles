// Loads nothing, steps nothing: links, and a Step() before Init() refuses
// instead of dereferencing an empty state.
#include "interface/simulation.hpp"

bool StepUninitialized(achilles::interface::Simulation& sim);

int main() {
  achilles::interface::Simulation sim;
  // Before Init(), Step() must refuse.
  return StepUninitialized(sim) ? 1 : 0;
}
