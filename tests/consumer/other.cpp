// The second translation unit: including the facade here too is the test.
#include "interface/simulation.hpp"

bool StepUninitialized(achilles::interface::Simulation& sim) {
  return sim.Step(0.01F);
}
