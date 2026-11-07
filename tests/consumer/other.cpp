// The second translation unit: including the facade here too is the test.
#include "interface/simulation.hpp"

bool RenderHeadless(achilles::interface::Simulation& sim) {
  return sim.Render();
}
