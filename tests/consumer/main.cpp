// Loads nothing, steps nothing: links, and Render() before Init() refuses
// instead of opening a window -- with the viewer built in or not.
#include "interface/simulation.hpp"

bool RenderHeadless(achilles::interface::Simulation& sim);

int main() {
  achilles::interface::Simulation sim;
  // Before Init(), Render() must refuse.
  return RenderHeadless(sim) ? 1 : 0;
}
