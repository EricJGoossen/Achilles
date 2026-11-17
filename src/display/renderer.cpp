#include "display/renderer.hpp"

#include <cstddef>
#include <string_view>

#include "display/window_set.hpp"

namespace achilles::display {

Renderer::Renderer(std::string_view title) : screen_(title) {}

// The windows go before the screen, and free their textures in its context.
Renderer::~Renderer() { screen_.MakeCurrent(); }

void Renderer::Detach(WindowId id) {
  screen_.MakeCurrent();
  windows_.Remove(id);
}

bool Renderer::Draw() {
  screen_.BeginFrame();
  const std::size_t up = windows_.Draw();
  screen_.EndFrame();
  return up > 0;
}

}  // namespace achilles::display
