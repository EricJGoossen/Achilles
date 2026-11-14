#include "display/renderer.hpp"

#include <cstddef>
#include <string_view>

#include "display/display_set.hpp"

namespace achilles::display {

Renderer::Renderer(std::string_view title) : screen_(title) {}

// The windows go before the screen, and free their textures in its context.
Renderer::~Renderer() { screen_.MakeCurrent(); }

void Renderer::Detach(DisplayId id) {
  screen_.MakeCurrent();
  displays_.Remove(id);
}

bool Renderer::Draw() {
  screen_.BeginFrame();
  const std::size_t up = displays_.Draw();
  screen_.EndFrame();
  return up > 0;
}

}  // namespace achilles::display
