#pragma once

// The real-time loop the example programs share, with their displays.
#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>

#include "achilles/build_config.hpp"
#include "display/renderer.hpp"
#include "interface/simulation.hpp"

namespace achilles::examples {

// Runs an example in real time: each pass, as many fixed-dt ticks as the
// wall clock covers -- clamped, so a hitch doesn't turn into a burst -- each
// a tick() that returns false to stop, until done(). With the display
// (WithDisplay, by default whether this build has one), each pass is also
// one frame of the displays attach(renderer) puts up, titled `title`, and
// closing them all stops the run; without one, or with no display server,
// it runs window-less. A template parameter rather than an if on
// kWithDisplay alone, so a build without the display never names it.
template <
    bool WithDisplay = kWithDisplay,
    typename AttachFn,
    typename TickFn,
    typename DoneFn>
void RunInRealTime(
    const interface::Simulation& sim,
    std::string_view title,
    float dt,
    const AttachFn& attach,
    const TickFn& tick,
    const DoneFn& done
) {
  using Renderer = display::Renderer;
  [[maybe_unused]] std::
      conditional_t<WithDisplay, std::optional<Renderer>, bool> renderer{};
  if constexpr (WithDisplay) {
    try {
      renderer.emplace(title);
      attach(*renderer);
    } catch (const std::exception& e) {
      std::cerr << title << ": no display (" << e.what()
                << ") -- running without one.\n";
      renderer.reset();
    }
  }

  constexpr float kMaxFrameTime = 0.25F;
  float accumulator = 0.0F;
  auto last_time = std::chrono::steady_clock::now();
  bool running = true;
  while (running && !done()) {
    const auto now = std::chrono::steady_clock::now();
    accumulator += std::min(
        std::chrono::duration<float>(now - last_time).count(), kMaxFrameTime
    );
    last_time = now;
    while (running && accumulator >= dt) {
      if constexpr (WithDisplay) {
        if (renderer.has_value()) {
          renderer->ApplyEfforts();
        }
      }
      running = tick();
      accumulator -= dt;
    }
    if constexpr (WithDisplay) {
      if (running && renderer.has_value()) {
        running = renderer->Draw();
      }
    }
  }
}

}  // namespace achilles::examples
