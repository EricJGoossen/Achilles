#pragma once

#include <cstdint>

namespace achilles::domain {

// The sim's one clock: how much time the state has reached, and how many ticks
// it took. Lives in the sim's own memory, one for the whole sim, and is
// advanced at the end of every tick -- so during a tick it reads that tick's
// starting state's time, and between ticks the time the state has reached.
// Kept in double so a long run at a fine dt doesn't lose ticks to rounding (the
// physics itself runs in float).
struct SimClock {
  double time = 0.0;
  std::uint64_t tick = 0;
};

// One tick of `dt` has passed.
inline void Advance(SimClock* clock, float dt) {
  clock->time += static_cast<double>(dt);
  ++clock->tick;
}

}  // namespace achilles::domain
