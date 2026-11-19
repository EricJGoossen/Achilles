// The real-time loop every example program runs on
// (examples/shared/real_time.hpp), without a display: fixed-dt ticks paced
// by the wall clock, stopping when it's done or a tick fails.
#include <gtest/gtest.h>

#include <chrono>

#include "../examples/shared/real_time.hpp"
#include "display/renderer.hpp"

namespace {

using achilles::examples::RunInRealTime;

// Never called without a display.
const auto kNoAttach = [](achilles::display::Renderer& /*renderer*/) {
  ADD_FAILURE() << "attach was called without a display";
};

}  // namespace

TEST(RunInRealTime, TicksUntilItsDone) {
  int ticks = 0;
  RunInRealTime<false>(
      "test",
      0.001F,
      kNoAttach,
      [&] {
        ++ticks;
        return true;
      },
      [&] { return ticks >= 5; }
  );
  EXPECT_EQ(ticks, 5);
}

TEST(RunInRealTime, StopsAtTheFirstTickThatFails) {
  int ticks = 0;
  RunInRealTime<false>(
      "test",
      0.001F,
      kNoAttach,
      [&] { return ++ticks < 3; },
      [] { return false; }
  );
  EXPECT_EQ(ticks, 3);
}

TEST(RunInRealTime, AlreadyDoneRunsNothing) {
  int ticks = 0;
  RunInRealTime<false>(
      "test",
      0.001F,
      kNoAttach,
      [&] {
        ++ticks;
        return true;
      },
      [] { return true; }
  );
  EXPECT_EQ(ticks, 0);
}

// Paced by the wall clock: N ticks of dt take about N * dt, never much less.
TEST(RunInRealTime, KeepsToTheWallClock) {
  constexpr float kDt = 0.005F;
  constexpr int kTicks = 20;
  int ticks = 0;
  const auto start = std::chrono::steady_clock::now();
  RunInRealTime<false>(
      "test",
      kDt,
      kNoAttach,
      [&] {
        ++ticks;
        return true;
      },
      [&] { return ticks >= kTicks; }
  );
  const float elapsed =
      std::chrono::duration<float>(std::chrono::steady_clock::now() - start)
          .count();
  EXPECT_EQ(ticks, kTicks);
  // The last tick runs once (kTicks - 1) * dt have passed.
  EXPECT_GE(elapsed, (kTicks - 1) * kDt * 0.95F);
}
