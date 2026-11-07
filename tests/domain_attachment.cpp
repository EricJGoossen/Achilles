// Attachment: what every sensor and actuator shares -- when an update is due,
// and what one stamps.
#include <gtest/gtest.h>

#include "domain/attachment/attachment.hpp"
#include "domain/sim_clock.hpp"

using achilles::domain::SimClock;
using achilles::domain::attachment::Attachment;
using achilles::domain::attachment::AttachmentOptions;
using achilles::domain::attachment::AttachmentTick;
using achilles::domain::attachment::Reading;

namespace {

constexpr float kDt = 0.01F;

Attachment WithPeriod(float period) {
  return Attachment(AttachmentOptions{period});
}

Reading<int> SampledAt(double time) {
  return {.value = 7, .sample_time = time, .sequence = 3};
}

}  // namespace

TEST(AttachmentDue, TheFirstUpdateIsAlwaysDue) {
  EXPECT_TRUE(WithPeriod(100.0F).Due(0.0, kDt, Reading<int>{}));
}

TEST(AttachmentDue, NoPeriodIsDueEveryTick) {
  EXPECT_TRUE(WithPeriod(0.0F).Due(1.0, kDt, SampledAt(1.0)));
}

TEST(AttachmentDue, APeriodWaitsUntilItHasElapsed) {
  const Attachment attachment = WithPeriod(0.1F);
  EXPECT_FALSE(attachment.Due(1.05, kDt, SampledAt(1.0)));
  EXPECT_TRUE(attachment.Due(1.1, kDt, SampledAt(1.0)));
}

// Within half a tick, so a whole number of ticks isn't missed to rounding.
TEST(AttachmentDue, APeriodIsDueWithinHalfATick) {
  const Attachment attachment = WithPeriod(0.1F);
  EXPECT_TRUE(attachment.Due(1.096, kDt, SampledAt(1.0)));
  EXPECT_FALSE(attachment.Due(1.09, kDt, SampledAt(1.0)));
}

TEST(AttachmentUpdate, ADueUpdateIsStampedWithTheClock) {
  Attachment attachment;
  attachment.SetStream(9);
  const SimClock clock{.time = 2.0, .tick = 5};
  Reading<int> out;
  attachment.UpdateIfDue(
      clock,
      kDt,
      SampledAt(1.0),
      &out,
      [](const AttachmentTick& tick, int* value) {
        EXPECT_FLOAT_EQ(tick.dt, kDt);
        EXPECT_EQ(tick.time, 2.0);
        EXPECT_EQ(tick.tick, 5U);
        EXPECT_EQ(tick.stream, 9U);
        *value = 42;
      }
  );
  EXPECT_EQ(out.value, 42);
  EXPECT_EQ(out.sample_time, 2.0);
  EXPECT_EQ(out.sequence, 4U);
}

TEST(AttachmentUpdate, AnUpdateThatIsntDueCarriesTheLastForward) {
  const Attachment attachment = WithPeriod(10.0F);
  const SimClock clock{.time = 2.0, .tick = 5};
  Reading<int> out;
  bool updated = false;
  attachment.UpdateIfDue(
      clock,
      kDt,
      SampledAt(1.0),
      &out,
      [&](const AttachmentTick& /*tick*/, int* /*value*/) { updated = true; }
  );
  EXPECT_FALSE(updated);
  EXPECT_EQ(out.value, 7);
  EXPECT_EQ(out.sample_time, 1.0);
  EXPECT_EQ(out.sequence, 3U);
}
