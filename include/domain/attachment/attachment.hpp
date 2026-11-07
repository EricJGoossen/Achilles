#pragma once

#include <cstdint>

#include "domain/sim_clock.hpp"
#include "util/tmp.hpp"

// What sensors and actuators have in common: each is an attachment, mounted on
// a joint, declared in the .arow, and run by the sim at its own rate with its
// own random stream.
namespace achilles::domain::attachment {

// The sim quantities a kind reads at its mount joint (SimInputs) or, for a
// scanning kind, at every body (SceneInputs): see sim_quantities.hpp.
template <typename... Inputs>
struct SimInputs {};

template <typename... Inputs>
struct SceneInputs {
  using List = util::TypeList<Inputs...>;
};

// The State of a kind that keeps none.
struct NoState {};

// The Command of a kind nobody commands: every sensor.
struct NoCommand {};

// Per-instance settings every attachment has, whatever it models.
struct AttachmentOptions {
  // Seconds between updates; 0 updates every tick. An attachment whose period
  // hasn't elapsed holds what it last did: a sensor reports its last reading
  // again, an actuator keeps applying its last effort.
  float period = 0.0F;
};

// What an attachment last did: its output, when (sim time, seconds since Init),
// and how many updates it has made -- 0 until its first.
template <typename OutputT>
struct Reading {
  OutputT value{};
  double sample_time = 0.0;
  std::uint32_t sequence = 0;
};

// Everything about "now" an attachment receives. stream and tick key a
// counter-based random generator (domain::random::CounterRng): a draw depends
// only on (stream, tick, draw index), never on what ran before it, so results
// are reproducible whichever thread runs which instance. stream identifies the
// instance: its index mixed with the sim's seed (see Attachment::Stream).
struct AttachmentTick {
  float dt = 0.0F;
  double time = 0.0;
  std::uint64_t tick = 0;
  std::uint64_t stream = 0;
};

// The base of Sensor and Actuator.
class Attachment {
 public:
  Attachment() = default;
  explicit Attachment(AttachmentOptions options) : options_(options) {}

  const AttachmentOptions& Options() const { return options_; }

  // This instance's random stream (AttachmentTick::stream), set by whatever
  // places it in the sim, from the sim's seed and the instance's index, so no
  // two instances draw the same noise.
  std::uint64_t Stream() const { return stream_; }
  void SetStream(std::uint64_t stream) { stream_ = stream; }

  // Whether an update is due at sim time `time`, given the last one published:
  // always for the first, then once `period` has elapsed -- within half a tick,
  // so a period that's a whole number of ticks isn't missed to rounding.
  template <typename OutputT>
  bool Due(double time, float dt, const Reading<OutputT>& last) const {
    return last.sequence == 0 ||
           time - last.sample_time >= options_.period - (0.5F * dt);
  }

  // What it's handed for a tick at `clock`.
  AttachmentTick TickAt(const SimClock& clock, float dt) const {
    return {dt, clock.time, clock.tick, stream_};
  }

  // One tick of an instance: when an update is due, `update(tick, &value)`
  // makes a new output, stamped into `out`; otherwise `last` carries forward.
  template <typename OutputT, typename UpdateFn>
  void UpdateIfDue(
      const SimClock& clock,
      float dt,
      const Reading<OutputT>& last,
      Reading<OutputT>* out,
      const UpdateFn& update
  ) const {
    if (!Due(clock.time, dt, last)) {
      *out = last;
      return;
    }
    Reading<OutputT> reading;
    update(TickAt(clock, dt), &reading.value);
    reading.sample_time = clock.time;
    reading.sequence = last.sequence + 1;
    *out = reading;
  }

 private:
  AttachmentOptions options_;
  std::uint64_t stream_ = 0;
};

}  // namespace achilles::domain::attachment
