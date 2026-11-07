#pragma once

#include <cstddef>
#include <string_view>

#include "domain/attachment/attachment.hpp"
#include "domain/attachment/attachment_config.hpp"
#include "domain/attachment/sim_quantities.hpp"
#include "domain/sim_clock.hpp"
#include "util/tmp.hpp"

namespace achilles::domain::sensing {

// What a kind is written in.
using attachment::AttachmentConfig;
using attachment::AttachmentOptions;
using attachment::AttachmentTick;
using attachment::NoState;
using attachment::SceneInputs;
using attachment::SimInputs;
namespace input = attachment::input;

// The base every sensor kind derives from, declaring what it needs:
//   - Inputs: SimInputs<...> -- quantities of the sim (input::JointVelocity
//     and so on), read in place at the mount joint;
//   - StateT: persistent per-instance state (a delay line, a drifting bias,
//     last tick's velocity, ...), trivially copyable; NoState if none;
//   - OutputT: what one sample is.
// The derived class itself is the instance's configuration: it's stored,
// whole, in the sim's own memory, so it must be trivially copyable and
// default-constructible. It provides
//
//   static constexpr std::string_view kListName = "...";  // its .arow name
//   static Self FromConfig(AttachmentConfig& config);     // its settings
//   void Read(const AttachmentTick&, const Input&..., StateT*, OutputT*) const;
//   void Initialize(const AttachmentTick&, StateT*) const;  // optional
//   void InitializeOutput(const AttachmentTick&, const Input&...,
//                         OutputT*) const;                  // optional
//
// A scanning kind -- one that looks at every body, not just its own -- also
// declares
//
//   using Scene = SceneInputs<...>;
//   void Observe(const AttachmentTick&, const Input&..., const SceneInput&...,
//                StateT*) const;
//
// Observe runs, on a tick it samples, once per body (each body's scene
// values), accumulating into its state; Read then turns that into the reading.
//
// A sensor never names the algorithm a quantity comes from: the engine side
// binds each one (algorithms/attachment_kinds/sim_input.hpp) and hands over its
// value. Read runs only when a sample is due (see Attachment::Due). Initialize
// seeds the state from the configuration, at Init and whenever the
// configuration is replaced; without it the state starts value-initialized.
// InitializeOutput fills the reading seen before the first sample, once, at
// Init; without it that reading is value-initialized. Each Input is one of
// Inputs' Value. Read's parameters are shaped like an Op's: inputs by const
// reference, then everything written by pointer. Storage and
// the passes that run it are generated from these declarations
// (algorithms/attachment). No virtuals: a sensor is only ever called through
// its own concrete type.
template <typename Derived, typename Inputs, typename StateT, typename OutputT>
class Sensor;

template <
    typename Derived,
    typename... Inputs,
    typename StateT,
    typename OutputT>
class Sensor<Derived, attachment::SimInputs<Inputs...>, StateT, OutputT>
    : public attachment::Attachment {
 public:
  using DerivedType = Derived;
  using SensorBase = Sensor;
  using InputList = util::TypeList<Inputs...>;
  static constexpr std::size_t kInputCount = sizeof...(Inputs);
  using State = StateT;
  using Command = attachment::NoCommand;
  using Output = OutputT;
  // The key an .arow declares one under.
  static constexpr std::string_view kKey = "sensor";

  using Attachment::Attachment;

  // One tick: reads when a sample is due, else carries `last` forward.
  void Update(
      const SimClock& clock,
      float dt,
      const typename Inputs::Value&... inputs,
      const attachment::Reading<OutputT>& last,
      StateT* state,
      attachment::Reading<OutputT>* out
  ) const {
    UpdateIfDue(clock, dt, last, out, [&](const auto& tick, OutputT* value) {
      static_cast<const Derived&>(*this).Read(tick, inputs..., state, value);
    });
  }

  // One body of a scanning kind's tick: `values` are its own inputs, then that
  // body's scene values, handed to Observe when a sample is due. Due reads the
  // same clock and last reading here as in Update, so the two agree.
  template <typename... Values>
  void Scan(
      const SimClock& clock,
      float dt,
      const attachment::Reading<OutputT>& last,
      StateT* state,
      const Values&... values
  ) const {
    if (Due(clock.time, dt, last)) {
      static_cast<const Derived&>(*this).Observe(
          TickAt(clock, dt), values..., state
      );
    }
  }
};

}  // namespace achilles::domain::sensing
