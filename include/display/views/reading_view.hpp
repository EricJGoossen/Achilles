#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "display/contract/display.hpp"
#include "display/contract/frame.hpp"
#include "domain/actuation/dc_motor.hpp"
#include "domain/actuation/truth_actuator.hpp"
#include "domain/attachment/attachment_kind.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sensing/encoder.hpp"
#include "domain/sensing/limit_switch.hpp"
#include "domain/spatial/transform.hpp"
#include "interface/attachment_handle.hpp"

namespace achilles::display {

// The channels a ReadingView can plot, one enum per type of value an
// attachment reads, in the order the reading lists them: what its `channels`
// picks from. A view only takes its own type's, so a pose's channel can't be
// asked of an encoder.

// A vector: a gyroscope's rates (rad/s), an accelerometer's specific force
// (m/s^2), a magnetometer's field.
enum class VectorChannel : std::uint8_t { kX, kY, kZ };
// An encoder: its position (rad, or m for a linear one), then its rate.
enum class EncoderChannel : std::uint8_t { kPosition, kVelocity };
// A pose (a joint's position, a body's world transform): where it is (m),
// then how it's turned -- the angle (rad) about each axis.
enum class PoseChannel : std::uint8_t { kX, kY, kZ, kTurnX, kTurnY, kTurnZ };
// A spatial velocity or acceleration: its angular part, then its linear.
enum class MotionChannel : std::uint8_t {
  kAngularX,
  kAngularY,
  kAngularZ,
  kLinearX,
  kLinearY,
  kLinearZ,
};
// What a truth actuator's joint got (N*m, or N).
enum class EffortChannel : std::uint8_t { kEffort };
// What a motor's joint got (N*m), then the current behind it (A).
enum class MotorChannel : std::uint8_t { kEffort, kCurrent };
// A limit switch: pressed (0 or 1), then the position it thresholds.
enum class SwitchChannel : std::uint8_t { kPressed, kPosition };

// The channels of a reading of type Value: its Channel enum, every one of
// them in order (kAll), and Emit(value, emit), which hands each to
// `emit(channel, label, value)`. Every kind reading a type shows it the same
// way. Nothing for a type with none (a camera's image, a lidar's scan).
template <typename Value>
struct ReadingChannels {};

template <>
struct ReadingChannels<domain::math::Vector3<float>> {
  using Channel = VectorChannel;
  static constexpr std::array kAll = {Channel::kX, Channel::kY, Channel::kZ};
  static void Emit(
      const domain::math::Vector3<float>& value, const auto& emit
  ) {
    emit(Channel::kX, "x", value.X());
    emit(Channel::kY, "y", value.Y());
    emit(Channel::kZ, "z", value.Z());
  }
};

template <>
struct ReadingChannels<domain::sensing::EncoderReading> {
  using Channel = EncoderChannel;
  static constexpr std::array kAll = {Channel::kPosition, Channel::kVelocity};
  static void Emit(
      const domain::sensing::EncoderReading& value, const auto& emit
  ) {
    emit(Channel::kPosition, "position", value.position);
    emit(Channel::kVelocity, "velocity", value.velocity);
  }
};

// How it's turned, as a rotation vector: signed, on the short way round.
template <>
struct ReadingChannels<domain::spatial::Transform<float>> {
  using Channel = PoseChannel;
  static constexpr std::array kAll = {
      Channel::kX,
      Channel::kY,
      Channel::kZ,
      Channel::kTurnX,
      Channel::kTurnY,
      Channel::kTurnZ,
  };
  static void Emit(
      const domain::spatial::Transform<float>& value, const auto& emit
  ) {
    const domain::math::Vector3<float>& at = value.Translation();
    const domain::math::Vector3<float> turn = value.Rotation().Log();
    emit(Channel::kX, "x (m)", at.X());
    emit(Channel::kY, "y (m)", at.Y());
    emit(Channel::kZ, "z (m)", at.Z());
    emit(Channel::kTurnX, "turn x (rad)", turn.X());
    emit(Channel::kTurnY, "turn y (rad)", turn.Y());
    emit(Channel::kTurnZ, "turn z (rad)", turn.Z());
  }
};

template <typename D>
  requires requires(const D& value) {
    { value.Angular() } -> std::convertible_to<domain::math::Vector3<float>>;
    { value.Linear() } -> std::convertible_to<domain::math::Vector3<float>>;
  }
struct ReadingChannels<D> {
  using Channel = MotionChannel;
  static constexpr std::array kAll = {
      Channel::kAngularX,
      Channel::kAngularY,
      Channel::kAngularZ,
      Channel::kLinearX,
      Channel::kLinearY,
      Channel::kLinearZ,
  };
  static void Emit(const D& value, const auto& emit) {
    const domain::math::Vector3<float> angular = value.Angular();
    const domain::math::Vector3<float> linear = value.Linear();
    emit(Channel::kAngularX, "angular x", angular.X());
    emit(Channel::kAngularY, "angular y", angular.Y());
    emit(Channel::kAngularZ, "angular z", angular.Z());
    emit(Channel::kLinearX, "linear x", linear.X());
    emit(Channel::kLinearY, "linear y", linear.Y());
    emit(Channel::kLinearZ, "linear z", linear.Z());
  }
};

template <>
struct ReadingChannels<domain::actuation::EffortOutput> {
  using Channel = EffortChannel;
  static constexpr std::array kAll = {Channel::kEffort};
  static void Emit(
      const domain::actuation::EffortOutput& value, const auto& emit
  ) {
    emit(Channel::kEffort, "effort", value.effort);
  }
};

template <>
struct ReadingChannels<domain::actuation::DcMotorOutput> {
  using Channel = MotorChannel;
  static constexpr std::array kAll = {Channel::kEffort, Channel::kCurrent};
  static void Emit(
      const domain::actuation::DcMotorOutput& value, const auto& emit
  ) {
    emit(Channel::kEffort, "effort (N*m)", value.effort);
    emit(Channel::kCurrent, "current (A)", value.current);
  }
};

template <>
struct ReadingChannels<domain::sensing::SwitchReading> {
  using Channel = SwitchChannel;
  static constexpr std::array kAll = {Channel::kPressed, Channel::kPosition};
  static void Emit(
      const domain::sensing::SwitchReading& value, const auto& emit
  ) {
    emit(Channel::kPressed, "pressed", value.pressed ? 1.0F : 0.0F);
    emit(Channel::kPosition, "position", value.position);
  }
};

// Whether attachment kind K's reading has channels: what a ReadingView can
// show.
template <typename K>
concept HasChannels =
    requires { typename ReadingChannels<typename K::Output>::Channel; };

// Readings over time: each channel of what an attachment reads, plotted over
// its last few hundred samples, with its latest value. One template for every
// kind whose reading has channels (ReadingChannels): encoders, inertial and
// truth sensors, and what an actuator last applied -- only read, never
// pushed on: this display only shows. A camera's image and a lidar's scan
// have their own views.
//
// It's made with one attachment's handle, or N attachments' of the kind at
// once. Its settings pick what to show: `channels`, which of its reading's
// channels to plot (its own type's Channel, in any order -- every one by
// default); and, for N attachments, `names`, what to call each. With N > 1
// each channel is one plot, a line per attachment, so they can be told apart
// at a glance.
template <domain::attachment::AttachmentLike K, std::size_t N = 1>
  requires(N > 0 && HasChannels<K>)
class ReadingView final : public IDisplay {
  using Channels = ReadingChannels<typename K::Output>;

 public:
  // The channels its reading has: what `channels` lists.
  using Channel = typename Channels::Channel;
  using Handle = interface::AttachmentHandle<K>;

  static constexpr std::string_view kName = K::kListName;
  static constexpr float kWidth = 440.0F;
  static constexpr float kHeight = 330.0F;

  struct Settings {
    // Public on purpose: a display's settings (see display.hpp).
    std::vector<Channel> channels{Channels::kAll.begin(), Channels::kAll.end()};
    std::array<std::string_view, N> names{};
  };

  // One attachment's readings.
  explicit ReadingView(Handle handle)
    requires(N == 1)
      : ReadingView(std::array<Handle, 1>{std::move(handle)}, Settings{}) {}
  ReadingView(Handle handle, Settings settings)
    requires(N == 1)
      : ReadingView(
            std::array<Handle, 1>{std::move(handle)}, std::move(settings)
        ) {}
  // N attachments' readings, on shared plots.
  explicit ReadingView(std::array<Handle, N> handles)
      : ReadingView(std::move(handles), Settings{}) {}
  ReadingView(std::array<Handle, N> handles, Settings settings)
      : settings(std::move(settings)), handles_(std::move(handles)) {}

  Settings settings;

  void Draw(Frame* frame) const override {
    const auto first = handles_[0].Read();
    frame->text.push_back(
        std::string(K::kListName) + ", " +
        (first.sequence == 0
             ? std::string("no sample yet")
             : "sampled at " + Fixed(static_cast<float>(first.sample_time)) +
                   " s")
    );
    for (std::size_t i = 0; i < N; ++i) {
      DrawOne(handles_[i].Read(), i, frame);
    }
  }

 private:
  void DrawOne(const auto& reading, std::size_t index, Frame* frame) const {
    Channels::Emit(
        reading.value,
        [&](Channel channel, std::string_view label, float value) {
          if (std::ranges::find(settings.channels, channel) !=
              settings.channels.end()) {
            if constexpr (N == 1) {
              frame->traces.push_back(
                  {.label = std::string(label),
                   .value = value,
                   .time = reading.sample_time}
              );
            } else {
              frame->traces.push_back(
                  {.label = Name(index),
                   .value = value,
                   .time = reading.sample_time,
                   .plot = std::string(label)}
              );
            }
          }
        }
    );
  }

  std::string Name(std::size_t index) const {
    return settings.names[index].empty() ? "#" + std::to_string(index + 1)
                                         : std::string(settings.names[index]);
  }

  std::array<Handle, N> handles_;
};

}  // namespace achilles::display
