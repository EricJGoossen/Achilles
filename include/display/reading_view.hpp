#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "display/display.hpp"
#include "display/frame.hpp"
#include "domain/actuation/dc_motor.hpp"
#include "domain/actuation/truth_actuator.hpp"
#include "domain/attachment/attachment_kind.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sensing/encoder.hpp"
#include "domain/spatial/transform.hpp"
#include "util/tmp.hpp"

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
// Its settings pick what to show: `channels`, which of its reading's
// channels to plot (its own type's Channel, in any order -- every one by
// default); and, for N attachments of
// the kind at once, `names`, what to call each. With N > 1 each channel is
// one plot, a line per attachment, so they can be told apart at a glance.
template <domain::attachment::AttachmentLike K, std::size_t N = 1>
  requires(N > 0 && HasChannels<K>)
struct ReadingView {
  using Channels = ReadingChannels<typename K::Output>;
  // The channels its reading has: what `channels` lists.
  using Channel = typename Channels::Channel;

  using Attachments = util::RepeatListT<util::TypeList<K>, N>;
  static constexpr std::string_view kName = K::kListName;
  static constexpr float kWidth = 440.0F;
  static constexpr float kHeight = 330.0F;

  // A display's settings: public on purpose (see display.hpp).
  std::vector<Channel> channels{Channels::kAll.begin(), Channels::kAll.end()};
  std::array<std::string_view, N> names{};

  void Draw(const HandlesForT<Attachments>& handles, Frame* frame) const {
    DrawEach(handles, frame, std::make_index_sequence<N>{});
  }

 private:
  template <std::size_t... Is>
  void DrawEach(
      const HandlesForT<Attachments>& handles,
      Frame* frame,
      std::index_sequence<Is...> /*indices*/
  ) const {
    const auto first = std::get<0>(handles).Read();
    frame->text.push_back(
        std::string(K::kListName) + ", " +
        (first.sequence == 0
             ? std::string("no sample yet")
             : "sampled at " + Fixed(static_cast<float>(first.sample_time)) +
                   " s")
    );
    (DrawOne(std::get<Is>(handles).Read(), Is, frame), ...);
  }

  void DrawOne(const auto& reading, std::size_t index, Frame* frame) const {
    Channels::Emit(
        reading.value,
        [&](Channel channel, std::string_view label, float value) {
          if (std::ranges::find(channels, channel) != channels.end()) {
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
    return names[index].empty() ? "#" + std::to_string(index + 1)
                                : std::string(names[index]);
  }
};

}  // namespace achilles::display
