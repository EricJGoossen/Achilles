#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "domain/attachment/attachment.hpp"
#include "domain/error/effects.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles::domain::attachment {

// An attachment's settings, as its .arow file gives them (`config:`), read
// by whatever defines it -- a sensor kind's FromConfig, say: each by name,
// with a default for when it's absent unless it's required.
// Every setting given must be read -- Finish throws std::invalid_argument
// for any left over -- so a misspelled setting is an error rather than
// silently doing nothing. Whatever owns the settings names the attachment in
// what's thrown (AttachmentHandle::Configure).
class AttachmentConfig {
 public:
  using Entries = std::span<const std::pair<std::string, std::vector<double>>>;

  explicit AttachmentConfig(Entries entries)
      : entries_(entries), read_(entries.size(), false) {}

  float Scalar(std::string_view key, float fallback) {
    const std::vector<double>* values = Take(key, 1);
    return values == nullptr ? fallback : static_cast<float>((*values)[0]);
  }

  // A true/false setting (the .arow gives true/false or 1/0).
  bool Flag(std::string_view key, bool fallback) {
    const std::vector<double>* values = Take(key, 1);
    return values == nullptr ? fallback : (*values)[0] != 0.0;
  }

  std::uint32_t Count(std::string_view key, std::uint32_t fallback) {
    const std::vector<double>* values = Take(key, 1);
    if (values == nullptr) {
      return fallback;
    }
    if ((*values)[0] < 0.0) {
      Fail(key, "must not be negative");
    }
    return static_cast<std::uint32_t>((*values)[0]);
  }

  math::Vector3<float> Vector(
      std::string_view key, const math::Vector3<float>& fallback
  ) {
    const std::vector<double>* values = Take(key, 3);
    return values == nullptr ? fallback : ToVector(*values);
  }

  // The same, for a setting that has no default.
  math::Vector3<float> RequiredVector(std::string_view key) {
    const std::vector<double>* values = Take(key, 3);
    if (values == nullptr) {
      Fail(key, "is required");
    }
    return ToVector(*values);
  }

  // A pose, given like a joint's fixed_joint_transform: {translation:
  // [x, y, z], rotation: [w, x, y, z]}, seven numbers in that order; the
  // rotation is normalized.
  spatial::Transform<float> Pose(
      std::string_view key, const spatial::Transform<float>& fallback
  ) {
    const std::vector<double>* values = Take(key, 7);
    if (values == nullptr) {
      return fallback;
    }
    const auto at = [&](std::size_t i) {
      return static_cast<float>((*values)[i]);
    };
    const math::Quaternion<float> rotation(at(3), at(4), at(5), at(6));
    if (rotation.Norm() <= 0.0F) {
      Fail(key, "must not have a zero rotation");
    }
    return {math::Vector3<float>(at(0), at(1), at(2)), rotation.Normalize()};
  }

  // The options every attachment has (AttachmentOptions): `period`.
  AttachmentOptions Options() { return {Scalar("period", 0.0F)}; }

  // Where a sensor sits on its body: its `mount`, a pose in the body's frame,
  // identity when absent.
  spatial::Transform<float> BodyFromSensor() { return Pose("mount", {}); }

  // One error model (error::ErrorChain), from the settings named
  // `prefix` + scale_error, bias, bias_instability (the random walk's
  // density), noise_density, noise_sigma, range, resolution and delay_samples
  // -- each
  // left off when absent. A three-axis model's scale_error and bias take
  // three numbers.
  template <error::ErrorValueLike V>
  typename error::ErrorChain<V>::Params ErrorModel(
      std::string_view prefix = {}
  ) {
    using Chain = error::ErrorChain<V>;
    typename Chain::Params params;
    params.scale.error = Value(Key(prefix, "scale_error"), V{});
    params.bias.bias = Value(Key(prefix, "bias"), V{});
    params.walk.density = Scalar(Key(prefix, "bias_instability"), 0.0F);
    params.noise.density = Scalar(Key(prefix, "noise_density"), 0.0F);
    params.sample_noise.sigma = Scalar(Key(prefix, "noise_sigma"), 0.0F);
    params.range.limit = Scalar(Key(prefix, "range"), 0.0F);
    params.resolution.resolution = Scalar(Key(prefix, "resolution"), 0.0F);
    const std::string delay = Key(prefix, "delay_samples");
    params.latency.samples = Count(delay, 0);
    constexpr std::size_t kCapacity =
        std::tuple_size_v<decltype(typename Chain::Latency::State{}.ring)>;
    if (params.latency.samples >= kCapacity) {
      Fail(delay, "must be below " + std::to_string(kCapacity));
    }
    return params;
  }

  // Throws for any setting no read asked for.
  void Finish() const {
    for (std::size_t i = 0; i < entries_.size(); ++i) {
      if (!read_[i]) {
        throw std::invalid_argument(
            "unknown setting '" + entries_[i].first + "'."
        );
      }
    }
  }

 private:
  float Value(std::string_view key, float fallback) {
    return Scalar(key, fallback);
  }
  math::Vector3<float> Value(
      std::string_view key, const math::Vector3<float>& fallback
  ) {
    return Vector(key, fallback);
  }

  static math::Vector3<float> ToVector(const std::vector<double>& values) {
    return {
        static_cast<float>(values[0]),
        static_cast<float>(values[1]),
        static_cast<float>(values[2])
    };
  }

  static std::string Key(std::string_view prefix, std::string_view name) {
    return std::string(prefix) + std::string(name);
  }

  // The setting named `key`, marked read, checked to hold `count` numbers;
  // null when it isn't given.
  const std::vector<double>* Take(std::string_view key, std::size_t count) {
    for (std::size_t i = 0; i < entries_.size(); ++i) {
      if (entries_[i].first == key) {
        read_[i] = true;
        if (entries_[i].second.size() != count) {
          Fail(key, "takes " + std::to_string(count) + " number(s)");
        }
        return &entries_[i].second;
      }
    }
    return nullptr;
  }

  [[noreturn]] void Fail(std::string_view key, const std::string& why) const {
    throw std::invalid_argument(
        "setting '" + std::string(key) + "' " + why + "."
    );
  }

  Entries entries_;
  std::vector<bool> read_;
};

}  // namespace achilles::domain::attachment
