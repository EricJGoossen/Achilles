#pragma once

#include <memory>
#include <numbers>

#include "control/plant_state.hpp"
#include "control/sensors/distortions/i_distortion.hpp"
#include "control/sensors/i_sensor.hpp"
#include "dynamics/joints/abstract_joint.hpp"

namespace achilles::control::sensors {

class Encoder : public ISensor {
  public:
    Encoder(
        std::unique_ptr<distortions::IDistortion> distortion,
        const dynamics::joints::AbstractJoint& joint,
        double ticks_per_revolution,
        double initial_ticks
    )
      : distortion_(std::move(distortion)),
        joint_(&joint),
        ticks_per_revolution_(ticks_per_revolution),
        encoder_ticks_(initial_ticks) {}

    void Update(const PlantState& plant_state, double dt) override {
        double current_angle = joint_->Angle();
        double distorted_angle =
            distortion_->AddDistortion(current_angle - last_angle_, dt);

        encoder_ticks_ += distorted_angle / kTwoPi * ticks_per_revolution_;
        last_angle_ = current_angle;
    }

    double GetData() const { return encoder_ticks_; }

  private:
    static constexpr double kTwoPi = std::numbers::pi * 2;

    std::unique_ptr<distortions::IDistortion> distortion_;
    const dynamics::joints::AbstractJoint* joint_;

    const double ticks_per_revolution_;
    double encoder_ticks_;

    double last_angle_ = 0;
};
}  // namespace achilles::control::sensors