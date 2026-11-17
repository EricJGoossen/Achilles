// Flying a camera by hand (display::FlyCamera): its window's keys and mouse,
// turned into its welded mount's pose, written outright -- here the arm's
// camera rig (examples/two_joint_arm/camera_rig.arow), welded to the world,
// z up. No screen: its input is made by hand.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <numbers>
#include <string>

#include "achilles/build_config.hpp"
#include "display/contract/input.hpp"
#include "display/controllers/fly_camera.hpp"
#include "domain/attachment/sim_quantities.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sensing/camera.hpp"
#include "domain/spatial/transform.hpp"
#include "interface/simulation.hpp"
#include "interface/state_handle.hpp"

using achilles::display::Button;
using achilles::display::FlyCamera;
using achilles::display::Input;
using achilles::display::Key;
using achilles::domain::sensing::Camera;
using achilles::interface::Simulation;
using Pose = achilles::domain::spatial::Transform<float>;
using Vector3 = achilles::domain::math::Vector3<float>;
namespace input = achilles::domain::attachment::input;

namespace {

constexpr float kDt = 0.002F;

void ExpectNear(const Vector3& actual, const Vector3& expected) {
  EXPECT_NEAR(actual.X(), expected.X(), 1e-4F);
  EXPECT_NEAR(actual.Y(), expected.Y(), 1e-4F);
  EXPECT_NEAR(actual.Z(), expected.Z(), 1e-4F);
}

// A focused window's input over one second, with `key` held.
Input Holding(Key key) {
  Input input;
  input.seconds = 1.0F;
  input.focused = true;
  input.keys_held[static_cast<std::size_t>(key)] = true;
  return input;
}

class Flying : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string dir =
        std::string(ACHILLES_SOURCE_DIR) + "/examples/two_joint_arm/";
    ASSERT_TRUE(sim_.LoadArowFile(dir + "two_joint_arm.arow"));
    ASSERT_TRUE(sim_.LoadConfigFile(dir + "sim_config.yaml"));
    ASSERT_TRUE(sim_.Init(1));
    // Once, so the mount's world pose is worked out.
    ASSERT_TRUE(sim_.Step(kDt));
  }

  FlyCamera::Joint Mount() const {
    return sim_.State<input::JointPosition>(
        {.archetype = "camera_rig", .joint = "mount"}
    );
  }
  Pose MountFromCamera() const {
    return sim_.Sensor<Camera>("arm_camera")
        .Config()
        .GetParams()
        .body_from_sensor;
  }
  FlyCamera Flyer() const {
    return FlyCamera(
        Mount(),
        sim_.State<const input::WorldTransform>(
            {.archetype = "camera_rig", .joint = "mount"}
        ),
        MountFromCamera(),
        {.up = sim_.Up()}
    );
  }
  // Where the camera is, in the world: its mount's pose, then the mount.
  Pose CameraPose() const { return Mount().Read() * MountFromCamera(); }

  Simulation sim_;
};

}  // namespace

// W held for a second carries the camera `speed` meters along where it
// looks, and turns it not at all.
TEST_F(Flying, ForwardIsWhereItLooks) {
  FlyCamera flyer = Flyer();
  const Pose before = CameraPose();
  const Vector3 ahead = before.Rotation().Rotate(Vector3(1.0F, 0.0F, 0.0F));

  flyer.Feed(Holding(Key::kW));
  flyer.ApplyWrites();
  const Pose after = CameraPose();
  ExpectNear(
      after.Translation(), before.Translation() + (ahead * flyer.settings.speed)
  );
  EXPECT_TRUE(after.Rotation().IsApprox(before.Rotation(), 1e-4F));

  // Shift: `fast` times as far.
  Input fast = Holding(Key::kS);
  fast.keys_held[static_cast<std::size_t>(Key::kShift)] = true;
  flyer.Feed(fast);
  flyer.ApplyWrites();
  ExpectNear(
      CameraPose().Translation(),
      after.Translation() -
          (ahead * (flyer.settings.speed * flyer.settings.fast))
  );
}

// E rises along the world's up -- against gravity, +z here -- wherever the
// camera looks.
TEST_F(Flying, UpIsTheWorldsUp) {
  ExpectNear(sim_.Up(), Vector3(0.0F, 0.0F, 1.0F));
  FlyCamera flyer = Flyer();
  const Pose before = CameraPose();
  flyer.Feed(Holding(Key::kE));
  flyer.ApplyWrites();
  ExpectNear(
      CameraPose().Translation(),
      before.Translation() + Vector3(0.0F, 0.0F, flyer.settings.speed)
  );
}

// A drag across turns the camera about the world's up, in place: where it
// looks swings round, its height above the horizon kept.
TEST_F(Flying, ADragAcrossTurnsItAboutUp) {
  FlyCamera flyer = Flyer();
  const Pose before = CameraPose();
  Input drag;
  drag.pointer.over = true;
  drag.pointer.held[static_cast<std::size_t>(Button::kLeft)] = true;
  drag.pointer.dx = 0.25F;  // a quarter of the way across: a quarter turn
  flyer.settings.look = 2.0F * std::numbers::pi_v<float>;
  flyer.Feed(drag);
  flyer.ApplyWrites();

  const Pose after = CameraPose();
  ExpectNear(after.Translation(), before.Translation());
  const Vector3 was = before.Rotation().Rotate(Vector3(1.0F, 0.0F, 0.0F));
  const Vector3 now = after.Rotation().Rotate(Vector3(1.0F, 0.0F, 0.0F));
  EXPECT_NEAR(now.Z(), was.Z(), 1e-4F);
  // Dragged right, it turns right: clockwise seen from above.
  ExpectNear(now, Vector3(was.Y(), -was.X(), was.Z()));
}

// Nothing held and no drag -- or a window not focused, which reports no
// keys -- writes nothing, so the camera stays wherever it was put.
TEST_F(Flying, NothingToDoWritesNothing) {
  FlyCamera flyer = Flyer();
  const Pose before = Mount().Read();
  Input idle;
  idle.seconds = 1.0F;
  idle.focused = true;
  flyer.Feed(idle);
  flyer.ApplyWrites();
  flyer.Feed(Input{});
  flyer.ApplyWrites();
  EXPECT_TRUE(Mount().Read().IsApprox(before, 1e-6F));
}

// Its mount welded, nothing but the flying moves the camera: it stays where
// it's flown as the sim runs on, and takes its images from there.
TEST_F(Flying, AWeldedMountStaysWhereItsFlown) {
  FlyCamera flyer = Flyer();
  flyer.Feed(Holding(Key::kW));
  flyer.ApplyWrites();
  const Pose flown = Mount().Read();
  for (int i = 0; i < 40; ++i) {
    ASSERT_TRUE(sim_.Step(kDt));
  }
  EXPECT_TRUE(Mount().Read().IsApprox(flown, 1e-5F));

  if constexpr (achilles::kWithGpu) {
    const auto frame = sim_.Sensor<Camera>("arm_camera").Read().value;
    ASSERT_GT(frame.width, 0U);
    ExpectNear(
        frame.world_from_camera.Translation(), CameraPose().Translation()
    );
  }
}
