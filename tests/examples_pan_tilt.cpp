// Loads examples/pan_tilt/ and checks its rig is what its camera's mount
// assumes: at rest the camera looks back along the world's -x, level, and
// gravity leaves it there; turning the pan joint swings that view sideways,
// about the vertical, and the tilt joint raises or lowers it, about a
// horizontal axis. The example drives neither joint, so to turn them this
// loads a copy of it with a torque actuator added on each -- the test's
// own, never the example's. Needs no display -- only the camera, read
// through its handle -- but the camera renders on the GPU, so without one
// this skips.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>

#include "achilles/build_config.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sensing/camera.hpp"
#include "interface/simulation.hpp"
#include "support/temp_dir.hpp"

using achilles::domain::math::Vector3;
using achilles::domain::sensing::Camera;
using achilles::interface::Simulation;
using Torque = achilles::interface::truth::Torque;
using achilles::test_support::TempDir;

namespace {

constexpr float kDt = 0.002F;

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// A torque actuator on the rig's joint `joint`, named `<joint>_motor`.
std::string MotorFile(const std::string& joint) {
  return "actuator: " + joint + "_motor\nkind: angular\nattach: {archetype: " +
         "rig, joint: " + joint + "}\nconfig:\n  axis: [1, 0, 0]\n";
}

// examples/pan_tilt/ copied into `dir`, its rig with a torque actuator on
// each of its pan and tilt joints.
void WriteDrivenCopy(TempDir& dir) {
  const std::filesystem::path example =
      std::filesystem::path(ACHILLES_SOURCE_DIR) / "examples" / "pan_tilt";
  for (const auto& entry : std::filesystem::directory_iterator(example)) {
    if (entry.path().extension() != ".cpp") {
      dir.Write(entry.path().filename().string(), ReadFile(entry.path()));
    }
  }
  std::string rig = ReadFile(example / "rig.arow");
  const std::string includes = "  - rig_camera.arow\n";
  const std::size_t at = rig.find(includes);
  if (at == std::string::npos) {
    throw std::runtime_error("rig.arow no longer includes rig_camera.arow");
  }
  rig.insert(at + includes.size(), "  - pan_motor.arow\n  - tilt_motor.arow\n");
  dir.Write("rig.arow", rig);
  dir.Write("pan_motor.arow", MotorFile("pan"));
  dir.Write("tilt_motor.arow", MotorFile("tilt"));
}

class PanTilt : public ::testing::Test {
 protected:
  void SetUp() override {
    if constexpr (!achilles::kWithGpu) {
      GTEST_SKIP() << "cameras render only on the GPU";
    }
    WriteDrivenCopy(dir_);
    ASSERT_TRUE(sim_.LoadArowFile((dir_.Path() / "pan_tilt.arow").string()));
    ASSERT_TRUE(sim_.LoadConfigFile((dir_.Path() / "sim_config.yaml").string())
    );
    ASSERT_TRUE(sim_.Init(1));
  }

  // Steps `ticks` ticks, applying `torque` to `motor` before each (none
  // when `motor` is empty).
  void Run(int ticks, std::string_view motor = {}, float torque = 0.0F) {
    for (int i = 0; i < ticks; ++i) {
      if (!motor.empty()) {
        sim_.Actuator<Torque>(motor).Apply(torque);
      }
      ASSERT_TRUE(sim_.Step(kDt));
    }
  }

  // Where the camera's latest image looks, and which way is its up, in the
  // world.
  Vector3<float> Forward() const {
    return sim_.Sensor<Camera>("rig_camera")
        .Read()
        .value.world_from_camera.Rotation()
        .Rotate(Vector3<float>::UnitX());
  }
  Vector3<float> Up() const {
    return sim_.Sensor<Camera>("rig_camera")
        .Read()
        .value.world_from_camera.Rotation()
        .Rotate(Vector3<float>(0.0F, 0.0F, 1.0F));
  }

  TempDir dir_;
  Simulation sim_;
};

}  // namespace

TEST_F(PanTilt, AtRestTheCameraLooksBackAlongMinusXLevel) {
  Run(40);  // past the first (pipelined) image
  const Vector3<float> forward = Forward();
  EXPECT_NEAR(forward.X(), -1.0F, 1e-4F);
  EXPECT_NEAR(forward.Y(), 0.0F, 1e-4F);
  EXPECT_NEAR(forward.Z(), 0.0F, 1e-4F);
  EXPECT_NEAR(Up().Z(), 1.0F, 1e-4F);
  // From 5 m out, at the shoulder's height.
  const Vector3<float> at = sim_.Sensor<Camera>("rig_camera")
                                .Read()
                                .value.world_from_camera.Translation();
  EXPECT_NEAR(at.X(), 5.0F, 1e-4F);
  EXPECT_NEAR(at.Y(), 0.0F, 1e-4F);
  EXPECT_NEAR(at.Z(), 0.0F, 1e-4F);
}

TEST_F(PanTilt, PanningSwingsTheViewAboutTheVertical) {
  Run(200, "pan_motor", 0.05F);
  const Vector3<float> forward = Forward();
  EXPECT_GT(std::abs(forward.Y()), 0.05F);  // it turned
  EXPECT_NEAR(forward.Z(), 0.0F, 1e-3F);    // and stayed level
  EXPECT_NEAR(Up().Z(), 1.0F, 1e-3F);
}

TEST_F(PanTilt, TiltingRaisesOrLowersTheViewAboutAHorizontalAxis) {
  Run(200, "tilt_motor", 0.05F);
  const Vector3<float> forward = Forward();
  EXPECT_GT(std::abs(forward.Z()), 0.05F);  // it tilted
  EXPECT_NEAR(forward.Y(), 0.0F, 1e-3F);    // without turning sideways
}

TEST_F(PanTilt, GravityLeavesTheRigWhereItIs) {
  Run(500);
  EXPECT_NEAR(Forward().X(), -1.0F, 1e-4F);
}
