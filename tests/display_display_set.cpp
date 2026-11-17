// Attaching, changing and removing displays at runtime, and the efforts they
// put on actuators (display::DisplaySet) -- all without a screen: nothing
// here draws.
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "display/contract/display.hpp"
#include "display/contract/frame.hpp"
#include "display/contract/input.hpp"
#include "display/display_set.hpp"
#include "display/views/camera/camera_view.hpp"
#include "display/views/effort_control.hpp"
#include "display/views/lidar_view.hpp"
#include "interface/attachment_handle.hpp"
#include "interface/simulation.hpp"
#include "support/display_handles.hpp"

using achilles::display::CameraView;
using achilles::display::DisplayId;
using achilles::display::DisplayLike;
using achilles::display::DisplaySet;
using achilles::display::Frame;
using achilles::display::HandlesForT;
using achilles::display::HandlesOfT;
using achilles::display::Input;
using achilles::display::LidarView;
using achilles::display::ReadingView;
using achilles::display::TorqueControl;
using achilles::domain::sensing::Camera;
using achilles::domain::sensing::Encoder;
using achilles::domain::sensing::Lidar;
using achilles::interface::AttachmentHandle;
using achilles::interface::Simulation;
using achilles::test_support::HandlesNamed;
using Torque = achilles::interface::truth::Torque;

// A display takes handles of exactly the kinds it names: a camera view can't
// be bound to an encoder, nor a torque control to a camera.
static_assert(!std::is_constructible_v<
              HandlesOfT<CameraView>,
              AttachmentHandle<Encoder>>);
static_assert(!std::is_constructible_v<
              HandlesOfT<TorqueControl>,
              AttachmentHandle<Camera>>);

namespace {

// A would-be display showing a camera with a slider for an actuator: sensors
// are shown and actuators driven by separate displays, so it isn't one.
struct CameraWithMotor {
  using Attachments = achilles::util::TypeList<Camera, Torque>;
  static constexpr std::string_view kName = "camera with motor";
  static constexpr float kWidth = 100.0F;
  static constexpr float kHeight = 100.0F;
  void Draw(const HandlesForT<Attachments>& /*handles*/, Frame* /*frame*/)
      const {}
  achilles::display::EffortsForT<Attachments> Efforts(const Input& /*input*/
  ) const {
    return {};
  }
};
static_assert(!DisplayLike<CameraWithMotor>);
static_assert(DisplayLike<CameraView>);
static_assert(DisplayLike<TorqueControl>);
// An actuator can be shown -- read, never pushed on -- or driven: only the
// display that drives it gives it efforts.
static_assert(DisplayLike<ReadingView<Torque>>);
static_assert(!achilles::display::Drives<ReadingView<Torque>>);
static_assert(achilles::display::Drives<TorqueControl>);

constexpr float kDt = 0.002F;

class Displays : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string dir =
        std::string(ACHILLES_SOURCE_DIR) + "/examples/maze/";
    ASSERT_TRUE(sim_.LoadArowFile(dir + "maze.arow"));
    ASSERT_TRUE(sim_.LoadConfigFile(dir + "sim_config.yaml"));
    ASSERT_TRUE(sim_.Init(1));
  }

  HandlesOfT<LidarView> Scanner() const {
    return HandlesNamed<LidarView>(sim_, "lidar");
  }
  HandlesOfT<TorqueControl> Motor(std::string_view name) const {
    return HandlesNamed<TorqueControl>(sim_, name);
  }

  float LastEffort(std::string_view name) const {
    return sim_.Actuator<Torque>(name).Read().value.effort;
  }

  Simulation sim_;
};

}  // namespace

TEST_F(Displays, AddFindAndRemoveAtAnyTime) {
  DisplaySet set;
  const DisplayId scanner = set.Add(LidarView{}, Scanner(), "Lidar");
  ASSERT_TRUE(sim_.Step(kDt));
  const DisplayId rear =
      set.Add(TorqueControl{}, Motor("left_rear_motor"), "Left rear");
  EXPECT_EQ(set.Size(), 2U);
  EXPECT_NE(scanner, rear);

  // Settings change through Find, for its own kind only.
  ASSERT_NE(set.Find<TorqueControl>(rear), nullptr);
  set.Find<TorqueControl>(rear)->max_effort = 12.0F;
  EXPECT_FLOAT_EQ(set.Find<TorqueControl>(rear)->max_effort, 12.0F);
  EXPECT_EQ(set.Find<TorqueControl>(scanner), nullptr);

  EXPECT_TRUE(set.Remove(scanner));
  EXPECT_FALSE(set.Remove(scanner));
  EXPECT_FALSE(set.Contains(scanner));
  EXPECT_TRUE(set.Contains(rear));
  EXPECT_EQ(set.Title(rear), "Left rear");
  EXPECT_EQ(set.Title(scanner), "");

  // An id is never reused: a removed display's finds nothing new.
  const DisplayId again = set.Add(LidarView{}, Scanner(), "Lidar again");
  EXPECT_NE(again, scanner);
  EXPECT_EQ(set.Find<LidarView>(scanner), nullptr);
}

TEST_F(Displays, SwapPutsAnotherDisplayInTheSameWindow) {
  DisplaySet set;
  const DisplayId id =
      set.Add(TorqueControl{}, Motor("left_front_motor"), "Left front");
  set.Swap(id, LidarView{}, Scanner());
  EXPECT_EQ(set.Size(), 1U);
  EXPECT_EQ(set.Find<TorqueControl>(id), nullptr);
  EXPECT_NE(set.Find<LidarView>(id), nullptr);
  // The window, and so its title, stays.
  EXPECT_EQ(set.Title(id), "Left front");
  EXPECT_THROW(
      set.Swap(DisplayId{999}, LidarView{}, Scanner()), std::invalid_argument
  );
}

TEST_F(Displays, EffortsReachEachActuatorEveryTickTheyreApplied) {
  DisplaySet set;
  const DisplayId left =
      set.Add(TorqueControl{}, Motor("left_front_motor"), "Left front");
  const DisplayId right =
      set.Add(TorqueControl{}, Motor("right_front_motor"), "Right front");
  ASSERT_TRUE(set.Feed(left, Input{{1.5F}}));
  ASSERT_TRUE(set.Feed(right, Input{{-0.5F}}));

  for (int tick = 0; tick < 3; ++tick) {
    set.ApplyEfforts();
    ASSERT_TRUE(sim_.Step(kDt));
    EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 1.5F) << "tick " << tick;
    EXPECT_FLOAT_EQ(LastEffort("right_front_motor"), -0.5F) << "tick " << tick;
    EXPECT_FLOAT_EQ(LastEffort("left_rear_motor"), 0.0F);
  }
  // A truth effort lasts its one tick: not applied, none.
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 0.0F);
}

// Showing an actuator only reads it: fed whatever, the display that shows a
// motor never pushes on it.
TEST_F(Displays, ADisplayShowingAnActuatorPushesOnNothing) {
  DisplaySet set;
  const DisplayId shown = set.Add(
      ReadingView<Torque>{},
      HandlesNamed<ReadingView<Torque>>(sim_, "left_front_motor"),
      "Left front torque"
  );
  ASSERT_TRUE(set.Feed(shown, Input{{3.0F}}));
  set.ApplyEfforts();
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 0.0F);
}

TEST_F(Displays, ASensorsDisplayPushesOnNothing) {
  DisplaySet set;
  const DisplayId scanner = set.Add(LidarView{}, Scanner(), "Lidar");
  // Whatever it's fed, a sensor's display has no efforts to give.
  ASSERT_TRUE(set.Feed(scanner, Input{{3.0F, 3.0F}}));
  set.ApplyEfforts();
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 0.0F);
  EXPECT_FLOAT_EQ(LastEffort("right_front_motor"), 0.0F);
}

TEST_F(Displays, ARemovedDisplayAsksNothingMore) {
  DisplaySet set;
  const DisplayId rear =
      set.Add(TorqueControl{}, Motor("left_rear_motor"), "Left rear");
  const DisplayId left =
      set.Add(TorqueControl{}, Motor("left_front_motor"), "Left front");
  set.Feed(rear, Input{{4.0F}});
  set.Feed(left, Input{{1.0F}});
  set.Remove(left);
  set.ApplyEfforts();
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(LastEffort("left_rear_motor"), 4.0F);
  EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 0.0F);
  EXPECT_FALSE(set.Feed(left, Input{{1.0F}}));
}

TEST_F(Displays, TwoDisplaysOnOneActuatorAdd) {
  DisplaySet set;
  const DisplayId a = set.Add(TorqueControl{}, Motor("left_front_motor"), "A");
  const DisplayId b = set.Add(TorqueControl{}, Motor("left_front_motor"), "B");
  set.Feed(a, Input{{0.25F}});
  set.Feed(b, Input{{0.5F}});
  set.ApplyEfforts();
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 0.75F);
}
