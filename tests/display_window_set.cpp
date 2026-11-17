// Opening, changing and closing windows at runtime, and the controllers
// riding on them writing to the sim (display::WindowSet, display::Controller)
// -- all without a screen: nothing here draws.
#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include "display/contract/controller.hpp"
#include "display/contract/input.hpp"
#include "display/views/lidar_view.hpp"
#include "display/views/reading_view.hpp"
#include "display/window_set.hpp"
#include "domain/actuation/truth_actuator.hpp"
#include "domain/attachment/sim_quantities.hpp"
#include "domain/sensing/gyroscope.hpp"
#include "domain/sensing/lidar.hpp"
#include "domain/spatial/dual.hpp"
#include "interface/attachment_handle.hpp"
#include "interface/simulation.hpp"
#include "interface/state_handle.hpp"

using achilles::display::Controller;
using achilles::display::Input;
using achilles::display::Key;
using achilles::display::LidarView;
using achilles::display::ReadingView;
using achilles::display::VectorChannel;
using achilles::display::WindowId;
using achilles::display::WindowSet;
using achilles::domain::sensing::Gyroscope;
using achilles::domain::sensing::Lidar;
using achilles::interface::AngularActuator;
using achilles::interface::Simulation;
using achilles::interface::StateHandle;
using Velocity = achilles::domain::spatial::SpatialVelocity<float>;
namespace input = achilles::domain::attachment::input;

namespace {

constexpr float kDt = 0.002F;

// Steers a wheel two ways at once: W commands its motor `effort`; Space
// sets the wheel spinning at `spin` rad/s outright. Neither held, it writes
// nothing.
class Steer final
    : public Controller<AngularActuator, StateHandle<input::JointVelocity>> {
 public:
  Steer(AngularActuator motor, StateHandle<input::JointVelocity> wheel)
      : Controller(std::move(motor), std::move(wheel)) {}

  float effort = 1.5F;
  float spin = 3.0F;

 protected:
  Writes Control(const Input& input) const override {
    Writes writes;
    if (input.Held(Key::kW)) {
      std::get<0>(writes) =
          achilles::domain::actuation::EffortCommand{.effort = effort};
    }
    if (input.Held(Key::kSpace)) {
      std::get<1>(writes) = Velocity(spin, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    }
    return writes;
  }
};

// The input of a focused window with `key` held.
Input Holding(Key key) {
  Input input;
  input.focused = true;
  input.keys_held[static_cast<std::size_t>(key)] = true;
  return input;
}

class Windows : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string dir =
        std::string(ACHILLES_SOURCE_DIR) + "/examples/maze/";
    ASSERT_TRUE(sim_.LoadArowFile(dir + "maze.arow"));
    ASSERT_TRUE(sim_.LoadConfigFile(dir + "sim_config.yaml"));
    ASSERT_TRUE(sim_.Init(1));
  }

  LidarView Scanner() const { return LidarView(sim_.Sensor<Lidar>("lidar")); }
  ReadingView<Gyroscope> Gyro() const {
    return ReadingView<Gyroscope>(sim_.Sensor<Gyroscope>("gyroscope"));
  }
  StateHandle<input::JointVelocity> Wheel(const std::string& joint) const {
    return sim_.State<input::JointVelocity>(
        {.archetype = "wheeled_robot", .joint = joint}
    );
  }
  Steer SteerOf(const std::string& wheel) const {
    return Steer(
        sim_.Actuator<achilles::interface::truth::Torque>(wheel + "_motor"),
        Wheel(wheel)
    );
  }

  float LastEffort(const std::string& motor) const {
    return sim_.Actuator<achilles::interface::truth::Torque>(motor)
        .Read()
        .value.effort;
  }
  float Spin(const std::string& wheel) const {
    return Wheel(wheel).Read().Angular().X();
  }

  Simulation sim_;
};

}  // namespace

TEST_F(Windows, AddFindAndRemoveAtAnyTime) {
  WindowSet set;
  const WindowId scanner = set.Add(Scanner(), "Lidar");
  ASSERT_TRUE(sim_.Step(kDt));
  const WindowId gyro = set.Add(Gyro(), "Gyroscope");
  EXPECT_EQ(set.Size(), 2U);
  EXPECT_NE(scanner, gyro);

  // Settings change through Find, for its own kind only.
  ASSERT_NE(set.Find<ReadingView<Gyroscope>>(gyro), nullptr);
  set.Find<ReadingView<Gyroscope>>(gyro)->settings.channels = {VectorChannel::kY
  };
  EXPECT_EQ(
      set.Find<ReadingView<Gyroscope>>(gyro)->settings.channels.size(), 1U
  );
  EXPECT_EQ(set.Find<ReadingView<Gyroscope>>(scanner), nullptr);

  EXPECT_TRUE(set.Remove(scanner));
  EXPECT_FALSE(set.Remove(scanner));
  EXPECT_FALSE(set.Contains(scanner));
  EXPECT_TRUE(set.Contains(gyro));
  EXPECT_EQ(set.Title(gyro), "Gyroscope");
  EXPECT_EQ(set.Title(scanner), "");

  // An id is never reused: a removed window's finds nothing new.
  const WindowId again = set.Add(Scanner(), "Lidar again");
  EXPECT_NE(again, scanner);
  EXPECT_EQ(set.Find<LidarView>(scanner), nullptr);
}

TEST_F(Windows, SwapPutsAnotherDisplayInTheSameWindow) {
  WindowSet set;
  const WindowId id = set.Add(Gyro(), "Sensor");
  set.Swap(id, Scanner());
  EXPECT_EQ(set.Size(), 1U);
  EXPECT_EQ(set.Find<ReadingView<Gyroscope>>(id), nullptr);
  EXPECT_NE(set.Find<LidarView>(id), nullptr);
  // The window, and so its title, stays.
  EXPECT_EQ(set.Title(id), "Sensor");
  EXPECT_THROW(set.Swap(WindowId{999}, Scanner()), std::invalid_argument);
}

// One controller, writing through an actuator's handle and a joint's state
// handle alike, every tick it's applied, until it's fed otherwise.
TEST_F(Windows, AControllersWritesLandEveryTickTheyreApplied) {
  WindowSet set;
  const WindowId id = set.Add(Scanner(), "Lidar");
  ASSERT_TRUE(set.Control(id, SteerOf("left_front")));
  ASSERT_TRUE(set.Feed(id, Holding(Key::kW)));

  for (int tick = 0; tick < 3; ++tick) {
    set.ApplyWrites();
    ASSERT_TRUE(sim_.Step(kDt));
    EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 1.5F) << "tick " << tick;
    EXPECT_FLOAT_EQ(LastEffort("left_rear_motor"), 0.0F);
  }

  // Space sets the wheel's spin outright, from the write on -- that wheel's
  // only.
  const float rear = Spin("left_rear");
  ASSERT_TRUE(set.Feed(id, Holding(Key::kSpace)));
  set.ApplyWrites();
  EXPECT_FLOAT_EQ(Spin("left_front"), 3.0F);
  EXPECT_FLOAT_EQ(Spin("left_rear"), rear);
}

// Fed nothing to say -- nothing held, or its window unfocused -- a controller
// writes nothing: a truth effort lapses after its tick, and a joint is left
// to the sim.
TEST_F(Windows, NothingToSayWritesNothing) {
  WindowSet set;
  const WindowId id = set.Add(Scanner(), "Lidar");
  ASSERT_TRUE(set.Control(id, SteerOf("left_front")));
  set.ApplyWrites();  // never fed
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 0.0F);

  ASSERT_TRUE(set.Feed(id, Holding(Key::kW)));
  set.ApplyWrites();
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 1.5F);

  ASSERT_TRUE(set.Feed(id, Input{}));
  set.ApplyWrites();
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 0.0F);
}

TEST_F(Windows, ControllersStayThroughASwapAndGoWithTheirWindow) {
  WindowSet set;
  const WindowId front = set.Add(Scanner(), "Front");
  const WindowId rear = set.Add(Gyro(), "Rear");
  ASSERT_TRUE(set.Control(front, SteerOf("left_front")));
  ASSERT_TRUE(set.Control(rear, SteerOf("left_rear")));
  EXPECT_FALSE(set.Control(WindowId{999}, SteerOf("left_front")));

  // Settings change through FindController, for its own kind only.
  ASSERT_NE(set.FindController<Steer>(front), nullptr);
  set.FindController<Steer>(front)->effort = 2.5F;
  EXPECT_EQ(set.FindController<Steer>(WindowId{999}), nullptr);

  set.Swap(front, Gyro());
  set.Remove(rear);
  ASSERT_TRUE(set.Feed(front, Holding(Key::kW)));
  EXPECT_FALSE(set.Feed(rear, Holding(Key::kW)));
  set.ApplyWrites();
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 2.5F);
  EXPECT_FLOAT_EQ(LastEffort("left_rear_motor"), 0.0F);

  // Released, the window stays up but writes nothing more.
  EXPECT_TRUE(set.Release(front));
  EXPECT_EQ(set.FindController<Steer>(front), nullptr);
  EXPECT_TRUE(set.Contains(front));
  set.ApplyWrites();
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), 0.0F);
}

// Two controllers on one window both write, in the order they were put on:
// a command is replaced, not added to, so the last one's stands.
TEST_F(Windows, TwoControllersOnOneWindowWriteInTurn) {
  WindowSet set;
  const WindowId id = set.Add(Scanner(), "Lidar");
  Steer first = SteerOf("left_front");
  Steer second = SteerOf("left_front");
  second.effort = -0.5F;
  ASSERT_TRUE(set.Control(id, std::move(first)));
  ASSERT_TRUE(set.Control(id, std::move(second)));
  ASSERT_TRUE(set.Feed(id, Holding(Key::kW)));
  set.ApplyWrites();
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(LastEffort("left_front_motor"), -0.5F);
}
