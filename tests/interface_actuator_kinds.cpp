// Actuator kinds through the real stack: what a command does once the sim runs
// it -- held or used up, at the kind's own rate, with its error, and restarted
// by a new configuration.
#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>

#include "domain/actuation/dc_motor.hpp"
#include "interface/attachment_handle.hpp"
#include "interface/simulation.hpp"
#include "support/sensing_scene.hpp"
#include "support/shaft_scene.hpp"
#include "support/temp_dir.hpp"

using achilles::domain::actuation::DcMotor;
using achilles::domain::actuation::DcMotorCommand;
using achilles::interface::Simulation;
using achilles::test_support::LoadShaft;
using achilles::test_support::ShaftRate;
using achilles::test_support::TempDir;

namespace {

constexpr float kDt = 0.002F;

// Kt = Ke = 0.3, 0.2 ohm, 24 V, 20 A; 10:1 down, 90% efficient.
const std::string kMotor =
    "  torque_constant: 0.3\n  back_emf_constant: 0.3\n"
    "  resistance: 0.2\n  bus_voltage: 24\n  max_current: 20\n"
    "  gear_ratio: 0.1\n  gearbox_efficiency: 0.9\n";

// 1 A at stall: 0.3 * 0.9 / 0.1 at the joint.
constexpr float kOneAmpStall = 2.7F;

void Steps(Simulation& sim, int count) {
  for (int i = 0; i < count; ++i) {
    ASSERT_TRUE(sim.Step(kDt));
  }
}

}  // namespace

TEST(ActuatorKinds, ReadIsEmptyUntilTheFirstStep) {
  TempDir dir;
  Simulation sim;
  ASSERT_TRUE(LoadShaft(dir, sim, "dc_motor", kMotor));
  const auto motor = sim.Actuator<DcMotor>("shaft_motor");
  EXPECT_EQ(motor.Read().sequence, 0U);
  EXPECT_EQ(motor.Read().value.effort, 0.0F);
}

// What the joint got, and what the driver's current feedback reports.
TEST(ActuatorKinds, ReadReportsTheEffortAndTheKindsOwnFeedback) {
  TempDir dir;
  Simulation sim;
  ASSERT_TRUE(LoadShaft(dir, sim, "dc_motor", kMotor));
  const auto motor = sim.Actuator<DcMotor>("shaft_motor");
  motor.Command(DcMotorCommand::Current(1.0F));
  Steps(sim, 1);
  const auto reading = motor.Read();
  EXPECT_EQ(reading.sequence, 1U);
  EXPECT_EQ(reading.sample_time, 0.0);
  EXPECT_NEAR(reading.value.effort, kOneAmpStall, 1e-4F);
  EXPECT_FLOAT_EQ(reading.value.current, 1.0F);
}

// Unlike a truth effort, a motor's command stands until the next one.
TEST(ActuatorKinds, AMotorHoldsItsLastCommand) {
  TempDir dir;
  Simulation sim;
  ASSERT_TRUE(LoadShaft(dir, sim, "dc_motor", kMotor));
  const auto motor = sim.Actuator<DcMotor>("shaft_motor");
  motor.Command(DcMotorCommand::Current(1.0F));
  Steps(sim, 5);
  EXPECT_EQ(motor.Read().sequence, 5U);
  EXPECT_FLOAT_EQ(motor.Read().value.current, 1.0F);
}

// Sampled at the tick's starting state, so it stamps that tick's time.
TEST(ActuatorKinds, UpdatesAreStampedWithTheSimClock) {
  TempDir dir;
  Simulation sim;
  ASSERT_TRUE(LoadShaft(dir, sim, "dc_motor", kMotor));
  const auto motor = sim.Actuator<DcMotor>("shaft_motor");
  Steps(sim, 4);
  EXPECT_NEAR(motor.Read().sample_time, 3.0 * kDt, 1e-6);
  EXPECT_NEAR(sim.Time(), 4.0 * kDt, 1e-6);
  EXPECT_EQ(sim.StepCount(), 4U);
}

// A new command waits for the next update, and until then the last effort
// keeps pushing -- a driver between control cycles.
TEST(ActuatorKinds, APeriodHoldsTheEffortBetweenUpdates) {
  TempDir dir;
  Simulation sim;
  ASSERT_TRUE(LoadShaft(dir, sim, "dc_motor", kMotor + "  period: 0.01\n"));
  const auto motor = sim.Actuator<DcMotor>("shaft_motor");
  motor.Command(DcMotorCommand::Current(1.0F));
  Steps(sim, 1);
  EXPECT_EQ(motor.Read().sequence, 1U);

  motor.Command(DcMotorCommand::Current(2.0F));
  const float rate_before = ShaftRate(sim);
  Steps(sim, 4);
  EXPECT_EQ(motor.Read().sequence, 1U);
  EXPECT_FLOAT_EQ(motor.Read().value.current, 1.0F);
  EXPECT_GT(ShaftRate(sim), rate_before);

  Steps(sim, 1);
  EXPECT_EQ(motor.Read().sequence, 2U);
  EXPECT_FLOAT_EQ(motor.Read().value.current, 2.0F);
}

TEST(ActuatorKinds, AKindsErrorReachesTheJoint) {
  TempDir dir;
  Simulation sim;
  ASSERT_TRUE(LoadShaft(dir, sim, "dc_motor", kMotor + "  bias: 1\n"));
  const auto motor = sim.Actuator<DcMotor>("shaft_motor");
  motor.Command(DcMotorCommand::Current(1.0F));
  Steps(sim, 1);
  EXPECT_NEAR(motor.Read().value.effort, kOneAmpStall + 1.0F, 1e-4F);
}

TEST(ActuatorKinds, TheSameSeedGivesTheSameNoise) {
  const auto noisy_effort = [](std::uint64_t seed) {
    TempDir dir;
    Simulation sim;
    EXPECT_TRUE(
        LoadShaft(dir, sim, "dc_motor", kMotor + "  noise_sigma: 0.5\n", seed)
    );
    Steps(sim, 3);
    return sim.Actuator<DcMotor>("shaft_motor").Read().value.effort;
  };
  EXPECT_EQ(noisy_effort(4), noisy_effort(4));
  EXPECT_NE(noisy_effort(4), noisy_effort(5));
}

// Replaced, it starts over: its command is gone and it updates again.
TEST(ActuatorKinds, SetConfigStartsAnActuatorOver) {
  TempDir dir;
  Simulation sim;
  ASSERT_TRUE(LoadShaft(dir, sim, "dc_motor", kMotor));
  const auto motor = sim.Actuator<DcMotor>("shaft_motor");
  motor.Command(DcMotorCommand::Current(1.0F));
  Steps(sim, 3);

  motor.SetConfig(motor.Config());
  EXPECT_EQ(motor.Read().sequence, 0U);
  Steps(sim, 1);
  EXPECT_EQ(motor.Read().sequence, 1U);
  EXPECT_FLOAT_EQ(motor.Read().value.current, 0.0F);
}

TEST(ActuatorKinds, ABadSettingFailsInit) {
  TempDir dir;
  Simulation sim;
  EXPECT_FALSE(LoadShaft(dir, sim, "dc_motor", kMotor + "  nope: 1\n"));
}

TEST(ActuatorKinds, AMotorWithoutItsRequiredSettingsFailsInit) {
  TempDir dir;
  Simulation sim;
  EXPECT_FALSE(LoadShaft(dir, sim, "dc_motor", ""));
}

TEST(ActuatorKinds, AskingForTheWrongKindThrows) {
  TempDir dir;
  Simulation sim;
  ASSERT_TRUE(LoadShaft(dir, sim, "angular", ""));
  EXPECT_THROW(
      (void)sim.Actuator<DcMotor>("shaft_motor"), std::invalid_argument
  );
}

// A truth sensor's first reading is the field's own initial value, before any
// Step: here, a shaft already turning. Its sequence stays 0.
TEST(ActuatorKinds, ATruthSensorIsValidFromInit) {
  TempDir dir;
  dir.Write(
      "rate.arow",
      achilles::test_support::SensorFile(
          "rate", "joint_velocity", "rig", "shaft"
      )
  );
  const auto file = dir.Write(
      "rig.arow",
      achilles::test_support::Archetype(
          "rig",
          {achilles::test_support::SceneJoint{
              .name = "shaft", .dof = 2, .velocity = {3, 0, 0, 0, 0, 0}
          }},
          {"rate.arow"}
      )
  );
  Simulation sim;
  ASSERT_TRUE(sim.LoadArowFile(file.string()));
  ASSERT_TRUE(sim.Init(1));
  const auto reading =
      sim.Sensor<achilles::interface::truth::JointVelocity>("rate").Read();
  EXPECT_EQ(reading.sequence, 0U);
  EXPECT_FLOAT_EQ(reading.value[0], 3.0F);
}
