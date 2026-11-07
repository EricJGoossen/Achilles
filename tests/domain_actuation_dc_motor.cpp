// DcMotor: a current- or voltage-commanded motor through a gearbox, checked
// against the model's own equations at the operating points that matter
// to a firmware's control loops -- stall, the current limit, the back-EMF
// limit toward free speed, back-driving, and friction.
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "domain/actuation/dc_motor.hpp"
#include "domain/attachment/attachment.hpp"
#include "domain/math/vector3.hpp"
#include "interface/simulation.hpp"
#include "support/shaft_scene.hpp"
#include "support/temp_dir.hpp"

using achilles::domain::actuation::DcMotor;
using achilles::domain::actuation::DcMotorCommand;
using achilles::domain::actuation::DcMotorOutput;
using achilles::domain::actuation::DcMotorState;
using achilles::domain::attachment::AttachmentOptions;
using achilles::domain::attachment::AttachmentTick;
using achilles::interface::Simulation;
using achilles::test_support::LoadShaft;
using achilles::test_support::ShaftRate;
using achilles::test_support::TempDir;

namespace {

// Kt = Ke = 0.3, 0.2 ohm, 24 V, 20 A; 10:1 down (gear ratio 0.1), 90%
// efficient: free speed 80 rad/s at the rotor, 8 at the joint.
DcMotor::Params TestMotor() {
  DcMotor::Params params;
  params.torque_constant = 0.3F;
  params.back_emf_constant = 0.3F;
  params.resistance = 0.2F;
  params.bus_voltage = 24.0F;
  params.max_current = 20.0F;
  params.gear_ratio = 0.1F;
  params.losses.transmission.efficiency = 0.9F;
  return params;
}

const achilles::domain::math::Vector3<float> kAxis(0.0F, 0.0F, 1.0F);

// One update of a motor on a joint that yaws, at joint speed `omega`.
DcMotorOutput Spin(
    const DcMotor::Params& params, DcMotorCommand command, float omega
) {
  const DcMotor motor(AttachmentOptions{}, kAxis, params);
  DcMotor::Subspace subspace{};
  subspace(2, 0) = 1.0F;
  const DcMotor::Velocity qd(omega, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  DcMotorState state;
  DcMotorOutput out;
  motor.Drive(AttachmentTick{0.002F}, qd, subspace, &command, &state, &out);
  return out;
}

}  // namespace

// At stall the commanded current flows: Kt I, through the gearbox's
// efficiency and ratio.
TEST(Motor, StallTorqueIsKtTimesCurrentThroughTheGearbox) {
  const auto out = Spin(TestMotor(), DcMotorCommand::Current(10.0F), 0.0F);
  EXPECT_NEAR(out.effort, 0.3F * 10.0F * 0.9F / 0.1F, 1e-4F);
  EXPECT_FLOAT_EQ(out.current, 10.0F);
}

// A command beyond the driver's limit is clipped to it.
TEST(Motor, CurrentIsHeldToTheDriversLimit) {
  const auto out = Spin(TestMotor(), DcMotorCommand::Current(50.0F), 0.0F);
  EXPECT_FLOAT_EQ(out.current, 20.0F);
}

// Near free speed the bus can't push the commanded current against the
// back-EMF, so torque falls to zero at free speed -- what limits a
// firmware's speed loop on the real motor.
TEST(Motor, BackEmfLimitsCurrentTowardFreeSpeed) {
  const auto command = DcMotorCommand::Current(10.0F);
  // Rotor 79 rad/s: 0.3 V of headroom.
  EXPECT_NEAR(Spin(TestMotor(), command, 7.9F).current, 1.5F, 1e-3F);
  EXPECT_NEAR(Spin(TestMotor(), command, 8.0F).effort, 0.0F, 1e-4F);
}

// Past free speed the back-EMF would drive more current than the driver allows
// -- 30 V against 24 here, -30 A -- but it's still held to the limit.
TEST(Motor, PastFreeSpeedCurrentIsStillHeldToTheLimit) {
  EXPECT_FLOAT_EQ(
      Spin(TestMotor(), DcMotorCommand::Current(10.0F), 10.0F).current, -20.0F
  );
  EXPECT_FLOAT_EQ(
      Spin(TestMotor(), DcMotorCommand::Current(-10.0F), -10.0F).current, 20.0F
  );
}

// Voltage mode: the winding's own I = (V - Ke w) / R, within the limit.
TEST(Motor, VoltageCommandSetsCurrentThroughTheWinding) {
  const auto command = DcMotorCommand::Voltage(12.0F);
  // Back-EMF 12 V.
  EXPECT_NEAR(Spin(TestMotor(), command, 4.0F).current, 0.0F, 1e-4F);
  // Back-EMF 6 V: 30 A, limited.
  EXPECT_NEAR(Spin(TestMotor(), command, 2.0F).current, 20.0F, 1e-4F);
}

// A voltage past the bus is held to the bus: back-EMF 21 V leaves 3 V.
TEST(Motor, VoltageIsHeldToTheBus) {
  const auto out = Spin(TestMotor(), DcMotorCommand::Voltage(100.0F), 7.0F);
  EXPECT_NEAR(out.current, 15.0F, 1e-3F);
}

// Back-driven, the gearbox loses nothing: the efficiency applies only when
// the motor drives the load.
TEST(Motor, BackDrivenGearboxIsLossless) {
  const auto out = Spin(TestMotor(), DcMotorCommand::Current(-10.0F), 1.0F);
  EXPECT_NEAR(out.effort, -0.3F * 10.0F / 0.1F, 1e-3F);
}

// No command yet gives no torque.
TEST(Motor, NoCommandGivesNoTorque) {
  EXPECT_FLOAT_EQ(Spin(TestMotor(), DcMotorCommand{}, 0.0F).effort, 0.0F);
}

// Friction opposes the rotor's motion, at the joint through the ratio.
TEST(Motor, FrictionOpposesMotion) {
  DcMotor::Params params = TestMotor();
  params.losses.viscous.coefficient = 0.01F;
  params.losses.coulomb.level = 0.1F;
  const float expected = -(0.01F * 10.0F + 0.1F * std::tanh(5.0F)) / 0.1F;
  EXPECT_NEAR(Spin(params, DcMotorCommand{}, 1.0F).effort, expected, 1e-4F);
}

// The error model acts on what reaches the joint, not the winding's current.
TEST(Motor, ItsErrorModelActsOnTheJointTorque) {
  DcMotor::Params params = TestMotor();
  params.error.bias.bias = 1.0F;
  const auto out = Spin(params, DcMotorCommand::Current(10.0F), 0.0F);
  EXPECT_NEAR(out.effort, 27.0F + 1.0F, 1e-3F);
  EXPECT_FLOAT_EQ(out.current, 10.0F);
}

TEST(Motor, ItsRangeLimitsTheJointTorque) {
  DcMotor::Params params = TestMotor();
  params.error.range.limit = 5.0F;
  const auto out = Spin(params, DcMotorCommand::Current(10.0F), 0.0F);
  EXPECT_FLOAT_EQ(out.effort, 5.0F);
  EXPECT_FLOAT_EQ(out.current, 10.0F);
}

// A motor that can't turn, or whose math has no answer, is rejected up front.
TEST(Motor, RejectsParamsItCantRunOn) {
  const auto rejected = [](void (*change)(DcMotor::Params&)) {
    DcMotor::Params params = TestMotor();
    change(params);
    EXPECT_THROW(
        DcMotor(AttachmentOptions{}, kAxis, params), std::invalid_argument
    );
  };
  rejected([](DcMotor::Params& p) { p.torque_constant = 0.0F; });
  rejected([](DcMotor::Params& p) { p.bus_voltage = 0.0F; });
  rejected([](DcMotor::Params& p) { p.max_current = 0.0F; });
  rejected([](DcMotor::Params& p) { p.resistance = 0.0F; });
  rejected([](DcMotor::Params& p) { p.gear_ratio = 0.0F; });
  rejected([](DcMotor::Params& p) { p.losses.coulomb.smoothing = 0.0F; });
  rejected([](DcMotor::Params& p) { p.losses.transmission.efficiency = 0.0F; });
  rejected([](DcMotor::Params& p) { p.losses.transmission.efficiency = 1.5F; });
  rejected([](DcMotor::Params& p) { p.back_emf_constant = -1.0F; });
  rejected([](DcMotor::Params& p) { p.losses.viscous.coefficient = -1.0F; });
  rejected([](DcMotor::Params& p) { p.losses.coulomb.level = -1.0F; });
}

// No back-EMF constant means no free speed: the bus only limits the stall.
TEST(Motor, WithoutBackEmfThereIsNoFreeSpeed) {
  DcMotor::Params params = TestMotor();
  params.back_emf_constant = 0.0F;
  const auto out = Spin(params, DcMotorCommand::Current(10.0F), 1000.0F);
  EXPECT_FLOAT_EQ(out.current, 10.0F);
}

// A dc_motor actuator reaches the joint: the sim runs the motor, and the shaft
// spins up the way the torque points.
TEST(Motor, TheSimDrivesTheJoint) {
  TempDir dir;
  Simulation sim;
  ASSERT_TRUE(LoadShaft(
      dir,
      sim,
      "dc_motor",
      "  torque_constant: 0.3\n  back_emf_constant: 0.3\n"
      "  resistance: 0.2\n  bus_voltage: 24\n  max_current: 20\n"
      "  gear_ratio: 0.1\n  gearbox_efficiency: 0.9\n"
  ));
  sim.Actuator<DcMotor>("shaft_motor").Command(DcMotorCommand::Current(1.0F));
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(sim.Step(0.002F));
  }
  EXPECT_GT(ShaftRate(sim), 0.0F);
}
