#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <numbers>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/broad_phase/broad_phase_step.hpp"
#include "algorithms/contact_solver/contact_solver_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/integrators.hpp"
#include "algorithms/joint_limit/joint_limit_step.hpp"
#include "algorithms/narrow_phase/narrow_phase_step.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "interface/archetype_loader.hpp"
#include "support/registered_sim.hpp"
#include "support/temp_dir.hpp"
#include "util/tmp.hpp"

// Covers algorithms/integrators.hpp: the name/enum table, and -- through
// each entry's IntegratorStepFn, exactly the way interface::Simulation calls
// it -- that every integrator's step list adds up to the scheme it claims.
//
// Every physical check uses one revolute-Z joint loaded through the real
// LoadArchetypes/SimAllocator path, in one of two scenes:
//   - Constant acceleration: a constant torque on a diagonal inertia spun
//     about its own principal axis has no bias force, so qdd = tau / Izz
//     at every state. Verlet, implicit midpoint and RK4 are all exact for
//     the resulting quadratic trajectory; semi-implicit Euler has its own,
//     different, closed form.
//   - A gravity pendulum, whose acceleration depends on position:
//     qdd = k cos(theta). Constant acceleration can't tell a correct RK4
//     from one with the wrong stage weights (any weights with the right
//     average agree there), so each integrator is also checked tick by
//     tick against the same scheme written out independently in double
//     precision for this one ODE -- and the three lower-order schemes
//     against their own order of convergence.

using achilles::algorithms::Integrator;
using achilles::algorithms::IntegratorEntry;
using achilles::algorithms::IntegratorFromName;
using achilles::algorithms::kIntegrators;
using achilles::algorithms::NameOf;
using achilles::algorithms::RegisteredAlgorithms;
using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::SimConfig;
using achilles::algorithms::StepFnFor;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::algorithms::aba::ABAStep;
using achilles::interface::LoadArchetypes;
using achilles::test_support::TempDir;
using achilles::util::TypeList;
using Allocator =
    achilles::engine::memory::SimAllocatorForT<RegisteredAlgorithms>;

namespace {

using B = achilles::algorithms::MathematicalT;

// One revolute-Z joint at angle `q0` (subspace entry (2, 0), see
// tests/interface_simulation.cpp's RevoluteZFieldsYaml for the layout).
// `inertia` is the rigid_body_inertia leaf: [m, h_x, h_y, h_z, Ixx, Iyy,
// Izz, Ixy, Ixz, Iyz], h the first moment of mass, I about the joint.
std::string RevoluteZArow(
    const std::string& inertia,
    ScalarOperationT q0,
    ScalarOperationT qd0,
    ScalarOperationT tau0
) {
  std::ostringstream out;
  out << "archetype: base\n"
         "joints:\n"
         "  - name: joint\n"
         "    fields:\n"
         "      joint_subspace: [0,0,0,0,0,0,0,0,0,0,0,0,1,"
         "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]\n"
         "      joint_activation_mask: [1]\n"
         "      fixed_joint_transform: [0,0,0, 1,0,0,0]\n"
         "      rigid_body_inertia: "
      << inertia
      << "\n"
         "      joint_position: [0,0,0, "
      << std::cos(q0 * 0.5F) << ", 0,0, " << std::sin(q0 * 0.5F)
      << "]\n"
         "      joint_velocity: ["
      << qd0
      << ", 0,0,0,0,0]\n"
         "      joint_torque: ["
      << tau0 << ", 0,0,0,0,0]\n";
  return out.str();
}

// A diagonal inertia with no mass offset, Izz = 4: under a constant torque
// tau about Z, qdd = tau / 4 at every state.
constexpr ScalarOperationT kIzz = 4.0F;
const char* const kCenteredInertia = "[2, 0,0,0, 2,3,4, 0,0,0]";

// Unit mass with its center 0.5 along X, Izz about the joint raised by the
// parallel axis theorem (0.01 + 1 * 0.5^2). With base_acceleration
// (0, -9.8, 0) -- the base accelerating down, i.e. gravity +9.8 along Y in
// the joint's frame -- the torque about Z is 0.5 cos(theta) * 9.8, so
// qdd = k cos(theta) with k = 0.5 * 9.8 / 0.26.
const char* const kPendulumInertia = "[1, 0.5,0,0, 0.01,0.26,0.26, 0,0,0]";
constexpr double kPendulumK = 0.5 * 9.8 / 0.26;

struct Sim {
  Allocator::State state;
  SimConfig config;
};

Sim MakeSim(
    const TempDir& dir,
    const std::string& inertia,
    ScalarOperationT q0,
    ScalarOperationT qd0,
    ScalarOperationT tau0,
    const SimConfig& config
) {
  std::filesystem::path file =
      dir.Write("joint.arow", RevoluteZArow(inertia, q0, qd0, tau0));
  return {Allocator::Build(LoadArchetypes(file.string())), config};
}

SimConfig PendulumConfig() {
  SimConfig config;
  config.base_acceleration = achilles::algorithms::Acceleration(
      achilles::algorithms::Vector3::Zero(),
      achilles::algorithms::Vector3(B(0.0F), B(-9.8F), B(0.0F))
  );
  return config;
}

Sim MakePendulum(
    const TempDir& dir, ScalarOperationT q0, ScalarOperationT qd0
) {
  return MakeSim(dir, kPendulumInertia, q0, qd0, 0.0F, PendulumConfig());
}

void StepTicks(
    Sim& sim, Integrator integrator, ScalarOperationT dt, int ticks
) {
  for (int i = 0; i < ticks; ++i) {
    StepFnFor(integrator)(sim.state.context, sim.config, dt);
  }
}

// The joint's own rotation angle about Z, in (-pi, pi].
ScalarOperationT Angle(const Sim& sim) {
  auto q = sim.state.context.ViewFor<ABAAlgorithm>()
               .Load<ABAField::kJointPosition, ScalarOperationT>(0);
  return 2.0F * std::atan2(q.Rotation().Z(), q.Rotation().W());
}

ScalarOperationT Rate(const Sim& sim) {
  return sim.state.context.ViewFor<ABAAlgorithm>()
      .Load<ABAField::kJointVelocity, ScalarOperationT>(0)[0];
}

// ABA's qdd at the sim's current state, without integrating anything.
ScalarOperationT EvaluateAcceleration(Sim& sim) {
  sim.state.context.Step<TypeList<ABAStep>>(0.0F, sim.config);
  return sim.state.context.ViewFor<ABAAlgorithm>()
      .Load<ABAField::kJointAcceleration, ScalarOperationT>(0)[0];
}

// --- Each scheme, written out independently for theta'' = f(theta) ---
//
// Single-axis rotations compose by adding angles, so for this joint the
// engine's SE(3) updates (q * Exp(S qd h)) are exactly theta + qd h.

struct State {
  double theta;
  double rate;
};

double PendulumAcceleration(double theta) {
  return kPendulumK * std::cos(theta);
}

State ReferenceEuler(State s, double h) {
  double rate = s.rate + h * PendulumAcceleration(s.theta);
  return {s.theta + h * rate, rate};
}

State ReferenceVerlet(State s, double h) {
  double half_kicked = s.rate + 0.5 * h * PendulumAcceleration(s.theta);
  double theta = s.theta + h * half_kicked;
  return {theta, half_kicked + 0.5 * h * PendulumAcceleration(theta)};
}

State ReferenceMidpoint(State s, double h, std::size_t iterations) {
  double next_rate = s.rate;
  for (std::size_t i = 0; i < iterations; ++i) {
    double mid_rate = 0.5 * (s.rate + next_rate);
    double mid_theta = s.theta + 0.5 * h * mid_rate;
    next_rate = s.rate + h * PendulumAcceleration(mid_theta);
  }
  return {s.theta + h * 0.5 * (s.rate + next_rate), next_rate};
}

State ReferenceRungeKutta4(State s, double h) {
  double kv1 = s.rate;
  double ka1 = PendulumAcceleration(s.theta);
  double kv2 = s.rate + 0.5 * h * ka1;
  double ka2 = PendulumAcceleration(s.theta + 0.5 * h * kv1);
  double kv3 = s.rate + 0.5 * h * ka2;
  double ka3 = PendulumAcceleration(s.theta + 0.5 * h * kv2);
  double kv4 = s.rate + h * ka3;
  double ka4 = PendulumAcceleration(s.theta + h * kv3);
  return {
      s.theta + h / 6.0 * (kv1 + 2.0 * kv2 + 2.0 * kv3 + kv4),
      s.rate + h / 6.0 * (ka1 + 2.0 * ka2 + 2.0 * ka3 + ka4)
  };
}

State ReferenceTick(Integrator integrator, State s, double h) {
  switch (integrator) {
    case Integrator::kSemiImplicitEuler:
    case Integrator::kImplicitFast:
      return ReferenceEuler(s, h);
    case Integrator::kVelocityVerlet:
      return ReferenceVerlet(s, h);
    case Integrator::kImplicitMidpoint:
      return ReferenceMidpoint(
          s, h, achilles::algorithms::kImplicitMidpointIterations
      );
    case Integrator::kRungeKutta4:
      return ReferenceRungeKutta4(s, h);
    case Integrator::kCount:
      break;
  }
  return s;
}

// Angle after `duration` seconds of `integrator` at `dt`, starting at rest
// from horizontal. `dt` must divide `duration` exactly.
double PendulumAngle(
    Integrator integrator, ScalarOperationT dt, ScalarOperationT duration
) {
  TempDir dir;
  Sim sim = MakePendulum(dir, 0.0F, 0.0F);
  StepTicks(sim, integrator, dt, static_cast<int>(std::lround(duration / dt)));
  return Angle(sim);
}

}  // namespace

// Every entry's name must look up to that same entry -- this is the one
// mapping the CLI, the long-horizon test and Simulation all go through.
TEST(IntegratorTable, EveryNameRoundTripsToItsOwnIntegrator) {
  for (const IntegratorEntry& entry : kIntegrators) {
    EXPECT_EQ(IntegratorFromName(entry.name), entry.integrator) << entry.name;
    EXPECT_EQ(NameOf(entry.integrator), entry.name);
  }
}

// Duplicate names would make IntegratorFromName silently return the first.
TEST(IntegratorTable, NamesAreUnique) {
  std::set<std::string_view> names;
  for (const IntegratorEntry& entry : kIntegrators) {
    EXPECT_TRUE(names.insert(entry.name).second) << entry.name;
  }
}

TEST(IntegratorTable, UnknownNameIsNotAnIntegrator) {
  EXPECT_FALSE(IntegratorFromName("not-an-integrator").has_value());
  EXPECT_FALSE(IntegratorFromName("").has_value());
}

// The constant-acceleration tests below rest on this premise: the scene's
// ABA really does give qdd = tau / Izz.
TEST(IntegratorConstantAcceleration, ScenePremiseAccelerationIsTauOverIzz) {
  TempDir dir;
  Sim sim = MakeSim(dir, kCenteredInertia, 0.0F, 1.0F, 2.0F, SimConfig{});
  EXPECT_FLOAT_EQ(EvaluateAcceleration(sim), 2.0F / kIzz);
}

// Verlet, implicit midpoint and RK4 integrate a constant acceleration
// exactly: theta = qd0 t + a t^2 / 2, qd = qd0 + a t. Checked after several
// ticks, so a scheme that's only right for the first one still fails.
TEST(IntegratorConstantAcceleration, SecondOrderAndHigherSchemesAreExact) {
  constexpr ScalarOperationT kQd0 = 1.0F;
  constexpr ScalarOperationT kTau = 2.0F;
  constexpr ScalarOperationT kA = kTau / kIzz;
  constexpr ScalarOperationT kDt = 0.1F;
  constexpr int kTicks = 10;
  constexpr ScalarOperationT kT = kDt * kTicks;

  for (Integrator integrator :
       {Integrator::kVelocityVerlet,
        Integrator::kImplicitMidpoint,
        Integrator::kRungeKutta4}) {
    SCOPED_TRACE(std::string(NameOf(integrator)));
    TempDir dir;
    Sim sim = MakeSim(dir, kCenteredInertia, 0.0F, kQd0, kTau, SimConfig{});
    StepTicks(sim, integrator, kDt, kTicks);

    EXPECT_NEAR(Angle(sim), kQd0 * kT + 0.5F * kA * kT * kT, 1e-5F);
    EXPECT_NEAR(Rate(sim), kQd0 + kA * kT, 1e-5F);
  }
}

// Semi-implicit Euler (and implicitfast, identical here) updates qd first,
// then q from the new qd: after n ticks, qd = qd0 + a n dt exactly, but
// theta = qd0 n dt + a dt^2 n(n+1)/2 -- half a tick's worth of
// acceleration ahead of the exact answer. Checking that exact closed form
// (rather than "close to exact") is what distinguishes it from the
// higher-order schemes above.
TEST(IntegratorConstantAcceleration, SemiImplicitEulerMatchesItsOwnClosedForm) {
  constexpr ScalarOperationT kQd0 = 1.0F;
  constexpr ScalarOperationT kTau = 2.0F;
  constexpr ScalarOperationT kA = kTau / kIzz;
  constexpr ScalarOperationT kDt = 0.1F;
  constexpr int kTicks = 10;
  constexpr ScalarOperationT kN = kTicks;

  for (Integrator integrator :
       {Integrator::kSemiImplicitEuler, Integrator::kImplicitFast}) {
    SCOPED_TRACE(std::string(NameOf(integrator)));
    TempDir dir;
    Sim sim = MakeSim(dir, kCenteredInertia, 0.0F, kQd0, kTau, SimConfig{});
    StepTicks(sim, integrator, kDt, kTicks);

    EXPECT_NEAR(
        Angle(sim),
        kQd0 * kN * kDt + kA * kDt * kDt * kN * (kN + 1.0F) / 2.0F,
        1e-5F
    );
    EXPECT_NEAR(Rate(sim), kQd0 + kA * kN * kDt, 1e-5F);
  }
}

// The pendulum tests below rest on this premise: the scene's ABA gives
// qdd = k cos(theta), checked at two angles so both k and the cos(theta)
// shape are pinned down.
TEST(IntegratorPendulum, ScenePremiseAccelerationIsKCosTheta) {
  for (ScalarOperationT theta : {0.0F, 1.0F}) {
    TempDir dir;
    Sim sim = MakePendulum(dir, theta, 0.0F);
    EXPECT_NEAR(EvaluateAcceleration(sim), kPendulumK * std::cos(theta), 1e-4)
        << "theta " << theta;
  }
}

// Each integrator's tick is exactly its scheme: the engine must follow the
// independent double-precision version of that same scheme, tick by tick,
// to float precision. dt is large (h * k ~ 4.7) so that any departure -- a
// wrong stage weight, a kick at the wrong point, a different iteration
// count -- moves the result by far more than the tolerance.
TEST(IntegratorPendulum, EveryTickFollowsItsSchemeWrittenOutIndependently) {
  constexpr ScalarOperationT kDt = 0.25F;
  constexpr int kTicks = 8;

  for (const IntegratorEntry& entry : kIntegrators) {
    SCOPED_TRACE(std::string(entry.name));
    TempDir dir;
    Sim sim = MakePendulum(dir, 0.3F, 1.0F);
    State reference{0.3, 1.0};

    for (int tick = 0; tick < kTicks; ++tick) {
      StepTicks(sim, entry.integrator, kDt, 1);
      reference = ReferenceTick(entry.integrator, reference, kDt);
      ASSERT_NEAR(
          Angle(sim),
          std::remainder(reference.theta, 2.0 * std::numbers::pi),
          1e-4
      ) << "tick "
        << tick;
      ASSERT_NEAR(Rate(sim), reference.rate, 1e-4) << "tick " << tick;
    }
  }
}

// Each lower-order scheme converges at its own order: halving dt shrinks
// the error by 2^p. Measured from three runs at dt, dt/2, dt/4 (power-of-2
// steps, so every run ends at exactly the same time). RK4 isn't checked
// this way: in single precision its error reaches float rounding (~5e-6
// over these runs) before it settles into its asymptotic h^4 regime, so
// its observed order on this scene is noise -- the scheme test above
// covers it instead.
TEST(IntegratorPendulum, LowerOrderSchemesConvergeAtTheirOwnOrder) {
  constexpr ScalarOperationT kDuration = 2.0F;
  constexpr ScalarOperationT kDt = 0.0625F;

  for (const auto& [integrator, expected_order] :
       {std::pair{Integrator::kSemiImplicitEuler, 1.0},
        std::pair{Integrator::kVelocityVerlet, 2.0},
        std::pair{Integrator::kImplicitMidpoint, 2.0}}) {
    SCOPED_TRACE(std::string(NameOf(integrator)));
    double coarse = PendulumAngle(integrator, kDt, kDuration);
    double medium = PendulumAngle(integrator, kDt / 2.0F, kDuration);
    double fine = PendulumAngle(integrator, kDt / 4.0F, kDuration);
    double order =
        std::log2(std::abs(coarse - medium) / std::abs(medium - fine));
    EXPECT_NEAR(order, expected_order, 0.1);
  }
}

// implicitfast is semi-implicit Euler for this engine's force model (see
// Integrator::kImplicitFast) -- the same tick, bit for bit.
TEST(IntegratorPendulum, ImplicitFastIsBitIdenticalToSemiImplicitEuler) {
  EXPECT_EQ(
      PendulumAngle(Integrator::kImplicitFast, 0.0625F, 1.0F),
      PendulumAngle(Integrator::kSemiImplicitEuler, 0.0625F, 1.0F)
  );
}

// ---------------------------------------------------------------------------
// The collision pipeline, the one-time Setup list, and the solve inside
// semi-implicit Euler's tick.
// ---------------------------------------------------------------------------

// Collision is the broad phase, then the narrow phase over its pairs, then
// the joint-limit detector -- the three Steps that leave contacts and limit
// rows for the solver.
static_assert(std::is_same_v<
              achilles::algorithms::Collision,
              TypeList<
                  achilles::algorithms::broad_phase::BroadPhaseStep,
                  achilles::algorithms::narrow_phase::NarrowPhaseStep,
                  achilles::algorithms::joint_limit::JointLimitStep>>);

// Setup is mj_setConst: ABA at the initial pose, then the solver's inverse
// weights from its factors.
static_assert(std::is_same_v<
              achilles::algorithms::Setup,
              TypeList<
                  ABAStep,
                  achilles::algorithms::contact_solver::InvWeightStep>>);

// A pendulum dropped from horizontal swings far past 0.5 rad free (see
// IntegratorPendulum above); with an upper limit at 0.5 rad, semi-implicit
// Euler's tick -- ABA, collision, the solve, then VI/PI integrating the
// constrained qacc -- holds it there: it may sink into the soft limit
// briefly on impact, but comes to rest at it.
TEST(IntegratorJointLimit, SemiImplicitEulerHoldsAPendulumAtItsLimit) {
  using achilles::test_support::Dof0LimitFields;
  using achilles::test_support::MakeRegisteredSim;
  using achilles::test_support::RegisteredSim;
  using achilles::test_support::RevoluteZJointArow;
  constexpr ScalarOperationT kDt = 0.002F;

  TempDir dir;
  RegisteredSim sim = MakeRegisteredSim(
      dir,
      RevoluteZJointArow(
          "joint", "", kPendulumInertia, 0.0F, 0.0F,
          Dof0LimitFields(-0.5F, 0.5F)
      )
  );
  SimConfig config = PendulumConfig();
  sim.SimContext().Step<achilles::algorithms::Setup>(0.0F, config);

  double max_angle = 0.0;
  ScalarOperationT angle = 0.0F;
  for (int tick = 0; tick < 1000; ++tick) {
    StepFnFor(Integrator::kSemiImplicitEuler)(sim.SimContext(), config, kDt);
    auto q = sim.ViewFor<ABAAlgorithm>()
                 .Load<ABAField::kJointPosition, ScalarOperationT>(0);
    angle = 2.0F * std::atan2(q.Rotation().Z(), q.Rotation().W());
    max_angle = std::max(max_angle, static_cast<double>(angle));
  }

  EXPECT_LT(max_angle, 0.6);
  EXPECT_NEAR(angle, 0.5F, 0.01F);
}
