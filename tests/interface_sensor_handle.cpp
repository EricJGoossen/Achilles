// Finding, reading and reconfiguring sensors through Simulation, and the
// .arow sensor files they come from.
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "algorithms/sensing/encoder.hpp"
#include "algorithms/sensing/gyroscope.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "interface/sensor_handle.hpp"
#include "interface/simulation.hpp"
#include "support/sensing_scene.hpp"
#include "support/temp_dir.hpp"

using achilles::algorithms::sensing::Encoder;
using achilles::algorithms::sensing::Gyroscope;
using achilles::algorithms::sensing::SensorOptions;
using achilles::interface::Simulation;
using achilles::test_support::Archetype;
using achilles::test_support::kZUpConfig;
using achilles::test_support::SceneJoint;
using achilles::test_support::SensorFile;
using achilles::test_support::TempDir;

namespace {

constexpr float kDt = 0.002F;

// A world with three copies of a one-joint arm, each carrying a gyroscope
// (and whatever extra sensor files a test adds).
class Arms : public ::testing::Test {
 protected:
  void Write(
      const std::vector<std::string>& gyro_config = {},
      const std::vector<std::string>& extra_includes = {}
  ) {
    SceneJoint world{.name = "world"};
    dir_.Write("world.arow", Archetype("world", {world}, {"arm.arow"}));
    std::vector<std::string> includes = {"gyro.arow"};
    includes.insert(
        includes.end(), extra_includes.begin(), extra_includes.end()
    );
    std::string arm = Archetype(
        "arm",
        {SceneJoint{.name = "link", .dof = 2, .velocity = {1.0F, 0, 0, 0, 0, 0}}
        },
        includes,
        "world",
        "world"
    );
    dir_.Write("arm.arow", "copies: 3\n" + arm);
    dir_.Write(
        "gyro.arow", SensorFile("gyro", "gyroscope", "arm", "link", gyro_config)
    );
    dir_.Write("config.yaml", kZUpConfig);
  }

  bool Load(Simulation& sim, std::uint64_t seed = 1) {
    return sim.LoadArowFile((dir_.Path() / "world.arow").string()) &&
           sim.LoadConfigFile((dir_.Path() / "config.yaml").string()) &&
           sim.Init(seed);
  }

  void Steps(Simulation& sim, int count) {
    for (int i = 0; i < count; ++i) {
      ASSERT_TRUE(sim.Step(kDt));
    }
  }

  TempDir dir_;
};

}  // namespace

// ---------------------------------------------------------------------------
// Finding sensors
// ---------------------------------------------------------------------------

TEST_F(Arms, EveryCopyOfAnArchetypeCarriesItsOwnInstanceOfASensor) {
  Write();
  Simulation sim;
  ASSERT_TRUE(Load(sim));
  Steps(sim, 5);
  for (std::size_t copy = 0; copy < 3; ++copy) {
    const auto reading = sim.Read(sim.FindSensor<Gyroscope>({"gyro", copy}));
    EXPECT_NEAR(reading.value.Z(), 1.0F, 1e-4F) << "copy " << copy;
  }
}

TEST_F(Arms, FindSensorRejectsAnUnknownNameOrInstance) {
  Write();
  Simulation sim;
  ASSERT_TRUE(Load(sim));
  EXPECT_THROW(
      (void)sim.FindSensor<Gyroscope>({"nope", 0}), std::invalid_argument
  );
  EXPECT_THROW(
      (void)sim.FindSensor<Gyroscope>({"gyro", 3}), std::invalid_argument
  );
}

TEST_F(Arms, FindSensorRejectsTheWrongKind) {
  Write();
  Simulation sim;
  ASSERT_TRUE(Load(sim));
  EXPECT_THROW(
      (void)sim.FindSensor<Encoder>({"gyro", 0}), std::invalid_argument
  );
}

TEST(SensorHandleBeforeInit, FindSensorThrows) {
  Simulation sim;
  EXPECT_THROW(
      (void)sim.FindSensor<Gyroscope>({"gyro", 0}), std::runtime_error
  );
}

// ---------------------------------------------------------------------------
// Sampling and configuration
// ---------------------------------------------------------------------------

TEST_F(Arms, APeriodSamplesOnlyThatOften) {
  Write({"period: 0.01"});  // every 5 ticks of kDt
  Simulation sim;
  ASSERT_TRUE(Load(sim));
  const auto handle = sim.FindSensor<Gyroscope>({"gyro", 0});
  Steps(sim, 1);
  const auto first = sim.Read(handle);
  Steps(sim, 4);
  EXPECT_EQ(sim.Read(handle).sequence, first.sequence);
  EXPECT_EQ(sim.Read(handle).sample_time, first.sample_time);
  Steps(sim, 1);
  EXPECT_EQ(sim.Read(handle).sequence, first.sequence + 1);
}

TEST_F(Arms, SetSensorConfigReplacesASensorsSettings) {
  Write({"bias: [0, 0, 0.5]"});
  Simulation sim;
  ASSERT_TRUE(Load(sim));
  const auto handle = sim.FindSensor<Gyroscope>({"gyro", 1});
  Steps(sim, 2);
  EXPECT_NEAR(sim.Read(handle).value.Z(), 1.5F, 1e-4F);

  sim.SetSensorConfig(handle, Gyroscope(SensorOptions{}, Gyroscope::Params{}));
  Steps(sim, 2);
  EXPECT_NEAR(sim.Read(handle).value.Z(), 1.0F, 1e-4F);
  // Only that instance.
  EXPECT_NEAR(
      sim.Read(sim.FindSensor<Gyroscope>({"gyro", 0})).value.Z(), 1.5F, 1e-4F
  );
}

TEST_F(Arms, InitRemembersItsSeed) {
  Write();
  Simulation seeded;
  ASSERT_TRUE(Load(seeded, 42));
  EXPECT_EQ(seeded.Seed(), 42U);
}

TEST_F(Arms, TheSameSeedGivesTheSameNoise) {
  Write({"noise_density: 0.01"});
  Simulation first;
  Simulation again;
  ASSERT_TRUE(Load(first, 3));
  ASSERT_TRUE(Load(again, 3));
  Steps(first, 3);
  Steps(again, 3);
  const auto a = first.Read(first.FindSensor<Gyroscope>({"gyro", 2})).value;
  const auto b = again.Read(again.FindSensor<Gyroscope>({"gyro", 2})).value;
  EXPECT_EQ(a.X(), b.X());
  EXPECT_NE(a.X(), 0.0F);
  // Each instance has its own stream.
  const auto other = first.Read(first.FindSensor<Gyroscope>({"gyro", 0})).value;
  EXPECT_NE(a.X(), other.X());
}

// ---------------------------------------------------------------------------
// Bad sensor files
// ---------------------------------------------------------------------------

TEST_F(Arms, AnUnknownSettingFailsInit) {
  Write({"perod: 0.1"});
  Simulation sim;
  EXPECT_FALSE(Load(sim));
}

TEST_F(Arms, AnUnknownKindFailsInit) {
  Write({}, {"odd.arow"});
  dir_.Write("odd.arow", SensorFile("odd", "no_such_kind", "arm", "link"));
  Simulation sim;
  EXPECT_FALSE(Load(sim));
}

TEST_F(Arms, TwoSensorsWithOneNameFailToLoad) {
  Write({}, {"twin.arow"});
  dir_.Write("twin.arow", SensorFile("gyro", "gyroscope", "arm", "link"));
  Simulation sim;
  EXPECT_FALSE(sim.LoadArowFile((dir_.Path() / "world.arow").string()));
}

TEST_F(Arms, ASensorOnAnUnknownJointFails) {
  Write({}, {"lost.arow"});
  dir_.Write("lost.arow", SensorFile("lost", "gyroscope", "arm", "elbow"));
  Simulation sim;
  EXPECT_FALSE(Load(sim));
}
