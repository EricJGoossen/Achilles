// The maze demo's scene (examples/maze): the robot loads with all six of
// its sensors, and its lidar, at the start, sees the start cell's walls
// where maze.arow's drawing puts them.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <numbers>
#include <string>

#include "algorithms/sensing/encoder.hpp"
#include "algorithms/sensing/gyroscope.hpp"
#include "algorithms/sensing/lidar.hpp"
#include "interface/simulation.hpp"

using achilles::algorithms::sensing::Encoder;
using achilles::algorithms::sensing::Gyroscope;
using achilles::algorithms::sensing::Lidar;
using achilles::interface::Simulation;

namespace {

constexpr float kDt = 0.004F;
constexpr float kPi = std::numbers::pi_v<float>;
constexpr std::size_t kBeams = 360;  // sensors/lidar.arow

std::filesystem::path ExampleDir() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "examples" / "maze";
}

class Maze : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(sim_.LoadArowFile((ExampleDir() / "maze.arow").string()));
    ASSERT_TRUE(sim_.LoadConfigFile((ExampleDir() / "sim_config.yaml").string())
    );
    ASSERT_TRUE(sim_.Init(1));
    sim_.SetIntegrator(Simulation::Integrator::kSemiImplicitEuler);
    ASSERT_TRUE(sim_.Step(kDt));
  }

  // The range of the beam nearest `azimuth` (radians, counterclockwise
  // from the robot's forward).
  float RangeAt(float azimuth) const {
    const auto scan = sim_.Read(sim_.FindSensor<Lidar>({"lidar", 0})).value;
    const float step = 2.0F * kPi / kBeams;
    const auto beam =
        static_cast<std::size_t>(std::lround(((azimuth + kPi) / step) - 0.5F)) %
        kBeams;
    return scan.ranges[beam];
  }

  Simulation sim_;
};

}  // namespace

TEST_F(Maze, LoadsTheRobotsSensors) {
  EXPECT_NO_THROW((void)sim_.FindSensor<Lidar>({"lidar", 0}));
  EXPECT_NO_THROW((void)sim_.FindSensor<Gyroscope>({"gyroscope", 0}));
  for (const char* wheel :
       {"left_front", "left_rear", "right_front", "right_rear"}) {
    EXPECT_NO_THROW((void
    )sim_.FindSensor<Encoder>({std::string(wheel) + "_encoder", 0}));
  }
}

// The start cell, centered on (-3.75, -3.75) and facing +y, is walled in
// on three sides 0.75 m from its center -- less a wall's 0.05 m half
// thickness -- and open to the east for two cells.
TEST_F(Maze, TheLidarSeesTheStartCellsWalls) {
  constexpr float kTolerance = 0.03F;  // range noise, resolution, settling
  EXPECT_NEAR(RangeAt(0.0F), 0.70F, kTolerance);          // ahead, +y
  EXPECT_NEAR(RangeAt(kPi / 2), 0.70F, kTolerance);       // left, -x
  EXPECT_NEAR(RangeAt(-kPi + 0.01F), 0.70F, kTolerance);  // behind, -y
  EXPECT_NEAR(RangeAt(-kPi / 2), 2.20F, kTolerance);      // right, +x
}
