#pragma once

#include <string>

#include "interface/simulation.hpp"
#include "sensing_scene.hpp"
#include "temp_dir.hpp"

namespace achilles::test_support {

// One body (1 kg, 0.1 kg m^2 about each axis) on a joint that only yaws --
// a motor's shaft, for the actuator model tests -- with a joint_velocity
// truth sensor and a yaw actuator (shaft_motor) of `kind` on it, loaded and
// initialized. `settings` is the rest of its config: block, each line indented
// and newline-terminated.
inline bool LoadShaft(
    const TempDir& dir,
    interface::Simulation& sim,
    const std::string& kind,
    const std::string& settings
) {
  dir.Write(
      "shaft_rate.arow",
      SensorFile("shaft_rate", "joint_velocity", "rig", "shaft")
  );
  dir.Write(
      "shaft_motor.arow",
      "actuator: shaft_motor\nkind: " + kind +
          "\nattach: {archetype: rig, joint: shaft}\n"
          "config:\n  axis: [0, 0, 1]\n" +
          settings
  );
  const auto file = dir.Write(
      "rig.arow",
      Archetype(
          "rig",
          {SceneJoint{.name = "shaft", .dof = 2}},
          {"shaft_rate.arow", "shaft_motor.arow"}
      )
  );
  return sim.LoadArowFile(file.string()) && sim.Init(1);
}

// The shaft's rate, from its truth sensor.
inline float ShaftRate(const interface::Simulation& sim) {
  return sim.Sensor<interface::truth::JointVelocity>("shaft_rate")
      .Read()
      .value[0];
}

}  // namespace achilles::test_support
