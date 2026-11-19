// Reading and writing the running sim's own state from outside it
// (interface::StateHandle, Simulation::State): every listed quantity read,
// only those listed ReadWrite written.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "domain/attachment/sim_quantities.hpp"
#include "domain/layout/joint_records.hpp"
#include "domain/math/scalar.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"
#include "interface/simulation.hpp"
#include "interface/state_handle.hpp"
#include "interface/state_quantities.hpp"
#include "interface/writable_handle.hpp"
#include "support/sensing_scene.hpp"
#include "support/temp_dir.hpp"

using achilles::domain::layout::JointAddress;
using achilles::domain::math::Vector3;
using achilles::interface::Simulation;
using achilles::interface::StateHandle;
using achilles::interface::WritableHandle;
using achilles::test_support::Archetype;
using achilles::test_support::kZUpConfig;
using achilles::test_support::SceneJoint;
using achilles::test_support::TempDir;
namespace input = achilles::interface::input;

// What's meant to be written, and what only to be read.
static_assert(achilles::interface::kWritableV<input::JointPosition>);
static_assert(achilles::interface::kWritableV<input::JointVelocity>);
static_assert(achilles::interface::kWritableV<input::VisualColor>);
static_assert(achilles::interface::kReadableV<input::WorldTransform>);
static_assert(!achilles::interface::kWritableV<input::WorldTransform>);

// A const handle only reads; a writable one makes one, explicitly, never
// back.
static_assert(WritableHandle<StateHandle<input::JointPosition>>);
static_assert(!WritableHandle<StateHandle<const input::JointPosition>>);
static_assert(std::is_constructible_v<
              StateHandle<const input::JointPosition>,
              StateHandle<input::JointPosition>>);
static_assert(!std::is_convertible_v<
              StateHandle<input::JointPosition>,
              StateHandle<const input::JointPosition>>);
static_assert(!std::is_constructible_v<
              StateHandle<input::JointPosition>,
              StateHandle<const input::JointPosition>>);

namespace {

constexpr float kDt = 0.002F;

// A welded world with two copies of a one-joint arm, each spinning about
// its roll joint at 1 rad/s.
class Arms : public ::testing::Test {
 protected:
  void SetUp() override {
    SceneJoint world{.name = "world"};
    dir_.Write("world.arow", Archetype("world", {world}, {"arm.arow"}));
    dir_.Write(
        "arm.arow",
        "copies: 2\n" +
            Archetype(
                "arm",
                {SceneJoint{
                    .name = "link", .dof = 0, .velocity = {1.0F, 0, 0, 0, 0, 0}
                }},
                {},
                "world",
                "world"
            )
    );
    dir_.Write("config.yaml", kZUpConfig);
    ASSERT_TRUE(sim_.LoadArowFile((dir_.Path() / "world.arow").string()));
    ASSERT_TRUE(sim_.LoadConfigFile((dir_.Path() / "config.yaml").string()));
    ASSERT_TRUE(sim_.Init(1));
  }

  void Steps(int count) {
    for (int i = 0; i < count; ++i) {
      ASSERT_TRUE(sim_.Step(kDt));
    }
  }

  // How far copy `copy`'s link has rolled.
  float Roll(std::size_t copy) const {
    return sim_.State<const input::JointPosition>(Link(copy))
        .Read()
        .Rotation()
        .AngleAbout(Vector3<float>::UnitX());
  }

  static JointAddress Link(std::size_t copy) {
    return {.archetype = "arm", .joint = "link", .instance = copy};
  }

  TempDir dir_;
  Simulation sim_;
};

}  // namespace

TEST_F(Arms, ReadsEachJointWhereTheSimHasMovedIt) {
  Steps(10);
  for (std::size_t copy = 0; copy < 2; ++copy) {
    EXPECT_NEAR(Roll(copy), 10 * kDt, 1e-4F) << "copy " << copy;
  }
}

TEST_F(Arms, AWrittenVelocityStaysUntilTheSimMovesItOn) {
  Steps(5);
  // Stop copy 1; copy 0 keeps spinning.
  sim_.State<input::JointVelocity>(Link(1)).Write({});
  const float stopped_at = Roll(1);
  Steps(20);
  EXPECT_NEAR(Roll(1), stopped_at, 1e-6F);
  EXPECT_NEAR(Roll(0), 25 * kDt, 1e-4F);
}

TEST_F(Arms, AWrittenPositionIsWhereTheSimGoesOnFrom) {
  const auto link = sim_.State<input::JointPosition>(Link(0));
  auto pose = link.Read();
  pose = achilles::domain::spatial::Transform<float>(
      0, 0, 0, std::cos(0.25F), std::sin(0.25F), 0, 0  // 0.5 rad about x
  );
  link.Write(pose);
  // Read straight back, before any step.
  EXPECT_NEAR(Roll(0), 0.5F, 1e-5F);
  Steps(10);
  EXPECT_NEAR(Roll(0), 0.5F + (10 * kDt), 1e-4F);
  // And the body's world pose, worked out from it, follows on the step --
  // from where the step began, so one tick behind the joint it moved on.
  const float world_roll = sim_.State<const input::WorldTransform>(Link(0))
                               .Read()
                               .Rotation()
                               .AngleAbout(Vector3<float>::UnitX());
  EXPECT_NEAR(world_roll, Roll(0) - kDt, 1e-4F);
}

TEST_F(Arms, AReadOnlyHandleSeesWhatTheWritableOneWrites) {
  const StateHandle<input::VisualColor> color =
      sim_.State<input::VisualColor>(Link(1));
  const StateHandle<const input::VisualColor> looking(color);
  color.Write(Vector3<float>(0.25F, 0.5F, 0.75F));
  EXPECT_FLOAT_EQ(looking.Read().Y(), 0.5F);
}

TEST_F(Arms, SaysWhatWasWrong) {
  EXPECT_THROW(
      (void)sim_.State<const input::JointPosition>(
          {.archetype = "arm", .joint = "nope"}
      ),
      std::invalid_argument
  );
  EXPECT_THROW(
      (void)sim_.State<const input::JointPosition>(
          {.archetype = "arm", .joint = "link", .instance = 2}
      ),
      std::invalid_argument
  );
}

TEST_F(Arms, JointsListsEveryJointsAddress) {
  std::vector<std::string> listed;
  for (const JointAddress& joint : sim_.Joints()) {
    listed.push_back(
        joint.archetype + "." + joint.joint + " #" +
        std::to_string(joint.instance)
    );
  }
  EXPECT_EQ(
      listed,
      (std::vector<std::string>{"world.world #0", "arm.link #0", "arm.link #1"})
  );
}

TEST(StateHandleBeforeInit, Throws) {
  Simulation sim;
  EXPECT_THROW(
      (void)sim.State<const input::JointPosition>(
          {.archetype = "arm", .joint = "link"}
      ),
      std::runtime_error
  );
}

// Every quantity on the list is readable; exactly the ones the sim keeps
// from tick to tick are writable.
static_assert(achilles::interface::kReadableV<input::JointAcceleration>);
static_assert(achilles::interface::kReadableV<input::JointSubspace>);
static_assert(achilles::interface::kReadableV<input::SpatialVelocity>);
static_assert(achilles::interface::kReadableV<input::SpatialAcceleration>);
static_assert(achilles::interface::kReadableV<input::CollisionPose>);
static_assert(achilles::interface::kReadableV<input::CollisionExtents>);
static_assert(achilles::interface::kReadableV<input::CollisionRadius>);
static_assert(achilles::interface::kWritableV<input::VisualPose>);
static_assert(achilles::interface::kWritableV<input::VisualExtents>);
static_assert(achilles::interface::kWritableV<input::VisualShape>);
static_assert(!achilles::interface::kWritableV<input::JointAcceleration>);
static_assert(!achilles::interface::kWritableV<input::JointSubspace>);
static_assert(!achilles::interface::kWritableV<input::SpatialVelocity>);
static_assert(!achilles::interface::kWritableV<input::SpatialAcceleration>);
static_assert(!achilles::interface::kWritableV<input::CollisionPose>);
static_assert(!achilles::interface::kWritableV<input::CollisionExtents>);
static_assert(!achilles::interface::kWritableV<input::CollisionRadius>);

namespace {

// Something not on the list at all: neither read nor written.
struct NotAQuantity {
  using Value = float;
};
static_assert(!achilles::interface::kReadableV<NotAQuantity>);
static_assert(!achilles::interface::kWritableV<NotAQuantity>);

// One welded body with a collision box and ball of its own, to read back.
class Shapes : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_.Write(
        "shape.arow",
        Archetype(
            "shape",
            {SceneJoint{
                .name = "body",
                .dof = 0,
                .extents = {0.1F, 0.2F, 0.3F},
                .radius = 0.05F,
                .group = 1,
                .mask = 1
            }}
        )
    );
    dir_.Write("config.yaml", kZUpConfig);
    ASSERT_TRUE(sim_.LoadArowFile((dir_.Path() / "shape.arow").string()));
    ASSERT_TRUE(sim_.LoadConfigFile((dir_.Path() / "config.yaml").string()));
    ASSERT_TRUE(sim_.Init(1));
  }

  static JointAddress Body() { return {.archetype = "shape", .joint = "body"}; }

  TempDir dir_;
  Simulation sim_;
};

}  // namespace

// Every listed quantity reads back what the scene gave it, or what the sim
// worked out from that.
TEST_F(Shapes, EveryListedQuantityReads) {
  ASSERT_TRUE(sim_.Step(kDt));
  const auto extents = sim_.State<const input::CollisionExtents>(Body()).Read();
  EXPECT_FLOAT_EQ(extents.X(), 0.1F);
  EXPECT_FLOAT_EQ(extents.Y(), 0.2F);
  EXPECT_FLOAT_EQ(extents.Z(), 0.3F);
  EXPECT_FLOAT_EQ(
      sim_.State<const input::CollisionRadius>(Body()).Read().Value(), 0.05F
  );

  // Its one freedom: one column of its subspace, with a single 1 in it.
  const auto subspace = sim_.State<const input::JointSubspace>(Body()).Read();
  int ones = 0;
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t col = 0; col < 6; ++col) {
      ones += subspace(row, col) == 1.0F ? 1 : 0;
    }
  }
  EXPECT_EQ(ones, 1);

  // At rest at the origin, unturned: where it is, how it moves and how its
  // collision shape sits are all the identity, or nothing.
  const auto world = sim_.State<const input::WorldTransform>(Body()).Read();
  EXPECT_NEAR(world.Translation().Norm(), 0.0F, 1e-6F);
  EXPECT_NEAR(world.Rotation().W(), 1.0F, 1e-6F);
  const auto pose = sim_.State<const input::CollisionPose>(Body()).Read();
  EXPECT_NEAR(pose.Rotation().W(), 1.0F, 1e-6F);
  // Readable whatever their values: the sim's own accelerations and
  // velocities.
  (void)sim_.State<const input::JointAcceleration>(Body()).Read();
  (void)sim_.State<const input::SpatialVelocity>(Body()).Read();
  (void)sim_.State<const input::SpatialAcceleration>(Body()).Read();
}

// Each writable visual quantity reads back exactly what was written.
TEST_F(Shapes, EachWritableVisualQuantityReadsBackWhatWasWritten) {
  const auto extents = sim_.State<input::VisualExtents>(Body());
  extents.Write(Vector3<float>(0.4F, 0.5F, 0.6F));
  EXPECT_FLOAT_EQ(extents.Read().Y(), 0.5F);

  const auto shape = sim_.State<input::VisualShape>(Body());
  shape.Write(achilles::domain::math::Scalar<float>(2.0F));
  EXPECT_FLOAT_EQ(shape.Read().Value(), 2.0F);

  const auto pose = sim_.State<input::VisualPose>(Body());
  pose.Write(achilles::domain::spatial::Transform<float>(1, 2, 3, 1, 0, 0, 0));
  EXPECT_FLOAT_EQ(pose.Read().Translation().Z(), 3.0F);

  // And the sim keeps them through a step: nothing works them out again.
  ASSERT_TRUE(sim_.Step(kDt));
  EXPECT_FLOAT_EQ(extents.Read().X(), 0.4F);
  EXPECT_FLOAT_EQ(shape.Read().Value(), 2.0F);
  EXPECT_FLOAT_EQ(pose.Read().Translation().X(), 1.0F);
}
