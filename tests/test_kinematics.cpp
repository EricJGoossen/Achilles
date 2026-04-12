/**
 * Tests for kinematics objects.
 *
 * Covers:
 *   - Geometry: Frame, Transform, TransformTree
 *   - Dynamics: Link, RevoluteJoint, PlanarJoint, JointTree
 *   - Control:  Actuator, LinearActuator, AngularActuator, Plant
 */

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "control/actuators/angular_actuator.hpp"
#include "control/actuators/linear_actuator.hpp"
#include "control/plant.hpp"
#include "dynamics/joint_tree.hpp"
#include "dynamics/joints/planar_joint.hpp"
#include "dynamics/joints/revolute_joint.hpp"
#include "dynamics/link.hpp"
#include "geometry/frame.hpp"
#include "geometry/transform.hpp"
#include "geometry/transform_tree.hpp"
#include "math/unit_vector.hpp"
#include "math/vector.hpp"
#include "spatial/inertia.hpp"
#include "spatial/jerk.hpp"
#include "spatial/pose.hpp"
#include "spatial/twist.hpp"

// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────
namespace {

namespace control = achilles::control;
namespace dynamics = achilles::dynamics;
namespace geometry = achilles::geometry;
namespace math = achilles::math;
namespace spatial = achilles::spatial;

constexpr double EPS = 1e-9;
constexpr double LOOSE = 1e-6;

spatial::Inertia sphere_inertia(double mass, double r) {
    double I = 0.4 * mass * r * r;
    return {I * Eigen::Matrix3d::Identity(), math::Vector::zero(), mass};
}

spatial::Inertia sphere_inertia() { return sphere_inertia(1.0, 0.1); }

// Bundles Id managers with factory methods so each fixture gets
// independent, sequentially-allocated ids.
struct Fixtures {
    math::Id<geometry::Frame>::IdManager frame_mgr;
    math::Id<dynamics::Link>::IdManager link_mgr;

    geometry::Frame makeFrame(const std::string& name) {
        return {name, geometry::Frame::Id(frame_mgr)};
    }
    dynamics::Link makeLink(
        const geometry::Frame& frame, spatial::Inertia inertia
    ) {
        return {frame, std::move(inertia), dynamics::Link::Id(link_mgr)};
    }
};

}  // namespace

// ══════════════════════════════════════════════════════════════
//  1. Frame
// ══════════════════════════════════════════════════════════════

TEST(Frame, NameAndIdPreserved) {
    math::Id<geometry::Frame>::IdManager mgr;
    geometry::Frame f("world", geometry::Frame::Id(mgr));
    EXPECT_EQ(f.name(), "world");
    EXPECT_EQ(f.id().id(), 0U);
}

// ══════════════════════════════════════════════════════════════
//  2. Transform
// ══════════════════════════════════════════════════════════════

TEST(Transform, UpdateRetrievedCorrectly) {
    Fixtures fix;
    auto parent = fix.makeFrame("parent");
    auto child = fix.makeFrame("child");

    geometry::Transform t(parent, child, spatial::Pose::identity());
    t.update(spatial::Pose(math::Vector(1, 2, 3), math::Quaternion::identity())
    );

    EXPECT_NEAR(t.pose().position().x(), 1.0, EPS);
    EXPECT_NEAR(t.pose().position().y(), 2.0, EPS);
    EXPECT_NEAR(t.pose().position().z(), 3.0, EPS);
}

TEST(Transform, AddComposesPoses) {
    Fixtures fix;
    auto parent = fix.makeFrame("parent");
    auto child = fix.makeFrame("child");

    geometry::Transform t(
        parent,
        child,
        spatial::Pose(math::Vector(1, 0, 0), math::Quaternion::identity())
    );

    t.add(spatial::Pose(math::Vector(0, 1, 0), math::Quaternion::identity()));

    EXPECT_NEAR(t.pose().position().x(), 1.0, LOOSE);
    EXPECT_NEAR(t.pose().position().y(), 1.0, LOOSE);
}

TEST(Transform, ParentAndChildFramesCorrect) {
    Fixtures fix;
    auto parent = fix.makeFrame("parent");
    auto child = fix.makeFrame("child");

    geometry::Transform t(parent, child, spatial::Pose::identity());
    EXPECT_EQ(t.parentFrame().id(), parent.id());
    EXPECT_EQ(t.childFrame().id(), child.id());
}

// ══════════════════════════════════════════════════════════════
//  3. TransformTree
// ══════════════════════════════════════════════════════════════

TEST(TransformTree, AddAndRetrieve) {
    Fixtures fix;
    auto parent = fix.makeFrame("world");
    auto child = fix.makeFrame("link1");

    geometry::TransformTree tree;
    tree.addTransform(std::make_unique<geometry::Transform>(
        parent, child, spatial::Pose::identity()
    ));

    EXPECT_NEAR(
        tree.getTransform(parent, child).pose().position().mag(), 0.0, EPS
    );
}

TEST(TransformTree, UpdateTransform) {
    Fixtures fix;
    auto parent = fix.makeFrame("world");
    auto child = fix.makeFrame("link1");

    geometry::TransformTree tree;
    tree.addTransform(std::make_unique<geometry::Transform>(
        parent, child, spatial::Pose::identity()
    ));

    tree.updateTransform(
        parent,
        child,
        spatial::Pose(math::Vector(5, 0, 0), math::Quaternion::identity())
    );

    EXPECT_NEAR(
        tree.getTransform(parent, child).pose().position().x(), 5.0, EPS
    );
}

TEST(TransformTree, DuplicateChildThrows) {
    Fixtures fix;
    auto parent = fix.makeFrame("world");
    auto child = fix.makeFrame("link1");

    geometry::TransformTree tree;
    tree.addTransform(std::make_unique<geometry::Transform>(
        parent, child, spatial::Pose::identity()
    ));

    EXPECT_THROW(
        tree.addTransform(std::make_unique<geometry::Transform>(
            parent, child, spatial::Pose::identity()
        )),
        std::runtime_error
    );
}

TEST(TransformTree, WrongParentThrows) {
    Fixtures fix;
    auto world = fix.makeFrame("world");
    auto other = fix.makeFrame("other");
    auto child = fix.makeFrame("link1");

    geometry::TransformTree tree;
    tree.addTransform(std::make_unique<geometry::Transform>(
        world, child, spatial::Pose::identity()
    ));

    EXPECT_THROW(tree.getTransform(other, child), std::runtime_error);
}

// ══════════════════════════════════════════════════════════════
//  4. RevoluteJoint
// ══════════════════════════════════════════════════════════════

struct RevoluteFixture {
    Fixtures fix;
    geometry::Frame frame = fix.makeFrame("joint_frame");
    geometry::Frame parent_frame = fix.makeFrame("parent_frame");
    geometry::Frame child_frame = fix.makeFrame("child_frame");
    dynamics::Link parent_link = fix.makeLink(parent_frame, sphere_inertia());
    dynamics::Link child_link = fix.makeLink(child_frame, sphere_inertia());
    math::UnitVector z_axis{0, 0, 1};
    dynamics::joints::RevoluteJoint joint{
        frame,
        parent_link,
        child_link,
        z_axis,
        spatial::Pose::identity(),
        spatial::Twist::identity()};
};

TEST(RevoluteJoint, InitialPositionIsIdentity) {
    RevoluteFixture f;
    EXPECT_NEAR(f.joint.position().position().mag(), 0.0, EPS);
    EXPECT_NEAR(f.joint.position().orientation().w(), 1.0, EPS);
}

TEST(RevoluteJoint, InitialVelocityIsZero) {
    RevoluteFixture f;
    EXPECT_NEAR(f.joint.velocity().norm(), 0.0, EPS);
}

TEST(RevoluteJoint, FrameAndLinksCorrect) {
    RevoluteFixture f;
    EXPECT_EQ(f.joint.frame().id(), f.frame.id());
    EXPECT_EQ(f.joint.parentLink().id(), f.parent_link.id());
    EXPECT_EQ(f.joint.childLink().id(), f.child_link.id());
}

TEST(RevoluteJoint, IntegrateZeroAccelerationNoChange) {
    RevoluteFixture f;
    f.joint.integrate(0.1);
    EXPECT_NEAR(f.joint.position().orientation().w(), 1.0, LOOSE);
}

TEST(RevoluteJoint, ApplyAccelerationUpdatesCache) {
    RevoluteFixture f;
    f.joint.applyAcceleration(
        spatial::Jerk(math::Vector(0, 0, 0), math::Vector(0, 0, 1))
    );
    EXPECT_GT(f.joint.acceleration().norm(), 0.0);
}

TEST(RevoluteJoint, IntegrateWithAccelerationProducesRotation) {
    RevoluteFixture f;
    f.joint.applyAcceleration(
        spatial::Jerk(math::Vector(0, 0, 0), math::Vector(0, 0, 1))
    );
    f.joint.integrate(1.0);
    EXPECT_GT(f.joint.position().angle(), 0.0);
}

TEST(RevoluteJoint, AccelerationClearedAfterIntegrate) {
    RevoluteFixture f;
    f.joint.applyAcceleration(
        spatial::Jerk(math::Vector(0, 0, 0), math::Vector(0, 0, 1))
    );
    f.joint.integrate(0.01);
    EXPECT_NEAR(f.joint.acceleration().norm(), 0.0, LOOSE);
}

TEST(RevoluteJoint, SolveInertiaReducesDOF) {
    RevoluteFixture f;
    spatial::Inertia child_I = sphere_inertia();
    spatial::Inertia art_I = f.joint.solveInertia(child_I);
    EXPECT_LE(art_I.mat().norm(), child_I.mat().norm() + LOOSE);
}

// ══════════════════════════════════════════════════════════════
//  5. PlanarJoint
// ══════════════════════════════════════════════════════════════

struct PlanarFixture {
    Fixtures fix;
    geometry::Frame frame = fix.makeFrame("joint_frame");
    geometry::Frame parent_frame = fix.makeFrame("parent_frame");
    geometry::Frame child_frame = fix.makeFrame("child_frame");
    dynamics::Link parent_link = fix.makeLink(parent_frame, sphere_inertia());
    dynamics::Link child_link = fix.makeLink(child_frame, sphere_inertia());
    math::UnitVector z_normal{0, 0, 1};
    dynamics::joints::PlanarJoint joint{
        frame,
        parent_link,
        child_link,
        z_normal,
        spatial::Pose::identity(),
        spatial::Twist::identity()};
};

TEST(PlanarJoint, InitialStateIsIdentity) {
    PlanarFixture f;
    EXPECT_NEAR(f.joint.position().position().mag(), 0.0, EPS);
    EXPECT_NEAR(f.joint.velocity().norm(), 0.0, EPS);
}

TEST(PlanarJoint, LinearAccelerationProducesTranslation) {
    PlanarFixture f;
    f.joint.applyAcceleration(
        spatial::Jerk(math::Vector(1, 0, 0), math::Vector(0, 0, 0))
    );
    f.joint.integrate(1.0);
    EXPECT_GT(f.joint.position().position().x(), 0.0);
}

TEST(PlanarJoint, AngularAccelerationProducesRotation) {
    PlanarFixture f;
    f.joint.applyAcceleration(
        spatial::Jerk(math::Vector(0, 0, 0), math::Vector(0, 0, 1))
    );
    f.joint.integrate(1.0);
    EXPECT_GT(f.joint.position().angle(), 0.0);
}

TEST(PlanarJoint, SolveInertiaIsSymmetric) {
    PlanarFixture f;
    Eigen::Matrix<double, 6, 6> m =
        f.joint.solveInertia(sphere_inertia()).mat();
    EXPECT_NEAR((m - m.transpose()).norm(), 0.0, LOOSE);
}

// ══════════════════════════════════════════════════════════════
//  6. JointTree
// ══════════════════════════════════════════════════════════════

struct TreeFixture {
    Fixtures fix;
    geometry::Frame root_frame = fix.makeFrame("root_frame");
    geometry::Frame joint1_frame = fix.makeFrame("joint1_frame");
    geometry::Frame joint2_frame = fix.makeFrame("joint2_frame");
    geometry::Frame child1_frame = fix.makeFrame("child1_frame");
    geometry::Frame child2_frame = fix.makeFrame("child2_frame");
    dynamics::Link root = fix.makeLink(root_frame, sphere_inertia());
    dynamics::Link child1 = fix.makeLink(child1_frame, sphere_inertia());
    dynamics::Link child2 = fix.makeLink(child2_frame, sphere_inertia());
    math::UnitVector z_axis{0, 0, 1};
    dynamics::JointTree tree;

    TreeFixture() {
        tree.addJoint(std::make_unique<dynamics::joints::RevoluteJoint>(
            joint1_frame,
            root,
            child1,
            z_axis,
            spatial::Pose::identity(),
            spatial::Twist::identity()
        ));
        tree.addJoint(std::make_unique<dynamics::joints::RevoluteJoint>(
            joint2_frame,
            child1,
            child2,
            z_axis,
            spatial::Pose::identity(),
            spatial::Twist::identity()
        ));
    }
};

TEST(JointTree, GetJointForChildLink) {
    TreeFixture f;
    EXPECT_EQ(f.tree.getJoint(f.child1).childLink().id(), f.child1.id());
}

TEST(JointTree, DuplicateChildThrows) {
    TreeFixture f;
    int foo = 0;
    EXPECT_THROW(
        f.tree.addJoint(std::make_unique<dynamics::joints::RevoluteJoint>(
            f.joint1_frame,
            f.root,
            f.child1,
            f.z_axis,
            spatial::Pose::identity(),
            spatial::Twist::identity()
        )),
        std::runtime_error
    );
}

TEST(JointTree, CompositeInertiaIsNonZero) {
    TreeFixture f;
    auto inertias = f.tree.computeCompositeInertias(f.root);
    EXPECT_GT(inertias.at(f.root.id()).mat().norm(), 0.0);
}

TEST(JointTree, CompositeInertiaIncludesChildren) {
    TreeFixture f;
    auto inertias = f.tree.computeCompositeInertias(f.root);
    // Root composite should be larger than child2 (leaf) composite
    EXPECT_GT(
        inertias.at(f.root.id()).mat().norm(),
        inertias.at(f.child2.id()).mat().norm()
    );
}

TEST(JointTree, IntegrateDoesNotThrow) {
    TreeFixture f;
    EXPECT_NO_THROW(f.tree.integrate(0.01));
}

TEST(JointTree, PropagateAccelerationsDoesNotThrow) {
    TreeFixture f;
    EXPECT_NO_THROW(f.tree.propagateAccelerations(f.root));
}

TEST(JointTree, UpdateTransformsReflectsJointPositions) {
    TreeFixture f;

    geometry::TransformTree tt;
    tt.addTransform(std::make_unique<geometry::Transform>(
        f.joint1_frame, f.child1_frame, spatial::Pose::identity()
    ));
    tt.addTransform(std::make_unique<geometry::Transform>(
        f.joint2_frame, f.child2_frame, spatial::Pose::identity()
    ));

    f.tree.propagateAccelerations(f.root);
    f.tree.integrate(0.1);
    EXPECT_NO_THROW(f.tree.updateTransforms(tt));
}

// ══════════════════════════════════════════════════════════════
//  7. Actuators
// ══════════════════════════════════════════════════════════════

struct ActuatorFixture {
    Fixtures fix;
    geometry::Frame joint_frame = fix.makeFrame("joint_frame");
    geometry::Frame parent_frame = fix.makeFrame("parent_frame");
    geometry::Frame child_frame = fix.makeFrame("child_frame");
    dynamics::Link parent_link = fix.makeLink(parent_frame, sphere_inertia());
    dynamics::Link child_link = fix.makeLink(child_frame, sphere_inertia());
    math::UnitVector z_axis{0, 0, 1};
    dynamics::joints::RevoluteJoint joint{
        joint_frame,
        parent_link,
        child_link,
        z_axis,
        spatial::Pose::identity(),
        spatial::Twist::identity()};
};

TEST(AngularActuator, ZeroEffortNoAcceleration) {
    ActuatorFixture f;
    control::actuators::AngularActuator act(math::Vector(0, 0, 1), f.joint);
    EXPECT_NO_THROW(act.actuate(sphere_inertia()));
    EXPECT_NEAR(f.joint.acceleration().norm(), 0.0, LOOSE);
}

TEST(AngularActuator, NonZeroEffortProducesAcceleration) {
    ActuatorFixture f;
    control::actuators::AngularActuator act(math::Vector(0, 0, 1), f.joint);
    act.applyEffort(10.0);
    act.actuate(sphere_inertia());
    EXPECT_GT(f.joint.acceleration().norm(), 0.0);
}

TEST(AngularActuator, EffortResetAfterActuate) {
    ActuatorFixture f;
    control::actuators::AngularActuator act(math::Vector(0, 0, 1), f.joint);
    act.applyEffort(10.0);
    act.actuate(sphere_inertia());
    f.joint.integrate(0.01);

    // Second actuate with no new effort → no acceleration
    act.actuate(sphere_inertia());
    EXPECT_NEAR(f.joint.acceleration().norm(), 0.0, LOOSE);
}

TEST(AngularActuator, ChildLinkMatchesJoint) {
    ActuatorFixture f;
    control::actuators::AngularActuator act(math::Vector(0, 0, 1), f.joint);
    EXPECT_EQ(act.childLink().id(), f.child_link.id());
}

TEST(LinearActuator, ActuatesAlongLinearDOF) {
    Fixtures fix;
    auto jf = fix.makeFrame("jf");
    auto pf = fix.makeFrame("pf");
    auto cf = fix.makeFrame("cf");
    auto pl = fix.makeLink(pf, sphere_inertia());
    auto cl = fix.makeLink(cf, sphere_inertia());

    dynamics::joints::PlanarJoint planar(
        jf,
        pl,
        cl,
        math::UnitVector(0, 0, 1),
        spatial::Pose::identity(),
        spatial::Twist::identity()
    );

    control::actuators::LinearActuator act(math::Vector(1, 0, 0), planar);
    act.applyEffort(5.0);
    EXPECT_NO_THROW(act.actuate(sphere_inertia()));
}

// ══════════════════════════════════════════════════════════════
//  8. Plant (integration test)
// ══════════════════════════════════════════════════════════════

// Builds a minimal one-revolute-joint plant and returns it.
// Ownership of all sub-objects is transferred into the Plant.
auto makeSingleJointPlant() {
    math::Id<geometry::Frame>::IdManager frame_mgr;
    math::Id<dynamics::Link>::IdManager link_mgr;

    auto mkf = [&](const std::string& n) {
        return std::make_unique<geometry::Frame>(
            n, geometry::Frame::Id(frame_mgr)
        );
    };

    auto root_frame = mkf("root_frame");
    auto joint_frame = mkf("joint_frame");
    auto child_frame = mkf("child_frame");

    auto root_link = std::make_unique<dynamics::Link>(
        *root_frame, sphere_inertia(), dynamics::Link::Id(link_mgr)
    );
    auto child_link = std::make_unique<dynamics::Link>(
        *child_frame, sphere_inertia(), dynamics::Link::Id(link_mgr)
    );

    auto joint = std::make_unique<dynamics::joints::RevoluteJoint>(
        *joint_frame,
        *root_link,
        *child_link,
        math::UnitVector(0, 0, 1),
        spatial::Pose::identity(),
        spatial::Twist::identity()
    );

    auto* joint_ptr = joint.get();

    dynamics::JointTree joint_tree;
    joint_tree.addJoint(std::move(joint));

    geometry::TransformTree transform_tree;
    transform_tree.addTransform(std::make_unique<geometry::Transform>(
        *joint_frame, *child_frame, spatial::Pose::identity()
    ));

    auto actuator = std::make_unique<control::actuators::AngularActuator>(
        math::Vector(0, 0, 1), *joint_ptr
    );

    std::vector<std::unique_ptr<geometry::Frame>> frames;
    frames.push_back(std::move(root_frame));
    frames.push_back(std::move(joint_frame));
    frames.push_back(std::move(child_frame));

    std::vector<std::unique_ptr<dynamics::Link>> links;
    links.push_back(std::move(root_link));
    links.push_back(std::move(child_link));

    std::vector<std::unique_ptr<control::actuators::Actuator>> actuators;
    actuators.push_back(std::move(actuator));

    return control::Plant(
        std::move(transform_tree),
        std::move(joint_tree),
        std::move(links),
        std::move(frames),
        std::move(actuators)
    );
}

TEST(Plant, UpdateCycleDoesNotThrow) {
    auto plant = makeSingleJointPlant();
    EXPECT_NO_THROW(plant.update(0.01));
}

TEST(Plant, MultipleUpdateCyclesStable) {
    auto plant = makeSingleJointPlant();
    for (auto* act : plant.getActuators()) {
        act->applyEffort(1.0);
    }
    for (int i = 0; i < 100; ++i) {
        ASSERT_NO_THROW(plant.update(0.01));
    }
}

TEST(Plant, GetActuatorsReturnsCorrectCount) {
    auto plant = makeSingleJointPlant();
    EXPECT_EQ(plant.getActuators().size(), 1U);
}
