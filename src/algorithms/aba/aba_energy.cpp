#include "algorithms/aba/aba_energy.hpp"

#include <cstddef>

#include "domain/math/vector3.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles::algorithms::aba {

namespace {
using ScalarTransform = domain::spatial::Transform<float>;
using ScalarInertia = domain::spatial::Inertia<float>;
using ScalarVelocity = domain::spatial::SpatialVelocity<float>;
using ScalarVector3 = domain::math::Vector3<float>;
}  // namespace

SystemEnergy ComputeSystemEnergy(
    const ABAView& view,
    const domain::JointTopology& topology,
    const ScalarVector3& gravity
) {
  SystemEnergy energy;
  std::size_t row_count = topology.Size();
  for (std::size_t row = 0; row < row_count; ++row) {
    ScalarTransform x_world = view.Load<ABAField::kWorldTransform, float>(row);
    ScalarInertia inertia = view.Load<ABAField::kRigidBodyInertia, float>(row);
    ScalarVelocity v = view.Load<ABAField::kSpatialVelocity, float>(row);

    energy.kinetic += 0.5F * v.AsVector6().Dot(inertia.Apply(v).AsVector6());

    // H() is the body's first moment of mass (m * center-of-mass offset)
    // about the same origin rigid_body_inertia is expressed at -- which,
    // per this codebase's own convention, is the joint's own pivot (see
    // e.g. tests/examples_two_joint_arm.cpp's pivot-referenced inertia
    // parameterization), the exact frame kWorldTransform already places.
    ScalarVector3 com_body = inertia.H() / inertia.Mass();
    ScalarVector3 com_world =
        x_world.Translation() + x_world.Rotation().Rotate(com_body);
    energy.potential -= inertia.Mass() * gravity.Dot(com_world);
  }
  return energy;
}

}  // namespace achilles::algorithms::aba
