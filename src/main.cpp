#include <iostream>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_ops.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/math/vector6.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"
#include "domain/topology/joint_topology.hpp"
#include "domain/topology/topology_contract.hpp"
#include "engine/algorithm_step.hpp"
#include "engine/assembler.hpp"
#include "engine/field_contract.hpp"
#include "engine/op_contract.hpp"
#include "engine/op_invoker.hpp"
#include "engine/traversals.hpp"
#include "engine/view/planar_view.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "io_stream.hpp"
#include "util/simd_ops.hpp"
#include "util/tmp.hpp"

int main() {
  using achilles::domain::math::Quaternion;
  using achilles::domain::math::Vector3;
  using achilles::domain::spatial::Transform;
  using achilles::operator<<;

  Quaternion<float> rotation =
      Quaternion<float>(0.0f, 0.0f, 0.0f, 1.0f).Normalize();
  Transform<float> world_body(Vector3<float>(1.0f, 0.0f, 0.0f), rotation);

  Vector3<float> body_point(0.0f, 1.0f, 0.0f);
  Vector3<float> world_point =
      rotation.Rotate(body_point) + world_body.Translation();

  std::cout << "Achilles\n";
  std::cout << "world_body translation: " << world_body.Translation() << "\n";
  std::cout << "world_body rotation: " << world_body.Rotation() << "\n";
  std::cout << "body point " << body_point << " -> world point " << world_point
            << "\n";

  return 0;
}
