// NOLINTBEGIN(misc-include-cleaner) -- main.cpp is deliberately a kitchen
// sink while actively developing: it includes every header so that a
// single-TU compile (and clang-tidy run, via scripts/check-tidy.sh's
// "every header must be reachable from a src/*.cpp file" invariant)
// exercises the whole codebase, not just what main() itself calls. This
// is not the standard for the rest of the codebase -- it's specific to
// this file.
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
#include "util/simd_ops.hpp"
#include "util/tmp.hpp"
// NOLINTEND(misc-include-cleaner)

int main() {
  using achilles::domain::math::Quaternion;
  using achilles::domain::math::Vector3;
  using achilles::domain::spatial::Transform;

  Quaternion<float> rotation =
      Quaternion<float>(0.0F, 0.0F, 0.0F, 1.0F).Normalize();
  Transform<float> world_body(Vector3<float>(1.0F, 0.0F, 0.0F), rotation);

  Vector3<float> body_point(0.0F, 1.0F, 0.0F);
  Vector3<float> world_point =
      rotation.Rotate(body_point) + world_body.Translation();

  std::cout << "Achilles\n";
  std::cout << "world_body translation: " << world_body.Translation() << "\n";
  std::cout << "world_body rotation: " << world_body.Rotation() << "\n";
  std::cout << "body point " << body_point << " -> world point " << world_point
            << "\n";

  return 0;
}
