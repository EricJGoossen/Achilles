#include "algorithms/joint_limit/joint_limit_ops.hpp"

#include <cstddef>
#include <cstdint>

#include "algorithms/conventions.hpp"
#include "algorithms/joint_limit/joint_limit_data.hpp"
#include "util/simd_ops.hpp"

namespace achilles::algorithms::joint_limit {

void DetectJointLimitOp::operator()(
    const MotionSubspace& S,
    const Transform& q,
    const Mat6Mask& limited,
    const JointCoordinates& lower,
    const JointCoordinates& upper,
    const Scalar& margin,
    JointCoordinates* distance_out,
    JointCoordinates* side_out,
    Mat6Mask* active_out
) const {
  using Bits = util::MaskStorageFor<MathematicalT>;

  // theta = S^T Log(q)
  Position theta = S.Transpose() * q.Log();

  JointCoordinates distance;
  JointCoordinates side;
  Bits within(0);
  for (std::size_t k = 0; k < 6; ++k) {
    // d = min(theta - lower, upper - theta), side +1 at the lower end
    MathematicalT to_lower = theta[k] - lower[k];
    MathematicalT to_upper = upper[k] - theta[k];
    auto at_lower = to_lower < to_upper;
    distance[k] = util::Select(at_lower, to_lower, to_upper);
    side[k] = util::Select(at_lower, MathematicalT(1), MathematicalT(-1));

    within =
        within |
        util::Select(
            distance[k] < margin.Value(), Bits(std::int32_t{1} << k), Bits(0)
        );
  }

  *distance_out = distance;
  *side_out = side;
  *active_out = Mat6Mask(within) & limited;
}

}  // namespace achilles::algorithms::joint_limit
