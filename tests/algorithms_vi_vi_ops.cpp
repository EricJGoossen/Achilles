#include <gtest/gtest.h>

#include "algorithms/conventions.hpp"
#include "algorithms/vi/vi_ops.hpp"
#include "util/simd_ops.hpp"

// IntegrateVelocityOp wires SpatialAcceleration::Integrate (domain/spatial/
// dual.hpp: `SpatialVelocity<T>(this->AsVector6() * dt)`) into a single
// explicit-Euler velocity update: qd += qdd * dt. These tests confirm that
// wiring -- right scale, right sign, accumulate rather than overwrite --
// not an independent derivation of explicit Euler integration from first
// principles; that's a top-to-bottom correctness file to design separately
// (matches the scope aba_ops.cpp's own header comment describes).
//
// Every domain type here is instantiated at achilles::algorithms'
// MathematicalT (xsimd::batch<float>), the only scalar shape this Op is
// ever actually called with in production -- not float. Values are
// broadcast uniformly across lanes and read back as whole batches;
// per-lane independence is already covered generically for the underlying
// math types (e.g. Vector3Batched), so it isn't re-proven here.

using namespace achilles::algorithms;
using namespace achilles::algorithms::vi;

namespace {

using B = MathematicalT;

::testing::AssertionResult BatchTrue(const auto& mask) {
  if (achilles::util::AllTrue(mask)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "not all lanes true";
}

}  // namespace

// qdd == 0 means Integrate(dt) contributes exactly nothing, regardless of
// dt -- the pre-existing velocity must survive unchanged, proving the Op
// doesn't zero or otherwise disturb its output when there's nothing to add.
TEST(IntegrateVelocityOpTest, ZeroAccelerationLeavesVelocityUnchanged) {
  Acceleration qdd = Acceleration::Zero();
  Velocity qd_initial(
      Vector3(B(0.1F), B(0.2F), B(0.3F)), Vector3(B(0.4F), B(0.5F), B(0.6F))
  );
  Velocity qd_out = qd_initial;

  IntegrateVelocityOp op(0.5F);
  op(qdd, &qd_out);

  EXPECT_TRUE(BatchTrue(qd_out.IsApprox(qd_initial)));
}

// dt == 0 means no time has passed -- velocity must survive unchanged no
// matter how large the acceleration is. Distinct from the zero-acceleration
// case above: this exercises the dt argument's own zero, not qdd's.
TEST(IntegrateVelocityOpTest, ZeroDtLeavesVelocityUnchanged) {
  Acceleration qdd(
      Vector3(B(10.0F), B(-5.0F), B(2.0F)), Vector3(B(1.0F), B(1.0F), B(1.0F))
  );
  Velocity qd_initial(
      Vector3(B(0.1F), B(0.2F), B(0.3F)), Vector3(B(0.4F), B(0.5F), B(0.6F))
  );
  Velocity qd_out = qd_initial;

  IntegrateVelocityOp op(0.0F);
  op(qdd, &qd_out);

  EXPECT_TRUE(BatchTrue(qd_out.IsApprox(qd_initial)));
}

// Nonzero acceleration and dt: qd_out must equal the pre-existing velocity
// plus qdd*dt, recomputed independently via Vector6's own scalar multiply
// (domain_math_vector6.cpp) rather than by calling Integrate again. The
// pre-existing nonzero value must still be present in the result -- proving
// += (accumulate into whatever the View already held), not = (overwrite) --
// see TESTING.md 9.2 on overwrite-vs-accumulate coverage.
TEST(IntegrateVelocityOpTest, AccumulatesScaledAccelerationOntoExistingVelocity) {
  Acceleration qdd(
      Vector3(B(2.0F), B(-1.0F), B(0.5F)), Vector3(B(-3.0F), B(4.0F), B(0.0F))
  );
  Velocity qd_initial(
      Vector3(B(0.1F), B(0.2F), B(0.3F)), Vector3(B(0.4F), B(0.5F), B(0.6F))
  );
  Velocity qd_out = qd_initial;
  B dt(0.1F);

  IntegrateVelocityOp op(0.1F);
  op(qdd, &qd_out);

  Velocity expected_delta(qdd.AsVector6() * dt);
  Velocity expected = qd_initial + expected_delta;

  EXPECT_TRUE(BatchTrue(qd_out.IsApprox(expected)));
}
