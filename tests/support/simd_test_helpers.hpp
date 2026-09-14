#pragma once

#include <cstddef>
#include <vector>

#include <xsimd/xsimd.hpp>

namespace achilles::test_support {

// Builds an xsimd::batch<float> with `lanes[i]` in lane i and the last
// given value repeated into any remaining lanes (so callers don't need to
// know the native batch width). Used by domain/math test files to prove a
// type's scalar T is a genuine free parameter -- not just float in
// disguise -- by giving each lane a distinct value and checking the result
// lane-by-lane, the same way tests/test_helpers.cpp already does for
// PlanarView.
inline xsimd::batch<float> MakeBatch(std::initializer_list<float> lanes) {
  constexpr std::size_t kWidth = xsimd::batch<float>::size;
  std::vector<float> buf(kWidth, 0.0F);
  float last = 0.0F;
  std::size_t i = 0;
  for (float v : lanes) {
    if (i >= kWidth) {
      break;
    }
    buf[i++] = v;
    last = v;
  }
  for (; i < kWidth; ++i) {
    buf[i] = last;
  }
  return xsimd::batch<float>::load_unaligned(buf.data());
}

}  // namespace achilles::test_support
