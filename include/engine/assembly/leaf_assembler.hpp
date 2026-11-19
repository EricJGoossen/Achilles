// One leaf of a field: a record read and written whole.

#pragma once

#include <bit>
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <xsimd/xsimd.hpp>

#include "engine/assembly/assembler_contract.hpp"
#include "util/simd_ops.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::assembly {

// One leaf: a record, read and written whole through the Layout (which
// decides where it sits and how to load it). A scalar is just the simplest
// record. Every Assembler tree bottoms out in these -- an arithmetic child
// listed in an Assembler's Children (float, RepeatTypes<3>, ...) is a
// LeafAssembler<float> -- and one can also be a whole field on its own:
// LeafAssembler<LidarConfig> is a field holding one LidarConfig per row,
// which an Op takes directly as `const LidarConfig&` / `LidarConfig*`.
//
// With several record types, each row holds one of them: Rs' k-th type in
// segment k of the field's ordering (see policy::layout::SegmentedLayout, the
// layout such a field uses), so an Op written for segment k takes
// `const R_k&` / `R_k*`. ScalarType is the first record.
//
// A single arithmetic record reads as itself or as a batch of itself, and
// can be zeroed and accumulated into; any other record only reads as itself
// -- one instance at a time, no lanes.
template <typename... Rs>
  requires(
      sizeof...(Rs) > 0 && ((std::is_trivially_copyable_v<Rs> &&
                             std::is_default_constructible_v<Rs>) &&
                            ...)
  )
struct LeafAssembler {
  using Records = util::TypeList<Rs...>;
  using ScalarType = std::tuple_element_t<0, std::tuple<Rs...>>;
  static constexpr size_t kNumFields = 1;

 private:
  static constexpr bool kScalar =
      sizeof...(Rs) == 1 && std::is_arithmetic_v<ScalarType>;

  // The record a read or write as TargetScalar touches: itself, or a
  // batch's own scalar.
  template <typename TargetScalar>
  using LeafOf = std::conditional_t<
      util::kIsOneOfV<TargetScalar, Rs...>,
      TargetScalar,
      ScalarType>;

 public:
  template <typename TargetScalar>
    requires(util::kIsOneOfV<TargetScalar, Rs...> ||
             (kScalar && std::same_as<TargetScalar, xsimd::batch<ScalarType>>))
  using Value = TargetScalar;

  template <typename TargetScalar, typename Layout>
  static Value<TargetScalar> Read(
      const std::byte* field0, size_t /*field_stride_bytes*/
  ) {
    return Layout::template LoadLeaf<TargetScalar, LeafOf<TargetScalar>>(field0
    );
  }

  template <typename TargetScalar, typename Layout>
  static Value<TargetScalar> Broadcast(
      const std::byte* field0, size_t /*field_stride_bytes*/
  ) {
    return Layout::template BroadcastLeaf<TargetScalar, LeafOf<TargetScalar>>(
        field0
    );
  }

  template <typename TargetScalar, typename Layout>
  static void Write(
      std::byte* field0,
      size_t /*field_stride_bytes*/,
      const Value<TargetScalar>& value
  ) {
    Layout::template StoreLeaf<LeafOf<TargetScalar>>(field0, value);
  }

  template <typename TargetScalar>
    requires kScalar
  static Value<TargetScalar> Zero() {
    return TargetScalar(0);
  }

  template <typename TargetScalar, typename Layout>
    requires kScalar
  static void Accumulate(
      std::byte* field0,
      size_t /*field_stride_bytes*/,
      const Value<TargetScalar>& value
  ) {
    Layout::template AccumulateLeaf<ScalarType>(field0, value);
  }

  // What a padding row holds when this leaf is a whole field: a
  // value-initialized record. (Inside an Assembler tree the tree's own
  // Target decides -- see Assembler::PaddingSeed. A segmented field's
  // padding rows have no storage, so its seed is never written.)
  static ScalarType PaddingSeed() { return ScalarType{}; }
};

}  // namespace achilles::engine::assembly
