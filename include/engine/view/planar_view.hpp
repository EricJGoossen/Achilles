#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <xsimd/xsimd.hpp>

#include "engine/assembler.hpp"
#include "engine/field_contract.hpp"
#include "util/simd_ops.hpp"

namespace achilles::engine::view {

template <FieldEnumLike EnumT, AssemblerLike... Assemblers>
class PlanarView {
  static_assert(
      sizeof...(Assemblers) == static_cast<size_t>(EnumT::kCount),
      "Assembler pack size must equal EnumT::kCount."
  );

  using AssemblerTuple = std::tuple<Assemblers...>;
  static constexpr size_t kNumTypes = sizeof...(Assemblers);

 public:
  // One base pointer per type (EnumT order). Each type's array is its own
  // contiguous allocation of num_instances, independently placed by the
  // allocator -- types are not assumed to be laid out relative to one
  // another, so there is no shared arena to compute offsets into.
  PlanarView(std::array<std::byte*, kNumTypes> type_bases, size_t num_instances)
      : type_bases_(type_bases), num_instances_(num_instances) {}

  template <EnumT F>
  using Assembler =
      std::tuple_element_t<static_cast<size_t>(F), AssemblerTuple>;

  template <EnumT F>
  using Scalar = typename Assembler<F>::ScalarType;

  // T picks the shape read/written for field F: Scalar<F> for one plain
  // instance, or an xsimd::batch of it to gather several lanes' worth at
  // once (Assembler<F>::Read/Write reject any other T). `index` is in
  // units of however many raw instances one T covers -- instance index
  // directly when T is scalar, batch index when T is batched -- so the
  // same call shape serves both without a separate "batched" spelling.
  template <EnumT F, util::ArithmeticLike T>
  using Value = decltype(Assembler<F>::template Read<T>(
      std::declval<const std::byte*>(), size_t{0}
  ));

  template <util::ArithmeticLike T>
  static constexpr size_t LaneSize() {
    return util::LaneCountOf<T>();
  }

  size_t Size() const { return num_instances_; }

  template <EnumT F, util::ArithmeticLike T>
  size_t NumBatches() const {
    return num_instances_ / LaneSize<T>();
  }

  template <EnumT F, util::ArithmeticLike T>
  Value<F, T> Load(size_t index) const {
    return Assembler<F>::template Read<T>(
        FieldZero<F>(index * LaneSize<T>()), FieldStrideBytes<F>()
    );
  }

  template <EnumT F, util::ArithmeticLike T>
  void Store(size_t index, const Value<F, T>& value) const {
    Assembler<F>::template Write<T>(
        FieldZero<F>(index * LaneSize<T>()), FieldStrideBytes<F>(), value
    );
  }

  // Caches one field's resolved base pointer + stride once, for repeated
  // access in a hot loop -- 16 bytes, vs. holding the full PlanarView
  // (one std::byte* per type, ~160 bytes at ~20 types). Look a field up
  // via Field<F>() once outside the loop and index the cursor inside it.
  template <EnumT F>
  class FieldCursor {
   public:
    template <util::ArithmeticLike T>
    Value<F, T> Load(size_t index) const {
      return Assembler<F>::template Read<T>(
          field0_ + index * LaneSize<T>() * sizeof(Scalar<F>), stride_
      );
    }

    template <util::ArithmeticLike T>
    void Store(size_t index, const Value<F, T>& value) const {
      Assembler<F>::template Write<T>(
          field0_ + index * LaneSize<T>() * sizeof(Scalar<F>), stride_, value
      );
    }

   private:
    friend class PlanarView<EnumT, Assemblers...>;
    FieldCursor(std::byte* field0, size_t stride)
        : field0_(field0), stride_(stride) {}

    std::byte* field0_;
    size_t stride_;
  };

  template <EnumT F>
  FieldCursor<F> Field() const {
    return FieldCursor<F>(FieldZero<F>(0), FieldStrideBytes<F>());
  }

 private:
  // Stride between type F's own sub-fields (e.g. x/y/z of a Vector3) --
  // not an offset into any shared arena. Each type's block is sized for
  // its own num_instances_ independent of any other type's layout.
  template <EnumT F>
  size_t FieldStrideBytes() const {
    constexpr size_t kAlignment = alignof(xsimd::batch<Scalar<F>>);
    size_t raw_bytes = num_instances_ * sizeof(Scalar<F>);
    return (raw_bytes + kAlignment - 1) / kAlignment * kAlignment;
  }

  // type_bases_[F] is already this field's array start -- no arena offset
  // to add, since each type owns its own independent allocation.
  template <EnumT F>
  std::byte* FieldZero(size_t instance) const {
    return type_bases_[static_cast<size_t>(F)] + instance * sizeof(Scalar<F>);
  }

  std::array<std::byte*, kNumTypes> type_bases_;
  size_t num_instances_;
};

}  // namespace achilles::engine::view