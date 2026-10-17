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
#include "engine/memory/binding.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "util/simd_ops.hpp"

namespace achilles::engine::view {

// One field's (memory-style, packing, index-space) triple. Bundled rather
// than kept as parallel template packs, because a View's fields are
// heterogeneous per-field, not uniform: there is no way to zip an
// independent Policies... pack against an Assemblers... pack and know
// Policies[i] belongs to Assemblers[i] specifically. ViewFactory builds this
// pack from Traits (see view_factory.hpp); nothing else needs to construct
// one by hand. IndexingT defaults to topology::RowIndexing, the same default
// memory::FieldIndexingT resolves a Traits<F> with no Indexing to.
template <
    topology::LayoutPolicyLike PolicyT,
    AssemblerLike AssemblerT,
    topology::IndexingPolicyLike IndexingT = topology::RowIndexing>
struct FieldBinding {
  using Layout = PolicyT;
  using Assembler = AssemblerT;
  using Indexing = IndexingT;
};

template <typename T>
concept FieldBindingLike =
    requires {
      typename T::Layout;
      typename T::Assembler;
      typename T::Indexing;
    } && topology::LayoutPolicyLike<typename T::Layout> &&
    AssemblerLike<typename T::Assembler> &&
    topology::IndexingPolicyLike<typename T::Indexing>;

// A field-indexed view over externally-owned memory: every field's own
// (memory-style Layout policy, Assembler) pair is resolved independently
// (see FieldBinding above), so this class has no built-in notion of "the"
// layout -- each field's own FieldLayout<F> owns every formula
// (StrideBytes, ElementStrideBytes, ...) this class uses to place and
// address that field's data. A field bound to topology::PlanarLayout and
// a field bound to some other policy sit side by side here with no special
// casing between them.
template <FieldEnumLike EnumT, FieldBindingLike... Bindings>
class View {
  static_assert(
      sizeof...(Bindings) == static_cast<size_t>(EnumT::kCount),
      "Binding pack size must equal EnumT::kCount."
  );

  using BindingTuple = std::tuple<Bindings...>;
  static constexpr size_t kNumTypes = sizeof...(Bindings);

 public:
  // One base pointer per field (EnumT order). Each field's array is its own
  // contiguous allocation -- of FieldIndexing<F>::Count(num_instances)
  // elements, i.e. num_instances for an ordinary row-indexed field --
  // independently placed by the allocator regardless of which Layout policy
  // it uses -- there is no shared arena to compute cross-field offsets into.
  View(std::array<std::byte*, kNumTypes> type_bases, size_t num_instances)
      : type_bases_(type_bases), num_instances_(num_instances) {}

  template <EnumT F>
  using Binding = std::tuple_element_t<static_cast<size_t>(F), BindingTuple>;

  template <EnumT F>
  using Assembler = typename Binding<F>::Assembler;

  // Field F's own memory-style policy, resolved from its Traits by
  // ViewFactory/FieldLayoutT (binding.hpp) -- genuinely per-field, never
  // assumed uniform across a View's whole field set.
  template <EnumT F>
  using FieldLayout = typename Binding<F>::Layout;

  // Field F's own index space (see topology/indexing_policy.hpp): how many
  // elements its block holds per View row count, and how a traversal's
  // (target, source) pair resolves to one of them.
  template <EnumT F>
  using FieldIndexing = typename Binding<F>::Indexing;

  template <EnumT F>
  using Scalar = typename Assembler<F>::ScalarType;

  // T picks the shape read/written for field F: Scalar<F> for one plain
  // instance, or an xsimd::batch of it to gather several lanes' worth at
  // once (Assembler<F>::Read/Write reject any other T). `index` is in units
  // of however many raw instances one T covers -- instance index directly
  // when T is scalar, batch index when T is batched.
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
    return FieldIndexing<F>::Count(num_instances_) / LaneSize<T>();
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

  // Caches one field's resolved base pointer + stride for a hot loop.
  // Unaffected by which Layout policy field F uses -- that policy is
  // already baked into field0_/stride_ by the time Field<F>() hands this
  // out.
  template <EnumT F>
  class FieldCursor {
   public:
    // The index an argument of field F reads/writes at, given every index
    // the traversal handed (`indices`, a tuple ordered the way
    // engine::ArgIndex numbers them) and which of them the argument names
    // (Slot) -- entirely field F's own Indexing policy's call (see
    // topology/indexing_policy.hpp), resolved at compile time. Returns
    // either a group index (for Load/Store below) or, for a row-indexed
    // source argument under a PairTraversal, a BroadcastIndex (Load only).
    template <util::ArithmeticLike T, size_t Slot, typename Indices>
    auto Resolve(const Indices& indices) const {
      return FieldIndexing<F>::template Resolve<Slot, LaneSize<T>()>(
          indices, rows_
      );
    }

    template <util::ArithmeticLike T>
    Value<F, T> Load(size_t index) const {
      return Assembler<F>::template Read<T>(
          ElementAt(index * LaneSize<T>()), stride_
      );
    }

    // One row's value, splatted across every lane of T -- see
    // topology::BroadcastIndex. Deliberately no matching Store.
    template <util::ArithmeticLike T>
    Value<F, T> Load(topology::BroadcastIndex index) const {
      return Assembler<F>::template Broadcast<T>(ElementAt(index.row), stride_);
    }

    template <util::ArithmeticLike T>
    void Store(size_t index, const Value<F, T>& value) const {
      Assembler<F>::template Write<T>(
          ElementAt(index * LaneSize<T>()), stride_, value
      );
    }

   private:
    friend class View<EnumT, Bindings...>;
    FieldCursor(std::byte* field0, size_t stride, size_t rows)
        : field0_(field0), stride_(stride), rows_(rows) {}

    std::byte* ElementAt(size_t instance) const {
      return field0_ +
             instance *
                 FieldLayout<F>::template ElementStrideBytes<Assembler<F>>();
    }

    std::byte* field0_;
    size_t stride_;
    size_t rows_;
  };

  template <EnumT F>
  FieldCursor<F> Field() const {
    return FieldCursor<F>(
        FieldZero<F>(0), FieldStrideBytes<F>(), num_instances_
    );
  }

 private:
  // Byte distance between field F's own sub-fields -- entirely owned by F's
  // own Layout policy, never a formula this class inlines itself, so each
  // field computes its own stride independently of every other field's
  // policy. Taken over however many elements F's own Indexing policy says
  // its block holds, not the raw row count.
  template <EnumT F>
  size_t FieldStrideBytes() const {
    return FieldLayout<F>::template StrideBytes<Assembler<F>>(
        FieldIndexing<F>::Count(num_instances_)
    );
  }

  // Byte offset of `instance` within field F's block -- likewise entirely
  // policy-dependent (e.g. topology::PlanarLayout's ElementStrideBytes<A>()
  // is one scalar wide, sizeof(Scalar<F>), but a different policy answers
  // this however its own storage shape requires).
  template <EnumT F>
  std::byte* FieldZero(size_t instance) const {
    return type_bases_[static_cast<size_t>(F)] +
           instance *
               FieldLayout<F>::template ElementStrideBytes<Assembler<F>>();
  }

  std::array<std::byte*, kNumTypes> type_bases_;
  size_t num_instances_;
};

}  // namespace achilles::engine::view
