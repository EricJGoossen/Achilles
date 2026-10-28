#pragma once

#include <array>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <xsimd/xsimd.hpp>

#include "engine/assembler.hpp"
#include "engine/field_contract.hpp"
#include "engine/memory/binding.hpp"
#include "engine/memory/buffering_policy.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "util/simd_ops.hpp"

namespace achilles::engine::view {

// One field's (memory-style, packing, index-space, buffering) tuple. Bundled
// rather than kept as parallel template packs, because a View's fields are
// heterogeneous per-field, not uniform: there is no way to zip an
// independent Policies... pack against an Assemblers... pack and know
// Policies[i] belongs to Assemblers[i] specifically. ViewFactory builds this
// pack from Traits (see view_factory.hpp); nothing else needs to construct
// one by hand. IndexingT defaults to topology::RowIndexing and BufferingT to
// memory::SingleBuffered, the same defaults memory::FieldIndexingT/
// FieldBufferingT resolve a Traits<F> naming neither to.
template <
    topology::LayoutPolicyLike PolicyT,
    AssemblerLike AssemblerT,
    topology::IndexingPolicyLike IndexingT = topology::RowIndexing,
    memory::BufferingPolicyLike BufferingT = memory::SingleBuffered>
  requires AssemblesThrough<AssemblerT, PolicyT>
struct FieldBinding {
  using Layout = PolicyT;
  using Assembler = AssemblerT;
  using Indexing = IndexingT;
  using Buffering = BufferingT;
};

template <typename T>
concept FieldBindingLike =
    requires {
      typename T::Layout;
      typename T::Assembler;
      typename T::Indexing;
      typename T::Buffering;
    } && topology::LayoutPolicyLike<typename T::Layout> &&
    AssemblesThrough<typename T::Assembler, typename T::Layout> &&
    topology::IndexingPolicyLike<typename T::Indexing> &&
    memory::BufferingPolicyLike<typename T::Buffering>;

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
  // A double-buffered field's base is the start of its two-half region
  // (see memory::DoubleBuffered), and its entry in `parities` is its
  // group's parity, saying which half it addresses; a single-buffered
  // field's entry is ignored (null).
  View(
      std::array<std::byte*, kNumTypes> type_bases,
      size_t num_instances,
      std::array<const memory::BufferParity*, kNumTypes> parities = {}
  )
      : type_bases_(type_bases),
        num_instances_(num_instances),
        parities_(parities) {}

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

  // Field F's own buffering policy (see memory/buffering_policy.hpp): how
  // many copies of its block exist and which one Load/Store/Field reach.
  template <EnumT F>
  using FieldBuffering = typename Binding<F>::Buffering;

  template <EnumT F>
  using Scalar = typename Assembler<F>::ScalarType;

  // T picks the shape read/written for field F: Scalar<F> for one plain
  // instance, or an xsimd::batch of it to gather several lanes' worth at
  // once (Assembler<F>::Value rejects any other T). `index` is in units
  // of however many raw instances one T covers -- instance index directly
  // when T is scalar, batch index when T is batched.
  template <EnumT F, FieldShapeLike T>
  using Value = typename Assembler<F>::template Value<T>;

  template <FieldShapeLike T>
  static constexpr size_t LaneSize() {
    return util::LaneCountOf<T>();
  }

  size_t Size() const { return num_instances_; }

  template <EnumT F, FieldShapeLike T>
  size_t NumBatches() const {
    return FieldIndexing<F>::Count(num_instances_) / LaneSize<T>();
  }

  template <EnumT F, FieldShapeLike T>
  Value<F, T> Load(size_t index) const {
    return Assembler<F>::template Read<T, FieldLayout<F>>(
        FieldZero<F>(index * LaneSize<T>()), FieldStrideBytes<F>()
    );
  }

  template <EnumT F, FieldShapeLike T>
  void Store(size_t index, const Value<F, T>& value) const {
    Assembler<F>::template Write<T, FieldLayout<F>>(
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
    // either a group index or, for a row-indexed source argument under a
    // PairTraversal (or a batched GlobalIndexing one anywhere), a
    // BroadcastIndex -- broadcast when loaded, a lane reduction when
    // stored.
    template <FieldShapeLike T, size_t Slot, typename Indices>
    auto Resolve(const Indices& indices) const {
      return FieldIndexing<F>::template Resolve<Slot, LaneSize<T>()>(
          indices, rows_
      );
    }

    template <FieldShapeLike T>
    Value<F, T> Load(size_t index) const {
      return Assembler<F>::template Read<T, FieldLayout<F>>(
          ElementAt(index * LaneSize<T>()), stride_
      );
    }

    // One row's value, splatted across every lane of T -- see
    // topology::BroadcastIndex. An input's Load.
    template <FieldShapeLike T>
    Value<F, T> Load(topology::BroadcastIndex index) const {
      return Assembler<F>::template Broadcast<T, FieldLayout<F>>(
          ElementAt(index.row), stride_
      );
    }

    // What an output starts from before its Op runs. At a group index,
    // whatever the field holds there (outputs are read/write); at a
    // BroadcastIndex, zero -- the output is a reduction into that one row
    // (see the accumulating Store below), so the Op adds its lanes'
    // contributions onto nothing, never onto a splat of the row that every
    // lane would then sum back in.
    template <FieldShapeLike T>
    Value<F, T> LoadOutput(size_t index) const {
      return Load<T>(index);
    }
    template <FieldShapeLike T>
    Value<F, T> LoadOutput(topology::BroadcastIndex /*index*/) const {
      return Assembler<F>::template Zero<T>();
    }

    template <FieldShapeLike T>
    void Store(size_t index, const Value<F, T>& value) const {
      Assembler<F>::template Write<T, FieldLayout<F>>(
          ElementAt(index * LaneSize<T>()), stride_, value
      );
    }

    // row += (sum of value's lanes) -- see topology::BroadcastIndex.
    // Read-modify-write at store time, so it lands on top of anything
    // stored into the same memory earlier in the same call.
    template <FieldShapeLike T>
    void Store(topology::BroadcastIndex index, const Value<F, T>& value) const {
      Assembler<F>::template Accumulate<T, FieldLayout<F>>(
          ElementAt(index.row), stride_, value
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

  // A handle to (row, leaf) of double-buffered field F's write half -- the
  // half its group's read side is not on (see memory::BufferedScalar).
  // `leaf` is the scalar's position in F's Assembler flattening order.
  template <EnumT F>
    requires memory::GroupBufferedLike<FieldBuffering<F>>
  memory::BufferedScalar<Scalar<F>> WriteSlot(size_t row, size_t leaf) const {
    assert(row < FieldIndexing<F>::Count(num_instances_));
    assert(leaf < Assembler<F>::kNumFields);
    std::byte* first_element = FieldLayout<F>::LeafAt(
        type_bases_[static_cast<size_t>(F)] +
            (row * FieldLayout<F>::template ElementStrideBytes<Assembler<F>>()),
        leaf,
        FieldStrideBytes<F>()
    );
    return memory::BufferedScalar<Scalar<F>>(
        first_element, FieldBlockBytes<F>(), parities_[static_cast<size_t>(F)]
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
  // For a double-buffered field, offset into whichever half this field's
  // side addresses right now (FieldBuffering<F>::ReadOffset -- always 0 for
  // any other field).
  template <EnumT F>
  std::byte* FieldZero(size_t instance) const {
    return type_bases_[static_cast<size_t>(F)] +
           FieldBuffering<F>::ReadOffset(
               parities_[static_cast<size_t>(F)], FieldBlockBytes<F>()
           ) +
           instance *
               FieldLayout<F>::template ElementStrideBytes<Assembler<F>>();
  }

  // Bytes of ONE copy of field F's block -- the distance between a
  // double-buffered field's two halves.
  template <EnumT F>
  size_t FieldBlockBytes() const {
    return FieldLayout<F>::template BlockBytes<Assembler<F>>(
        FieldIndexing<F>::Count(num_instances_)
    );
  }

  std::array<std::byte*, kNumTypes> type_bases_;
  size_t num_instances_;
  std::array<const memory::BufferParity*, kNumTypes> parities_;
};

}  // namespace achilles::engine::view
