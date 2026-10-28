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

namespace detail {

// One field's RowTable inside a View, tagged with its field index so every
// slot is a distinct type. An empty table (a planar field's) is held as a
// static, so its slot is an empty class -- an empty base taking no space.
template <size_t I, typename Table>
struct RowTableSlot {
  Table table;

  const Table& Get() const { return table; }
};

template <size_t I, typename Table>
  requires std::is_empty_v<Table>
struct RowTableSlot<I, Table> {
  static constexpr Table kTable{};

  const Table& Get() const { return kTable; }
};

template <typename Fields, typename... Tables>
struct RowTableSet;

// Every field's RowTable, as an aggregate rather than a std::tuple, so a
// View stays trivially copyable.
template <size_t... Is, typename... Tables>
struct RowTableSet<std::index_sequence<Is...>, Tables...>
    : RowTableSlot<Is, Tables>... {};

}  // namespace detail

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
  // Every field's RowTable (see topology::PlanarLayout::RowTable), held by
  // value: a segmented field's (topology::SegmentedLayout) is its segment
  // table, read straight from the View on every lookup; every other
  // field's is empty, so a View over planar fields only is exactly its
  // bases, row count and parities.
  using RowTables = detail::RowTableSet<
      std::make_index_sequence<sizeof...(Bindings)>,
      typename Bindings::Layout::template RowTable<
          typename Bindings::Assembler>...>;

  // One base pointer per field (EnumT order). Each field's array is its own
  // contiguous allocation -- of FieldIndexing<F>::Count(num_instances)
  // elements, i.e. num_instances for an ordinary row-indexed field --
  // independently placed by the allocator regardless of which Layout policy
  // it uses -- there is no shared arena to compute cross-field offsets into.
  // A double-buffered field's base is the start of its two-half region
  // (see memory::DoubleBuffered), and its entry in `parities` is its
  // group's parity, saying which half it addresses; a single-buffered
  // field's entry is ignored (null). `row_tables` is every field's
  // RowTable (see RowTables).
  View(
      std::array<std::byte*, kNumTypes> type_bases,
      size_t num_instances,
      std::array<const memory::BufferParity*, kNumTypes> parities = {},
      RowTables row_tables = {}
  )
      : type_bases_(type_bases),
        num_instances_(num_instances),
        parities_(parities),
        row_tables_(row_tables) {}

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

  // Field F's RowTable (see RowTables).
  template <EnumT F>
  using RowTable = typename FieldLayout<F>::template RowTable<Assembler<F>>;

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
        FieldAt<F, T>(index * LaneSize<T>()), FieldStrideBytes<F>()
    );
  }

  template <EnumT F, FieldShapeLike T>
  void Store(size_t index, const Value<F, T>& value) const {
    Assembler<F>::template Write<T, FieldLayout<F>>(
        FieldAt<F, T>(index * LaneSize<T>()), FieldStrideBytes<F>(), value
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
          ElementAt<T>(index * LaneSize<T>()), stride_
      );
    }

    // One row's value, splatted across every lane of T -- see
    // topology::BroadcastIndex. An input's Load.
    template <FieldShapeLike T>
    Value<F, T> Load(topology::BroadcastIndex index) const {
      return Assembler<F>::template Broadcast<T, FieldLayout<F>>(
          ElementAt<T>(index.row), stride_
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
          ElementAt<T>(index * LaneSize<T>()), stride_, value
      );
    }

    // row += (sum of value's lanes) -- see topology::BroadcastIndex.
    // Read-modify-write at store time, so it lands on top of anything
    // stored into the same memory earlier in the same call.
    template <FieldShapeLike T>
    void Store(topology::BroadcastIndex index, const Value<F, T>& value) const {
      Assembler<F>::template Accumulate<T, FieldLayout<F>>(
          ElementAt<T>(index.row), stride_, value
      );
    }

   private:
    friend class View<EnumT, Bindings...>;
    using CursorTable =
        typename FieldLayout<F>::template CursorTable<Assembler<F>>;

    // Whatever F's layout addresses rows from, worked out once here rather
    // than per row (see e.g. topology::SegmentedLayout::CursorTableOf);
    // nothing for a planar field.
    FieldCursor(
        std::byte* field0,
        size_t stride,
        size_t rows,
        const RowTable<F>& row_table
    )
        : field0_(field0),
          stride_(stride),
          rows_(rows),
          table_(FieldLayout<F>::template CursorTableOf<Assembler<F>>(
              field0, row_table
          )) {}

    template <typename T>
    std::byte* ElementAt(size_t instance) const {
      return FieldLayout<F>::template CursorElement<Assembler<F>, T>(
          field0_, table_, instance
      );
    }

    std::byte* field0_;
    size_t stride_;
    size_t rows_;
    [[no_unique_address]] CursorTable table_;
  };

  template <EnumT F>
  FieldCursor<F> Field() const {
    return FieldCursor<F>(
        FieldBase<F>(),
        FieldStrideBytes<F>(),
        num_instances_,
        FieldRowTable<F>()
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
            FieldLayout<F>::template RowOffset<Assembler<F>, Scalar<F>>(
                row, FieldRowTable<F>()
            ),
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

  // Where field F's block starts: for a double-buffered field, whichever
  // half this field's side addresses right now (FieldBuffering<F>::
  // ReadOffset -- always 0 for any other field).
  template <EnumT F>
  std::byte* FieldBase() const {
    return type_bases_[static_cast<size_t>(F)] +
           FieldBuffering<F>::ReadOffset(
               parities_[static_cast<size_t>(F)], FieldBlockBytes<F>()
           );
  }

  // Address of `instance`, read as T, within field F's block -- entirely
  // policy-dependent (topology::PlanarLayout steps a fixed
  // ElementStrideBytes<A>() per instance; topology::SegmentedLayout looks
  // up T's segment in F's table).
  template <EnumT F, typename T>
  std::byte* FieldAt(size_t instance) const {
    return FieldBase<F>() + FieldLayout<F>::template RowOffset<Assembler<F>, T>(
                                instance, FieldRowTable<F>()
                            );
  }

  // Bytes of ONE copy of field F's block -- the distance between a
  // double-buffered field's two halves.
  template <EnumT F>
  size_t FieldBlockBytes() const {
    return FieldLayout<F>::template BlockBytes<Assembler<F>>(
        FieldIndexing<F>::Count(num_instances_), FieldRowTable<F>()
    );
  }

  template <EnumT F>
  const RowTable<F>& FieldRowTable() const {
    using Slot = detail::RowTableSlot<static_cast<size_t>(F), RowTable<F>>;
    return static_cast<const Slot&>(row_tables_).Get();
  }

  std::array<std::byte*, kNumTypes> type_bases_;
  size_t num_instances_;
  std::array<const memory::BufferParity*, kNumTypes> parities_;
  [[no_unique_address]] RowTables row_tables_;
};

}  // namespace achilles::engine::view
