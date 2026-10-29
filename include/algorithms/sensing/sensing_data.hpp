#pragma once

#include <cstddef>
#include <cstdint>
#include <tuple>

#include "algorithms/sensing/sensor.hpp"
#include "algorithms/sensing/sensor_kinds.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/joint_topology.hpp"
#include "engine/assembler.hpp"
#include "engine/field_contract.hpp"
#include "engine/memory/buffering_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::sensing {

enum class SensingField : std::uint8_t {
  kConfig,      // each sensor itself
  kClock,       // its SensorClock
  kState,       // its kind's State
  kOutput,      // the reading the Read pass writes
  kPublished,   // the last complete reading, what the user reads
  kFirstInput,  // then one per SensorInputs entry, read at the mount joint
                // or, by a scanning kind, at every body

  kCount = kFirstInput + util::kSizeV<SensorInputs>
};
static_assert(engine::FieldEnumLike<SensingField>);

template <SensingField F>
struct SensingFieldTraits;

// Config, state and output hold a different record type per kind, so they
// use SegmentedLayout: each kind's rows take exactly its own records' size.
template <>
struct SensingFieldTraits<SensingField::kConfig> {
  using Assembler = util::ApplyT<engine::LeafAssembler, SensorKinds>;
  using Ordering = SensingOrdering;
  using Layout = engine::topology::SegmentedLayout;
};
template <>
struct SensingFieldTraits<SensingField::kClock> {
  using Assembler = engine::LeafAssembler<SensorClock>;
  using Ordering = SensingOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct SensingFieldTraits<SensingField::kState> {
  using Assembler = util::ApplyT<engine::LeafAssembler, SensorStates>;
  using Ordering = SensingOrdering;
  using Layout = engine::topology::SegmentedLayout;
};
// Only the output is double-buffered: it's the one thing handed to the
// user. This is the sim's side, the half this tick's reading goes into;
// ReadStep flips the group once its passes are done, so it becomes
// kPublished.
template <>
struct SensingFieldTraits<SensingField::kOutput> {
  using Assembler = util::ApplyT<engine::LeafAssembler, SensorOutputs>;
  using SharedAs = SensingOutputSlot;
  using Buffering = engine::memory::
      DoubleBuffered<SensingOutputSlot, engine::memory::BufferSide::kWrite>;
  using Ordering = SensingOrdering;
  using Layout = engine::topology::SegmentedLayout;
};
// The user's side: the last complete reading, read between ticks, and by
// the Read pass as the reading to carry forward when no sample is due.
template <>
struct SensingFieldTraits<SensingField::kPublished> {
  using Assembler = util::ApplyT<engine::LeafAssembler, SensorOutputs>;
  using SharedAs = SensingOutputSlot;
  using Buffering = engine::memory::
      DoubleBuffered<SensingOutputSlot, engine::memory::BufferSide::kRead>;
  using Ordering = SensingOrdering;
  using Layout = engine::topology::SegmentedLayout;
};
// An input's source field, exactly as its own Algorithm declared it -- the
// same SharedAs slot, so this is the source's block, not a copy -- minus
// any kName (the source already populates it).
template <SensingField F>
  requires(F >= SensingField::kFirstInput && F < SensingField::kCount)
struct SensingFieldTraits<F> {
  using Source = InputSource<std::tuple_element_t<
      static_cast<std::size_t>(F) -
          static_cast<std::size_t>(SensingField::kFirstInput),
      util::ToTupleT<SensorInputs>>>;
  using Assembler = typename Source::Type::Assembler;
  using SharedAs = typename Source::Type::SharedAs;
  using Ordering = typename Source::Ordering;
  using Indexing = typename Source::Indexing;
  using Buffering = typename Source::Buffering;
  using Layout = typename Source::Layout;
};
static_assert(engine::FieldTraitsLike<SensingField, SensingFieldTraits>);

using SensingView = engine::view::ViewFactory<SensingField, SensingFieldTraits>;
static_assert(engine::view::ViewLike<SensingView, SensingField>);

using SensingTopology = domain::JointTopology;
static_assert(domain::TopologyLike<SensingTopology>);

}  // namespace achilles::algorithms::sensing
