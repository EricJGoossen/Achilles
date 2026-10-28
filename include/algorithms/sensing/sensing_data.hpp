#pragma once

#include <cstddef>
#include <cstdint>

#include "algorithms/sensing/sensor.hpp"
#include "domain/joint_topology.hpp"
#include "engine/assembler.hpp"
#include "engine/field_contract.hpp"
#include "engine/memory/buffering_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::sensing {

// A kind's rows: one per instance its archetypes attach under S::kListName,
// each linked to its mount joint's row (see engine::topology::ListOrdering).
template <SensorLike S>
using SensingOrdering =
    engine::topology::ListOrdering<S, engine::topology::TopologicalOrdering>;

// A kind's output buffer: its buffer group, and the shared slot both of its
// halves name.
template <typename S>
struct OutputGroup;

// One sensing algorithm per sensor kind S, generated from S's declarations
// (sensor.hpp) rather than written out per kind -- so its field enum and
// Traits live in SensingFields<S> rather than at namespace scope. Inputs
// come last: input i is static_cast<Field>(kFirstInput + i), one per
// S::kInputCount.
//
// Only the output is double-buffered: it's the one thing handed to the
// user. Everything else is ordinary sim memory the Read pass mutates in
// place.
template <SensorLike S>
struct SensingFields {
  enum class Field : std::uint8_t {
    kConfig,     // S itself, one per instance
    kClock,      // the instance's SensorClock
    kState,      // S::State
    kOutput,     // the reading the Read pass writes
    kPublished,  // the last complete reading
    kFirstInput,

    kCount = kFirstInput + S::kInputCount
  };
  static_assert(engine::FieldEnumLike<Field>);

  static constexpr Field Input(std::size_t i) {
    return static_cast<Field>(static_cast<std::size_t>(Field::kFirstInput) + i);
  }

  template <Field F>
  struct Traits;

  template <Field F>
    requires(F == Field::kConfig)
  struct Traits<F> {
    using Assembler = engine::LeafAssembler<S>;
    using Ordering = SensingOrdering<S>;
    using Layout = engine::topology::PlanarLayout;
  };

  template <Field F>
    requires(F == Field::kClock)
  struct Traits<F> {
    using Assembler = engine::LeafAssembler<SensorClock>;
    using Ordering = SensingOrdering<S>;
    using Layout = engine::topology::PlanarLayout;
  };

  template <Field F>
    requires(F == Field::kState)
  struct Traits<F> {
    using Assembler = engine::LeafAssembler<typename S::State>;
    using Ordering = SensingOrdering<S>;
    using Layout = engine::topology::PlanarLayout;
  };

  // The sim's side of the output buffer: the half this tick's reading goes
  // into. ReadStep flips the group once the pass is done, so it becomes
  // kPublished.
  template <Field F>
    requires(F == Field::kOutput)
  struct Traits<F> {
    using Assembler = engine::LeafAssembler<Reading<typename S::Output>>;
    using SharedAs = OutputGroup<S>;
    using Buffering = engine::memory::
        DoubleBuffered<OutputGroup<S>, engine::memory::BufferSide::kWrite>;
    using Ordering = SensingOrdering<S>;
    using Layout = engine::topology::PlanarLayout;
  };

  // The user's side: the last complete reading, read between ticks, and by
  // the Read pass as the reading to carry forward when no sample is due.
  template <Field F>
    requires(F == Field::kPublished)
  struct Traits<F> {
    using Assembler = engine::LeafAssembler<Reading<typename S::Output>>;
    using SharedAs = OutputGroup<S>;
    using Buffering = engine::memory::
        DoubleBuffered<OutputGroup<S>, engine::memory::BufferSide::kRead>;
    using Ordering = SensingOrdering<S>;
    using Layout = engine::topology::PlanarLayout;
  };

  // Input i's source field, exactly as its own Algorithm declared it -- the
  // same SharedAs slot, so this is the source's block, not a copy -- minus
  // any kName (the source already populates it).
  template <Field F>
    requires(F >= Field::kFirstInput && F < Field::kCount)
  struct Traits<F> {
    using Source = InputSource<InputAtT<
        S,
        static_cast<std::size_t>(F) -
            static_cast<std::size_t>(Field::kFirstInput)>>;
    using Assembler = typename Source::Type::Assembler;
    using SharedAs = typename Source::Type::SharedAs;
    using Ordering = typename Source::Ordering;
    using Indexing = typename Source::Indexing;
    using Buffering = typename Source::Buffering;
    using Layout = typename Source::Layout;
  };
};

template <SensorLike S>
using SensingField = typename SensingFields<S>::Field;

template <SensorLike S>
using SensingView = engine::view::
    ViewFactory<SensingField<S>, SensingFields<S>::template Traits>;

// A list ordering's topology: each row's entry is its link row (see
// engine::topology::ListOrdering).
using SensingTopology = domain::JointTopology;
static_assert(domain::TopologyLike<SensingTopology>);

}  // namespace achilles::algorithms::sensing
