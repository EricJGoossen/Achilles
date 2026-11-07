#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

#include "algorithms/attachment/attachment_data.hpp"
#include "algorithms/attachment_kinds/attachment_kinds.hpp"
#include "algorithms/conventions.hpp"
#include "domain/joint_data.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/topology/orderings.hpp"
#include "util/tmp.hpp"

namespace achilles::interface {

// One sensor or actuator instance of kind K, found once (Simulation::Sensor,
// Simulation::Actuator) and kept: the way anything outside the sim talks to
// one. Cheap enough to copy -- copies share the instance -- and valid until the
// Simulation that made it is destroyed or re-Init()ed. Every instance comes
// from the .arow.
//
// A command waits in the instance's row for the next Step(), which runs its
// kind and puts the effort on the joint before anything integrates. What a
// command does after that is its kind's: a truth actuator's effort lasts the
// one tick, so it must be Apply()ed again before every Step(), and a motor
// holds its last command.
template <domain::attachment::AttachmentLike K>
class AttachmentHandle {
  using View = algorithms::attachment::AttachmentView;
  using Field = algorithms::attachment::AttachmentField;
  template <typename Kind>
  using KindState = algorithms::attachment::KindState<Kind>;
  template <typename Kind>
  using KindCommand = algorithms::attachment::KindCommand<Kind>;
  template <typename Kind>
  using KindOutput = algorithms::attachment::KindOutput<Kind>;

 public:
  // The `kind:` an .arow declares one under.
  static constexpr std::string_view kListName = K::kListName;

  // Checks `attachment` is a well-formed K -- declared under K's key, with
  // settings K's FromConfig accepts, a setting it doesn't read rejected so a
  // typo doesn't silently do nothing, and for an actuator an axis its joint can
  // move along -- and writes it into its row: `entry` of K's list, on instance
  // `instance` of its archetype, with `stream` as its random stream. Throws
  // std::invalid_argument naming the attachment, whatever was wrong with it.
  template <engine::pass::SimContextLike SimStateT>
  static void Configure(
      const SimStateT& sim_state,
      const domain::JointAttachment& attachment,
      std::size_t instance,
      std::size_t entry,
      std::uint64_t stream
  ) {
    domain::RequireKey(attachment, K::kKey);
    try {
      domain::attachment::AttachmentConfig config(attachment.config);
      K kind = K::FromConfig(config);
      config.Finish();
      const View view = sim_state.template ViewFor<View>();
      if constexpr (domain::attachment::ActuatorLike<K>) {
        using algorithms::attachment::kInputFieldOf;
        using algorithms::attachment::SubspaceInput;
        kind.CheckAxis(view.template Load<
                       kInputFieldOf<SubspaceInput>,
                       algorithms::ScalarOperationT>(
            sim_state
                .template LayoutFor<engine::topology::TopologicalOrdering>()
                .ToSorted(instance, attachment.joint)
        ));
        kind.NormalizeAxis();
      }
      kind.SetStream(stream);
      view.template Store<Field::kConfig, K>(RowOf(sim_state, entry), kind);
    } catch (const std::invalid_argument& e) {
      throw std::invalid_argument(
          std::string(K::kKey) + " '" + attachment.name + "': " + e.what()
      );
    }
  }

  // The handle for `entry` of kind K's list.
  template <engine::pass::SimContextLike SimStateT>
  static AttachmentHandle Find(const SimStateT& sim_state, std::size_t entry) {
    return AttachmentHandle(
        sim_state.template ViewFor<View>(), RowOf(sim_state, entry)
    );
  }

  // Its latest published output: its value, when it last updated, and how many
  // updates it has made. Until its first update, on the first Step(), the
  // sequence is 0 and the value is what its kind starts from (see
  // algorithms::attachment::InitOp). An actuator's value carries the effort the
  // joint got, error included.
  auto Read() const {
    return view_.template Load<Field::kPublished, KindOutput<K>>(row_).value;
  }

  // Its current configuration.
  K Config() const { return view_.template Load<Field::kConfig, K>(row_); }

  // Replaces its configuration and starts it over, as Init did: its state is
  // re-seeded by its Initialize, and its command and output cleared, so it
  // updates again on the next tick. Its random stream carries on. For an
  // actuator, `config`'s axis must be unit length and one its joint can move
  // along: only Configure checks that.
  void SetConfig(K config) const {
    config.SetStream(Config().Stream());
    view_.template Store<Field::kConfig, K>(row_, config);

    KindState<K> state{};
    domain::attachment::InitializeState(config, &state.value);
    view_.template Store<Field::kState, KindState<K>>(row_, state);
    view_.template Store<Field::kCommand, KindCommand<K>>(
        row_, KindCommand<K>{}
    );
    view_.template Store<Field::kOutput, KindOutput<K>>(row_, KindOutput<K>{});
    view_.template Store<Field::kPublished, KindOutput<K>>(
        row_, KindOutput<K>{}
    );
  }

  // Replaces the command the next tick reads.
  void Command(const typename K::Command& command) const
    requires domain::attachment::ActuatorLike<K>
  {
    view_.template Store<Field::kCommand, KindCommand<K>>(
        row_, KindCommand<K>{command}
    );
  }

  // Adds `effort` (N for a linear actuator, N*m for an angular one) to what
  // the next tick applies, for a kind taking one-tick efforts.
  void Apply(algorithms::ScalarOperationT effort) const
    requires std::same_as<typename K::Command, domain::actuation::EffortCommand>
  {
    auto command = view_.template Load<Field::kCommand, KindCommand<K>>(row_);
    command.value.effort += effort;
    view_.template Store<Field::kCommand, KindCommand<K>>(row_, command);
  }

 private:
  AttachmentHandle(View view, std::size_t row) : view_(view), row_(row) {}

  template <typename SimStateT>
  static std::size_t RowOf(const SimStateT& sim_state, std::size_t entry) {
    using algorithms::attachment::AttachmentKinds;
    using algorithms::attachment::AttachmentOrdering;
    return sim_state.template LayoutFor<AttachmentOrdering>().ToSorted(
        util::kIndexOfV<K, AttachmentKinds>, entry
    );
  }

  View view_;
  std::size_t row_;  // in the attachment rows
};

// The same handle, by what it's a handle to.
template <domain::attachment::SensorLike K>
using SensorHandle = AttachmentHandle<K>;
template <domain::attachment::ActuatorLike K>
using ActuatorHandle = AttachmentHandle<K>;

using LinearActuator = ActuatorHandle<domain::actuation::truth::Force>;
using AngularActuator = ActuatorHandle<domain::actuation::truth::Torque>;

}  // namespace achilles::interface
