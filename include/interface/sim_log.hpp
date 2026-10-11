#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "algorithms/conventions.hpp"
#include "domain/archetype.hpp"
#include "engine/pass/field_log.hpp"
#include "util/csv_logger.hpp"
#include "util/io.hpp"

namespace achilles::interface {

// interface::Simulation's per-tick field log (see Simulation::EnableLog):
// owns the CSV sink, the resolved engine::pass::FieldLog selection, and
// the running timestamp. Specialized on Enabled so that with
// ACHILLES_ENABLE_LOGGING=OFF the real implementation below is never
// instantiated -- the disabled specialization is an empty no-op.
template <typename ContextT, bool Enabled = algorithms::kLoggingEnabled>
class SimLog;

template <typename ContextT>
class SimLog<ContextT, false> {
 public:
  template <typename RefreshFn>
  bool Enable(
      const ContextT& /*context*/,
      std::span<const domain::Archetype> /*archetypes*/,
      const std::string& path,
      const std::vector<std::string>& /*fields*/,
      RefreshFn /*refresh*/
  ) {
    return util::Warning(
        "Simulation::EnableLog(): logging is compiled out "
        "(ACHILLES_ENABLE_LOGGING=OFF)",
        path.c_str()
    );
  }

  template <typename RefreshFn>
  void AfterStep(
      const ContextT& /*context*/,
      algorithms::ScalarOperationT /*dt*/,
      RefreshFn /*refresh*/
  ) {}
};

template <typename ContextT>
class SimLog<ContextT, true> {
  using FieldLog = engine::pass::FieldLogForT<ContextT>;

 public:
  // `refresh` re-runs ABA at the current state before each row is written,
  // so ABA-derived fields (world_transform, spatial_velocity,
  // joint_acceleration, ...) describe the state the row is timestamped
  // with rather than whichever intermediate state the integrator evaluated
  // last (see Simulation::RefreshAccelerationForCurrentState). Costs one
  // ABA pass per logged tick; dynamics are unaffected, since every
  // integrator starts its next tick with its own ABA pass.
  template <typename RefreshFn>
  bool Enable(
      const ContextT& context,
      std::span<const domain::Archetype> archetypes,
      const std::string& path,
      const std::vector<std::string>& fields,
      RefreshFn refresh
  ) {
    std::vector<std::string> unknown;
    FieldLog field_log(context, fields, unknown);
    if (!unknown.empty()) {
      return util::Warning("Unknown log field", unknown.front().c_str());
    }
    if (field_log.Empty()) {
      return util::Warning("Simulation::EnableLog(): no fields selected");
    }

    std::vector<std::string> columns{"time"};
    for (std::string& column : field_log.ColumnNames(Labeler(archetypes))) {
      columns.push_back(std::move(column));
    }
    // ScalarOperationT's own max_digits10: every logged value is exactly
    // the stored one, without printing float values at double width.
    auto sink = util::CsvLogger::Open(
        path,
        std::move(columns),
        std::numeric_limits<algorithms::ScalarOperationT>::max_digits10
    );
    if (!sink.has_value()) {
      return util::Warning("Failed to open log file", path.c_str());
    }

    std::size_t width = field_log.Width();
    State& state = state_.emplace(State{
        std::move(*sink), std::move(field_log), std::vector<double>(1 + width)
    });
    WriteRow(context, state, refresh);
    return true;
  }

  template <typename RefreshFn>
  void AfterStep(
      const ContextT& context,
      algorithms::ScalarOperationT dt,
      RefreshFn refresh
  ) {
    if (!state_.has_value()) {
      return;
    }
    State& state = *state_;
    state.time += dt;
    WriteRow(context, state, refresh);
  }

 private:
  // `time` accumulates in double regardless of ScalarOperationT so long
  // runs keep distinct timestamps.
  struct State {
    util::CsvLogger sink;
    FieldLog fields;
    std::vector<double> row;
    double time = 0.0;
  };

  template <typename RefreshFn>
  static void WriteRow(
      const ContextT& context, State& state, RefreshFn& refresh
  ) {
    refresh();
    std::span<double> row(state.row);
    row[0] = state.time;
    state.fields.Gather(context, row.subspan(1));
    state.sink.LogRow(row);
  }

  // Labels a global (instance, joint) handle "<archetype>[<copy>].<joint>"
  // ("[<copy>]" omitted for a single-copy archetype) by walking archetypes
  // in load order -- the same order instance indices are assigned in (see
  // interface/archetype_loader.cpp).
  static typename FieldLog::JointLabeler Labeler(
      std::span<const domain::Archetype> archetypes
  ) {
    return [archetypes](domain::ArchetypeJointHandle handle) {
      std::size_t offset = 0;
      for (const domain::Archetype& archetype : archetypes) {
        std::size_t copies = archetype.InstanceCount();
        if (handle.instance_index < offset + copies) {
          std::string label = archetype.Name();
          if (copies > 1) {
            label += "[" + std::to_string(handle.instance_index - offset) + "]";
          }
          return label + "." + archetype.JointName(handle.joint_index);
        }
        offset += copies;
      }
      return "instance" + std::to_string(handle.instance_index) + ".j" +
             std::to_string(handle.joint_index);
    };
  }

  std::optional<State> state_;
};

}  // namespace achilles::interface
