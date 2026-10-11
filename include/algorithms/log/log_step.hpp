#pragma once

#include <cstddef>
#include <string>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/log/log_data.hpp"
#include "algorithms/log/log_ops.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "domain/joint_layout.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "util/logger.hpp"

namespace achilles::algorithms::log {

// Record tags. A log is a sequence of records, each starting with a tag:
//   joints,<label>,<label>,...        once, at the start: the joint each
//                                     value position belongs to
//   field,<name>,<scalars per joint>  once per LogField, at the start
//   t,<time>                          once per logged step
//   <name>,<v>,<v>,...                once per selected field per logged
//                                     step: every joint's scalars, joint-
//                                     major, in "joints" order
// The schema records describe every LogField, so fields can be selected
// or dropped mid-run without the log's shape ever changing -- a step's
// records simply include or omit that field.
inline constexpr std::string_view kJointsTag = "joints";
inline constexpr std::string_view kFieldTag = "field";
inline constexpr std::string_view kTimeTag = "t";

// Writes the schema records ("joints", then one "field" per LogField).
// `label` names one real joint (domain::ArchetypeJointHandle ->
// std::string). Joints are in sorted row order -- the same order LogStep
// writes each field's values in.
template <
    util::LoggerLike LoggerT,
    engine::pass::SimContextLike SimStateT,
    typename LabelFn>
void WriteLogSchema(
    const SimStateT& sim_state, LoggerT& logger, const LabelFn& label
) {
  const domain::JointLayout& rows =
      sim_state.template LayoutFor<engine::topology::TopologicalOrdering>();
  logger.BeginRecord();
  logger.Write(kJointsTag);
  for (std::size_t row = 0; row < rows.PaddedSize(); ++row) {
    if (!rows.IsPadding(row)) {
      logger.Write(std::string_view(label(rows.ToPhysical(row))));
    }
  }
  logger.EndRecord();

  for (std::size_t i = 0; i < kNumLogFields; ++i) {
    auto field = static_cast<LogField>(i);
    logger.BeginRecord();
    logger.Write(kFieldTag);
    logger.Write(LogFieldName(field));
    logger.Write(std::string_view(std::to_string(LogFieldScalars(field))));
    logger.EndRecord();
  }
}

// Writes the current state: a "t" record, then one record per selected
// field, each one pass of LogOp over every real joint. Registered right
// after ABA, so within one SimContext::Step it sees the state ABA just
// evaluated -- q, qd and every derived field describe the same instant.
// Does nothing without a logger or a selection, and compiles to nothing
// when ACHILLES_ENABLE_LOGGING=OFF.
struct LogStep {
  using FieldEnum = LogField;

  template <engine::pass::SimContextLike SimStateT, typename InstrumentationT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& /*sim_config*/,
      const InstrumentationT& instrumentation,
      ScalarOperationT /*dt*/
  ) {
    if constexpr (kLoggingEnabled) {
      if (instrumentation.logger == nullptr ||
          instrumentation.log_fields.none()) {
        return;
      }
      using LoggerT = typename InstrumentationT::Logger;
      using Op = LogOp<LoggerT>;
      using engine::pass::Ops;
      using engine::pass::Pass;
      using engine::pass::RealRowTraversal;

      LoggerT& logger = *instrumentation.logger;
      const auto view = sim_state.template ViewFor<LogView>();
      const domain::JointLayout& rows =
          sim_state.template LayoutFor<engine::topology::TopologicalOrdering>();

      logger.BeginRecord();
      logger.Write(kTimeTag);
      logger.Write(instrumentation.time);
      logger.EndRecord();

      for (std::size_t i = 0; i < kNumLogFields; ++i) {
        if (!instrumentation.log_fields.test(i)) {
          continue;
        }
        auto field = static_cast<LogField>(i);
        logger.BeginRecord();
        logger.Write(LogFieldName(field));
        engine::pass::Step(
            Ops<Pass<Op, RealRowTraversal>>{Op(logger, field)}, view, rows
        );
        logger.EndRecord();
      }
    }
  }
};

using LogAlgorithm = engine::Algorithm<LogField, LogFieldTraits, LogStep>;
static_assert(engine::AlgorithmLike<LogAlgorithm>);

}  // namespace achilles::algorithms::log
