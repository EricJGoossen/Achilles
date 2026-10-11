#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <initializer_list>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/instrumentation.hpp"
#include "algorithms/log/log_data.hpp"
#include "algorithms/log/log_step.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "domain/joint_layout.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "util/logger.hpp"

using achilles::algorithms::Instrumentation;
using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::SimConfig;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::algorithms::log::FindLogField;
using achilles::algorithms::log::LogAlgorithm;
using achilles::algorithms::log::LogField;
using achilles::algorithms::log::LogFieldName;
using achilles::algorithms::log::LogFieldScalars;
using achilles::algorithms::log::LogSelection;
using achilles::algorithms::log::LogStep;
using achilles::algorithms::log::WriteLogSchema;
using achilles::domain::Archetype;
using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::engine::memory::SimAllocator;
using achilles::engine::topology::TopologicalOrdering;

namespace {

// Keeps every record it's handed, cell by cell.
struct RecordingLogger {
  using Cell = std::variant<std::string, double>;

  void BeginRecord() { records.emplace_back(); }
  void Write(std::string_view text) {
    records.back().emplace_back(std::string(text));
  }
  void Write(float value) { records.back().emplace_back(double{value}); }
  void Write(double value) { records.back().emplace_back(value); }
  void EndRecord() {}

  std::vector<std::vector<Cell>> records;
};
static_assert(achilles::util::LoggerLike<RecordingLogger>);

using Sim = SimAllocator<ABAAlgorithm, LogAlgorithm>;

// One archetype, `instances` single-joint instances.
Sim MakeSim(std::size_t instances) {
  std::array<Archetype, 1> archetypes = {Archetype(
      "body",
      {ArchetypeTreeStructure::kNoParent},
      std::vector<ArchetypeJointHandle>(instances, ArchetypeJointHandle{0, 0}),
      true,
      {}
  )};
  return Sim(archetypes);
}

// Instance i's joint position: translation (i, 10i, 100i), identity
// rotation.
void StorePositions(const Sim& sim, std::size_t instances) {
  auto view = sim.ViewFor<ABAAlgorithm>();
  const auto& rows = sim.LayoutFor<TopologicalOrdering>();
  for (std::size_t i = 0; i < instances; ++i) {
    auto x = static_cast<ScalarOperationT>(i);
    view.Store<ABAField::kJointPosition, ScalarOperationT>(
        rows.ToSorted({i, 0}),
        achilles::domain::spatial::Transform<ScalarOperationT>(
            x, 10 * x, 100 * x, 1, 0, 0, 0
        )
    );
  }
}

LogSelection Select(std::initializer_list<LogField> fields) {
  LogSelection selection;
  for (LogField field : fields) {
    selection.set(static_cast<std::size_t>(field));
  }
  return selection;
}

void RunLogStep(
    const Sim& sim, RecordingLogger& logger, LogSelection selection
) {
  LogStep::Step(
      sim.SimContext(),
      SimConfig{},
      Instrumentation<RecordingLogger>{&logger, selection, 1.5},
      ScalarOperationT{0}
  );
}

}  // namespace

TEST(LogFieldNames, EveryFieldFindsItselfByName) {
  for (std::size_t i = 0; i < achilles::algorithms::log::kNumLogFields; ++i) {
    auto field = static_cast<LogField>(i);
    EXPECT_EQ(FindLogField(LogFieldName(field)), field);
  }
  EXPECT_FALSE(FindLogField("no_such_field").has_value());
  EXPECT_EQ(LogFieldScalars(LogField::kJointPosition), 7U);
  EXPECT_EQ(LogFieldScalars(LogField::kJointVelocity), 6U);
}

// A "t" record, then one record holding every real joint's scalars --
// sorted row order, padding skipped.
TEST(LogStep, WritesTimeThenOneRecordPerSelectedField) {
  if (!achilles::algorithms::kLoggingEnabled) {
    GTEST_SKIP() << "logging compiled out: LogStep writes nothing";
  }
  constexpr std::size_t kInstances = 2;
  Sim sim = MakeSim(kInstances);
  StorePositions(sim, kInstances);
  RecordingLogger logger;

  RunLogStep(sim, logger, Select({LogField::kJointPosition}));

  ASSERT_EQ(logger.records.size(), 2U);
  EXPECT_EQ(
      logger.records[0],
      (std::vector<RecordingLogger::Cell>{std::string("t"), 1.5})
  );
  const auto& record = logger.records[1];
  ASSERT_EQ(record.size(), 1U + (kInstances * 7));
  EXPECT_EQ(std::get<std::string>(record[0]), "joint_position");
  const auto& rows = sim.LayoutFor<TopologicalOrdering>();
  // Instance 1 comes second in sorted order; its translation is (1, 10, 100).
  ASSERT_LT(rows.ToSorted({0, 0}), rows.ToSorted({1, 0}));
  EXPECT_EQ(std::get<double>(record[1 + 7 + 0]), 1.0);
  EXPECT_EQ(std::get<double>(record[1 + 7 + 1]), 10.0);
  EXPECT_EQ(std::get<double>(record[1 + 7 + 2]), 100.0);
}

// Selected fields come out in LogField order, whatever order they were
// selected in.
TEST(LogStep, WritesSelectedFieldsInLogFieldOrder) {
  if (!achilles::algorithms::kLoggingEnabled) {
    GTEST_SKIP() << "logging compiled out: LogStep writes nothing";
  }
  Sim sim = MakeSim(1);
  RecordingLogger logger;

  RunLogStep(
      sim, logger, Select({LogField::kWorldTransform, LogField::kJointVelocity})
  );

  ASSERT_EQ(logger.records.size(), 3U);
  EXPECT_EQ(std::get<std::string>(logger.records[1][0]), "joint_velocity");
  EXPECT_EQ(std::get<std::string>(logger.records[2][0]), "world_transform");
}

TEST(LogStep, WritesNothingWithoutASelectionOrALogger) {
  Sim sim = MakeSim(1);
  RecordingLogger logger;

  RunLogStep(sim, logger, LogSelection{});
  LogStep::Step(
      sim.SimContext(),
      SimConfig{},
      Instrumentation<RecordingLogger>{
          nullptr, Select({LogField::kJointPosition}), 0.0
      },
      ScalarOperationT{0}
  );

  EXPECT_TRUE(logger.records.empty());
}

// "joints" names each real joint in the same order records list them;
// then one "field" record per LogField.
TEST(WriteLogSchema, NamesJointsThenEveryField) {
  Sim sim = MakeSim(2);
  RecordingLogger logger;

  WriteLogSchema(sim.SimContext(), logger, [](ArchetypeJointHandle handle) {
    return "i" + std::to_string(handle.instance_index);
  });

  ASSERT_EQ(
      logger.records.size(), 1U + achilles::algorithms::log::kNumLogFields
  );
  EXPECT_EQ(
      logger.records[0],
      (std::vector<RecordingLogger::Cell>{
          std::string("joints"), std::string("i0"), std::string("i1")
      })
  );
  EXPECT_EQ(
      logger.records[1],
      (std::vector<RecordingLogger::Cell>{
          std::string("field"), std::string("joint_position"), std::string("7")
      })
  );
}
