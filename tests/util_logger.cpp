#include <gtest/gtest.h>

#include <string_view>

#include "util/csv_logger.hpp"
#include "util/logger.hpp"

using achilles::util::CsvLogger;
using achilles::util::LoggerLike;
using achilles::util::NullLogger;

namespace {

// Exactly what LoggerLike requires. Nothing else.
struct LoggerArchetype {
  void BeginRecord();
  void Write(std::string_view text);
  void Write(float value);
  void Write(double value);
  void EndRecord();
};
static_assert(LoggerLike<LoggerArchetype>);

// Missing a member -> not a logger.
struct NoEndRecord {
  void BeginRecord();
  void Write(std::string_view text);
  void Write(float value);
  void Write(double value);
};
static_assert(!LoggerLike<NoEndRecord>);

}  // namespace

static_assert(LoggerLike<NullLogger>);
static_assert(LoggerLike<CsvLogger>);

// Everything above is compile-time; this keeps the file a runnable test
// target like every other tests/*.cpp.
TEST(Logger, ConceptChecksHold) { SUCCEED(); }
