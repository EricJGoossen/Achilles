#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include "support/temp_dir.hpp"
#include "util/csv_logger.hpp"

using achilles::test_support::TempDir;
using achilles::util::CsvLogger;

namespace {

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

}  // namespace

TEST(CsvLogger, OpenFailsOnAnUnwritableDirectory) {
  auto logger = CsvLogger::Open(
      "/no/such/directory/exists/energy.csv", {"time", "value"}
  );
  EXPECT_FALSE(logger.has_value());
}

TEST(CsvLogger, WritesAHeaderThenOneCommaSeparatedRowPerLogRowCall) {
  TempDir dir;
  std::filesystem::path path = dir.Write("energy.csv", "");

  auto logger = CsvLogger::Open(path.string(), {"time", "kinetic", "total"});
  if (!logger.has_value()) {
    FAIL() << "CsvLogger::Open failed";
  }

  logger->LogRow(std::array<double, 3>{0.0, 1.5, 3.0});
  logger->LogRow(std::array<double, 3>{1.0, 2.5, 5.0});

  std::string contents = ReadFile(path);
  EXPECT_EQ(contents, "time,kinetic,total\n0,1.5,3\n1,2.5,5\n");
}

// A float value written at float's own max_digits10 reads back as exactly
// the same float -- what Simulation's field log relies on.
TEST(CsvLogger, FloatPrecisionRoundTripsFloatValuesExactly) {
  TempDir dir;
  std::filesystem::path path = dir.Write("values.csv", "");

  auto logger = CsvLogger::Open(
      path.string(), {"value"}, std::numeric_limits<float>::max_digits10
  );
  if (!logger.has_value()) {
    FAIL() << "CsvLogger::Open failed";
  }
  constexpr float kValue = 0.1F;
  logger->LogRow(std::array<double, 1>{kValue});

  std::istringstream in(ReadFile(path));
  std::string header;
  std::string value;
  std::getline(in, header);
  std::getline(in, value);
  EXPECT_EQ(std::stof(value), kValue);
  EXPECT_EQ(value, "0.100000001");
}
