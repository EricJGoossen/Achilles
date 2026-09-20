#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
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
  ASSERT_TRUE(logger.has_value());

  logger->LogRow(std::array<float, 3>{0.0F, 1.5F, 3.0F});
  logger->LogRow(std::array<float, 3>{1.0F, 2.5F, 5.0F});

  std::string contents = ReadFile(path);
  EXPECT_EQ(contents, "time,kinetic,total\n0,1.5,3\n1,2.5,5\n");
}
