#include <gtest/gtest.h>

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
  EXPECT_FALSE(CsvLogger::Open("/no/such/directory/exists/log.csv").has_value()
  );
}

// One line per record, cells comma-separated in Write order.
TEST(CsvLogger, WritesOneLinePerRecord) {
  TempDir dir;
  std::filesystem::path path = dir.Write("log.csv", "");
  auto logger = CsvLogger::Open(path.string());
  if (!logger.has_value()) {
    FAIL() << "CsvLogger::Open failed";
  }

  logger->BeginRecord();
  logger->Write("t");
  logger->Write(0.5);
  logger->EndRecord();
  logger->BeginRecord();
  logger->Write("q");
  logger->Write(1.5F);
  logger->Write(-2.0F);
  logger->EndRecord();

  EXPECT_EQ(ReadFile(path), "t,0.5\nq,1.5,-2\n");
}

// Values are written in their shortest exact form: they parse back to
// exactly what was logged, without padding digits.
TEST(CsvLogger, NumbersRoundTripExactly) {
  TempDir dir;
  std::filesystem::path path = dir.Write("log.csv", "");
  auto logger = CsvLogger::Open(path.string());
  if (!logger.has_value()) {
    FAIL() << "CsvLogger::Open failed";
  }

  constexpr float kFloat = 0.1F;
  constexpr double kDouble = 0.1;
  logger->BeginRecord();
  logger->Write(kFloat);
  logger->Write(kDouble);
  logger->Write(std::numeric_limits<float>::min());
  logger->EndRecord();

  std::istringstream in(ReadFile(path));
  std::string first;
  std::string second;
  std::string third;
  std::getline(in, first, ',');
  std::getline(in, second, ',');
  std::getline(in, third);
  EXPECT_EQ(first, "0.1");
  EXPECT_EQ(std::stof(first), kFloat);
  EXPECT_EQ(std::stod(second), kDouble);
  EXPECT_EQ(std::stof(third), std::numeric_limits<float>::min());
}
