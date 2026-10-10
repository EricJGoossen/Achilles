#include "util/csv_logger.hpp"

#include <cassert>
#include <utility>

namespace achilles::util {

std::optional<CsvLogger> CsvLogger::Open(
    const std::string& path, std::vector<std::string> columns
) {
  std::ofstream file(path);
  if (!file.is_open()) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i < columns.size(); ++i) {
    if (i > 0) {
      file << ',';
    }
    file << columns[i];
  }
  file << '\n';
  file.flush();
  return CsvLogger(std::move(file), columns.size());
}

CsvLogger::CsvLogger(std::ofstream file, std::size_t num_columns)
    : file_(std::move(file)), num_columns_(num_columns) {}

void CsvLogger::LogRow(std::span<const float> values) {
  assert(
      values.size() == num_columns_ &&
      "CsvLogger::LogRow given a different number of values than Open() was "
      "given columns"
  );
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      file_ << ',';
    }
    file_ << values[i];
  }
  file_ << '\n';
  // Flushed every row, not just on close -- a caller reading this file
  // while the sim is still running (or a crash mid-run) should still see
  // every row logged so far, not whatever libstdc++'s ofstream buffer
  // happened to still be holding.
  file_.flush();
}

}  // namespace achilles::util
