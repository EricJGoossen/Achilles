#pragma once

#include <fstream>
#include <optional>
#include <string>
#include <string_view>

#include "util/logger.hpp"

namespace achilles::util {

// A LoggerLike that writes each record as one comma-separated line. Values
// are written in their shortest exact form (std::to_chars), so a reader
// parses back exactly the value that was logged. Knows nothing about what
// the records mean -- see util/logger.hpp.
class CsvLogger {
 public:
  // std::nullopt if `path` couldn't be opened for writing -- the same
  // "report failure via return value, let the caller decide whether that's
  // fatal" shape as interface::Simulation::Load*.
  static std::optional<CsvLogger> Open(const std::string& path);

  CsvLogger(const CsvLogger&) = delete;
  CsvLogger& operator=(const CsvLogger&) = delete;
  CsvLogger(CsvLogger&&) = default;
  CsvLogger& operator=(CsvLogger&&) = default;
  ~CsvLogger() = default;

  void BeginRecord();
  void Write(std::string_view text);
  void Write(float value);
  void Write(double value);
  // Ends the line and flushes, so a reader watching the file mid-run (or
  // after a crash) sees every complete record so far.
  void EndRecord();

 private:
  explicit CsvLogger(std::ofstream file);

  template <typename T>
  void WriteNumber(T value);
  void Separate();

  std::ofstream file_;
  bool first_in_record_ = true;
};
static_assert(LoggerLike<CsvLogger>);

}  // namespace achilles::util
