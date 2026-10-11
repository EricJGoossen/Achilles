#pragma once

#include <fstream>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace achilles::util {

// A minimal, generic per-tick scalar logger: construct with the column
// names a caller intends to log, then call LogRow once per tick with
// exactly that many values. Writes plain CSV so any offline tool (a
// spreadsheet, scripts/plot.py, ...) can read it back -- this class knows
// nothing about what the columns mean (energy, joint angles, whatever else
// a caller wants graphed later), which is the point: one logger backs any
// time series, not just system energy.
class CsvLogger {
 public:
  // std::nullopt if `path` couldn't be opened for writing -- the same
  // "report failure via return value, let the caller decide whether that's
  // fatal" shape as interface::Simulation::Load*. `precision` is the
  // significant digits each value is written with: the default round-trips
  // any double exactly; a caller whose values are really floats passes
  // std::numeric_limits<float>::max_digits10 to get the same exactness
  // without double-width noise.
  static std::optional<CsvLogger> Open(
      const std::string& path,
      std::vector<std::string> columns,
      int precision = std::numeric_limits<double>::max_digits10
  );

  CsvLogger(const CsvLogger&) = delete;
  CsvLogger& operator=(const CsvLogger&) = delete;
  CsvLogger(CsvLogger&&) = default;
  CsvLogger& operator=(CsvLogger&&) = default;
  ~CsvLogger() = default;

  // values.size() must equal the column count Open() was given.
  void LogRow(std::span<const double> values);

 private:
  CsvLogger(std::ofstream file, std::size_t num_columns);

  std::ofstream file_;
  std::size_t num_columns_;
};

}  // namespace achilles::util
