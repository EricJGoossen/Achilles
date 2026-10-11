#include "util/csv_logger.hpp"

#include <array>
#include <charconv>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace achilles::util {

std::optional<CsvLogger> CsvLogger::Open(const std::string& path) {
  std::ofstream file(path);
  if (!file.is_open()) {
    return std::nullopt;
  }
  return CsvLogger(std::move(file));
}

CsvLogger::CsvLogger(std::ofstream file) : file_(std::move(file)) {}

void CsvLogger::BeginRecord() { first_in_record_ = true; }

void CsvLogger::Write(std::string_view text) {
  Separate();
  file_ << text;
}

void CsvLogger::Write(float value) { WriteNumber(value); }

void CsvLogger::Write(double value) { WriteNumber(value); }

void CsvLogger::EndRecord() {
  file_ << '\n';
  file_.flush();
}

template <typename T>
void CsvLogger::WriteNumber(T value) {
  Separate();
  // Shortest representation that parses back to exactly `value`.
  std::array<char, 32> buffer{};
  auto [end, error] =
      std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
  if (error == std::errc{}) {
    file_.write(buffer.data(), end - buffer.data());
  }
}

void CsvLogger::Separate() {
  if (!first_in_record_) {
    file_ << ',';
  }
  first_in_record_ = false;
}

}  // namespace achilles::util
