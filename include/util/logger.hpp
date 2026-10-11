#pragma once

#include <concepts>
#include <string_view>

namespace achilles::util {

// What a logger is: something that writes the records it's handed, and
// nothing else. It never sees fields, joints or the engine -- whoever logs
// decides what goes in a record and in what order; the logger only decides
// how each record is written out (a CSV line, a binary chunk, ...).
//
// A record is BeginRecord(), then any mix of Write() calls, then
// EndRecord(). Strings are for tags and labels; floats and doubles are
// values, written exactly (a reader recovers the same value).
//
// Opening a destination is construction, not part of this contract.
template <typename L>
concept LoggerLike =
    requires(L& logger, std::string_view text, float f, double d) {
      { logger.BeginRecord() } -> std::same_as<void>;
      { logger.Write(text) } -> std::same_as<void>;
      { logger.Write(f) } -> std::same_as<void>;
      { logger.Write(d) } -> std::same_as<void>;
      { logger.EndRecord() } -> std::same_as<void>;
    };

// Discards everything. Every member is empty and inline, so logging into it
// compiles to nothing -- the logger used when ACHILLES_ENABLE_LOGGING=OFF.
struct NullLogger {
  void BeginRecord() {}
  void Write(std::string_view /*text*/) {}
  void Write(float /*value*/) {}
  void Write(double /*value*/) {}
  void EndRecord() {}
};
static_assert(LoggerLike<NullLogger>);

}  // namespace achilles::util
