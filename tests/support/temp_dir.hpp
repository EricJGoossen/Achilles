#pragma once

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

namespace achilles::test_support {

// A scratch directory, unique across concurrent processes, removed on
// destruction -- for tests that need to write real files to disk (a loader
// that reads a real path, e.g. LoadArchetypes/LoadSimConfig, can't be
// exercised without one).
//
// Uniqueness needs both a per-process id (getpid()) and a per-instance
// counter, not the counter alone: ctest's gtest_discover_tests runs every
// individual TEST() as its own process invocation (`<binary>
// --gtest_filter=Suite.Case`), so a counter starting fresh at 0 in every
// process is not unique across the concurrent processes `ctest -j` spins up
// from the same binary -- two tests from one file used to collide on the
// same directory name and corrupt each other's fixture files.
class TempDir {
 public:
  TempDir() {
    static std::atomic<int> counter{0};
    path_ = std::filesystem::temp_directory_path() /
            ("achilles_test_" + std::to_string(::getpid()) + "_" +
             std::to_string(counter++));
    std::filesystem::create_directories(path_);
  }
  ~TempDir() { std::filesystem::remove_all(path_); }

  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;
  TempDir(TempDir&&) = delete;
  TempDir& operator=(TempDir&&) = delete;

  std::filesystem::path Write(
      const std::string& file_name, const std::string& content
  ) const {
    std::filesystem::path file = path_ / file_name;
    std::ofstream out(file);
    out << content;
    return file;
  }

 private:
  std::filesystem::path path_;
};

}  // namespace achilles::test_support
