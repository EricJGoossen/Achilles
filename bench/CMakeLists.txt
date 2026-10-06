# Standalone benchmark build -- pulls the engine in as a subdirectory so the
# root CMakeLists.txt doesn't have to know benchmarks exist.
#
#   cmake -S bench -B build-bench -G Ninja -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-bench --target achilles_bench
#
# Pass -DACHILLES_BENCH_NATIVE=ON to build with -march=native (AVX2 on
# x86-64, so xsimd::batch<double> packs 4 lanes instead of SSE2's 2).
cmake_minimum_required(VERSION 3.16)
project(achilles_bench CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ACHILLES_BENCH_NATIVE "Compile with -march=native" OFF)
if (ACHILLES_BENCH_NATIVE)
    add_compile_options(-march=native)
endif ()

set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. achilles)

add_executable(achilles_bench achilles_bench.cpp)
target_link_libraries(achilles_bench PRIVATE achilles_lib)
