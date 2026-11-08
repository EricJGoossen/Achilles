# A minimal project that uses Achilles the way another repo does (e.g.
# achilles-hw-emulator): add_subdirectory, without the GPU, two translation
# units that both include interface/simulation.hpp. It catches what Achilles's
# own build can't see from inside -- a header defining a non-inline
# function (a duplicate symbol at link), a setting that leaks into the
# including project, a dependency on the GPU.
#
#   cmake -S tests/consumer -B build-consumer -G Ninja
#   cmake --build build-consumer && ./build-consumer/consumer
#
# Achilles's own ctest runs it too (consumer_no_gpu, consumer_gpu in
# tests/CMakeLists.txt); -DACHILLES_WITH_GPU=ON links the camera's GPU
# backend as well.
cmake_minimum_required(VERSION 3.21)
project(achilles_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ACHILLES_WITH_GPU "Render Achilles's camera sensors on the GPU" OFF)
add_subdirectory(../.. achilles EXCLUDE_FROM_ALL)

if (TARGET achilles OR TARGET maze OR TARGET algorithms_integrators)
    message(FATAL_ERROR "Achilles built its examples or tests inside another project")
endif ()

add_executable(consumer main.cpp other.cpp)
target_link_libraries(consumer PRIVATE achilles::achilles)
