cmake_minimum_required(VERSION 3.16)
project(achilles)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_CCACHE "Use ccache to speed up rebuilds" ON)
if (ENABLE_CCACHE)
    find_program(CCACHE_PROGRAM ccache)
    if (CCACHE_PROGRAM)
        set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
        set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
        message(STATUS "ccache enabled: ${CCACHE_PROGRAM}")
    endif ()
endif ()

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif ()

include(FetchContent)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
)

set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen)

# yaml-cpp backs the .arow archetype loader (engine/topology/archetype_loader.hpp)
# -- pulled in rather than hand-rolling a YAML parser.
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
    GIT_SHALLOW TRUE
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)

# GLFW backs the OpenGL visualization module (src/render/, wired into
# interface::Simulation's own headless flag -- see simulation.hpp) --
# always fetched, the same as Eigen/yaml-cpp above, since the renderer is
# now a normal part of achilles_lib rather than an opt-in module. Building
# GLFW from source needs its own X11/OpenGL dev headers (on Linux:
# libgl1-mesa-dev, libx11-dev, libxrandr-dev, libxinerama-dev,
# libxcursor-dev, libxi-dev, libxext-dev -- see .devcontainer/Dockerfile).
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
# X11 only -- Wayland support needs wayland-scanner, an extra build-time
# dependency this project doesn't otherwise carry, and the renderer has no
# Wayland-specific code of its own to lose by disabling it.
set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)
find_package(OpenGL REQUIRED)

file(GLOB_RECURSE ACHILLES_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

set(ACHILLES_LIB_SOURCES ${ACHILLES_SOURCES})
list(FILTER ACHILLES_LIB_SOURCES EXCLUDE REGEX ".*/main\\.cpp$")

if(ACHILLES_LIB_SOURCES)
    add_library(achilles_lib ${ACHILLES_LIB_SOURCES})
    set(ACHILLES_LIB_TYPE PUBLIC)
else()
    add_library(achilles_lib INTERFACE)
    set(ACHILLES_LIB_TYPE INTERFACE)
endif()

target_compile_options(achilles_lib ${ACHILLES_LIB_TYPE}
    $<$<CONFIG:Debug>:-g>
    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Release>:-O3>
)

target_include_directories(achilles_lib ${ACHILLES_LIB_TYPE}
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(achilles_lib ${ACHILLES_LIB_TYPE} Eigen3::Eigen yaml-cpp::yaml-cpp glfw OpenGL::GL)

add_executable(achilles "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
target_link_libraries(achilles PRIVATE achilles_lib)

option(BUILD_TESTS "Build test suite" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

add_custom_target(check-format
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check-format.sh"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Checking clang-format compliance (src/, include/)"
    VERBATIM
)

add_custom_target(check-tidy
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check-tidy.sh" "${CMAKE_BINARY_DIR}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Running clang-tidy over src/, include/, and tests/"
    VERBATIM
)

# Kept separate from check-tidy above: the Clang Static Analyzer
# (clang-analyzer-*) is path-sensitive symbolic execution, an order of
# magnitude more expensive per translation unit than the AST-matcher
# checks in check-tidy. Folding it in would make every routine lint run
# pay that cost. See .clang-tidy-analyzer and scripts/check-tidy-analyzer.sh.
add_custom_target(check-tidy-analyzer
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check-tidy-analyzer.sh" "${CMAKE_BINARY_DIR}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Running the clang-analyzer checks over src/, include/, and tests/"
    VERBATIM
)