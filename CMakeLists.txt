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

# For single-config generators (e.g. MinGW Makefiles), default to Debug so
# generated binaries include symbols and debugger breakpoints bind correctly.
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif ()

# Resolve Eigen for both compiler and IntelliSense include paths.
find_package(Eigen3 QUIET NO_MODULE)
if (NOT Eigen3_FOUND)
    find_path(EIGEN3_INCLUDE_DIR Eigen/Dense
        PATH_SUFFIXES eigen3
    )

    if (NOT EIGEN3_INCLUDE_DIR)
        message(FATAL_ERROR "Eigen3 not found. Install Eigen and/or set EIGEN3_INCLUDE_DIR.")
    endif ()
endif ()

# Recursively find all .cpp files under src/
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_executable(achilles ${SOURCES})

target_compile_options(achilles PRIVATE
    $<$<CONFIG:Debug>:-g -O0>
    $<$<CONFIG:Release>:-O3>
)

target_include_directories(achilles PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

if (Eigen3_FOUND)
    target_link_libraries(achilles PRIVATE Eigen3::Eigen)
else ()
    target_include_directories(achilles PRIVATE "${EIGEN3_INCLUDE_DIR}")
endif ()