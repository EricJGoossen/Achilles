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

# Fetch Eigen automatically
include(FetchContent)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE       # only fetch the tagged commit, not full history
)

# Disable Eigen's own tests and install targets
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen)

option(BUILD_TESTS "Build test suite" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Recursively find all .cpp files under src/
file(GLOB_RECURSE ACHILLES_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

set(ACHILLES_LIB_SOURCES ${ACHILLES_SOURCES})
list(FILTER ACHILLES_LIB_SOURCES EXCLUDE REGEX ".*/main\\.cpp$")

add_library(achilles_lib ${ACHILLES_LIB_SOURCES})

target_compile_options(achilles_lib PRIVATE
    $<$<CONFIG:Debug>:-g -O0>
    $<$<CONFIG:Release>:-O3>
)

target_include_directories(achilles_lib PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(achilles_lib PUBLIC Eigen3::Eigen)

add_executable(achilles "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
target_link_libraries(achilles PRIVATE achilles_lib)