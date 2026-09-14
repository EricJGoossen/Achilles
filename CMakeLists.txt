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
    $<$<CONFIG:Debug>:-g -O0>
    $<$<CONFIG:Release>:-O3>
)

target_include_directories(achilles_lib ${ACHILLES_LIB_TYPE}
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(achilles_lib ${ACHILLES_LIB_TYPE} Eigen3::Eigen)

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
    COMMENT "Running clang-tidy over src/ and include/"
    VERBATIM
)