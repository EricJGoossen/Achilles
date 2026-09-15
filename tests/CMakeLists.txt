# tests/CMakeLists.txt
#
# GoogleTest is fetched from source (mirroring how the root CMakeLists.txt
# pulls in Eigen) rather than found via find_package(GTest). Ubuntu's
# libgtest-dev package only ships headers/source, not prebuilt libraries,
# so find_package(GTest) fails on a stock CI runner -- fetching source
# avoids depending on the distro package at all.
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
    GIT_SHALLOW TRUE
)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

# One executable per top-level tests/*.cpp file, named after its stem --
# tests/domain_math_matrix.cpp becomes the `domain_math_matrix` target.
# CONFIGURE_DEPENDS (same as the root CMakeLists.txt's own source glob)
# makes CMake re-scan this directory on the next build after a file is
# added or removed, so a new test file needs no CMakeLists.txt edit.
# Non-recursive: tests/support/ holds shared headers only, never its own
# test executables.
file(GLOB ACHILLES_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

foreach(_test_src ${ACHILLES_TEST_SOURCES})
    get_filename_component(target ${_test_src} NAME_WE)

    add_executable(${target} ${_test_src})
    target_include_directories(${target}
        PRIVATE
            ${CMAKE_SOURCE_DIR}/src
            ${CMAKE_SOURCE_DIR}/include
    )
    target_link_libraries(${target}
        PRIVATE
            gtest
            gtest_main
            achilles_lib
            Eigen3::Eigen
    )

    gtest_discover_tests(${target})
endforeach()
