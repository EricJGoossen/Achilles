# tests/CMakeLists.txt
#
# GoogleTest is fetched from source (mirroring how the root CMakeLists.txt
# pulls in yaml-cpp) rather than found via find_package(GTest). Ubuntu's
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
            ${PROJECT_SOURCE_DIR}/src
            ${PROJECT_SOURCE_DIR}/include
    )
    target_link_libraries(${target}
        PRIVATE
            gtest
            gtest_main
            achilles_lib
    )
    # Lets a test load the real files under examples/ (examples_*.cpp)
    # rather than a copy of them.
    target_compile_definitions(${target}
        PRIVATE ACHILLES_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    )

    gtest_discover_tests(${target})
endforeach()

# tests/consumer, configured, built and run as its own project -- Achilles
# included the way another repo includes it -- once without the GPU and,
# when this build has it, once with. Same generator and compilers as this build; the nested build
# reuses the dependency sources this one already fetched rather than
# fetching them again. Each keeps its own build directory, so a rerun is
# incremental.
set(_consumer_options
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
)
if (CMAKE_CXX_COMPILER_LAUNCHER)
    list(APPEND _consumer_options
        -DCMAKE_C_COMPILER_LAUNCHER=${CMAKE_C_COMPILER_LAUNCHER}
        -DCMAKE_CXX_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER}
    )
endif ()
foreach (_dep yaml-cpp xsimd)
    string(TOUPPER ${_dep} _dep_upper)
    if (${_dep}_SOURCE_DIR)
        list(APPEND _consumer_options
            -DFETCHCONTENT_SOURCE_DIR_${_dep_upper}=${${_dep}_SOURCE_DIR}
        )
    endif ()
endforeach ()

set(_consumer_variants no_gpu OFF)
if (ACHILLES_WITH_GPU)
    list(APPEND _consumer_variants gpu ON)
endif ()
while (_consumer_variants)
    list(POP_FRONT _consumer_variants _name _gpu)
    set(_dir ${CMAKE_CURRENT_BINARY_DIR}/consumer_${_name})
    add_test(NAME consumer_${_name}
        COMMAND ${CMAKE_CTEST_COMMAND}
            --build-and-test
                ${CMAKE_CURRENT_SOURCE_DIR}/consumer
                ${_dir}
            --build-generator ${CMAKE_GENERATOR}
            --build-makeprogram ${CMAKE_MAKE_PROGRAM}
            --build-config $<CONFIG>
            --build-options ${_consumer_options} -DACHILLES_WITH_GPU=${_gpu}
            --test-command ${_dir}/consumer
    )
    # Run alone: each is a whole nested build, at the build tool's own
    # parallelism, so two at once (or beside other tests, under ctest -j)
    # can run a small machine out of memory.
    set_tests_properties(consumer_${_name} PROPERTIES
        LABELS consumer
        TIMEOUT 900
        RUN_SERIAL TRUE
    )
endwhile ()
