cmake_minimum_required(VERSION 3.21)
project(achilles LANGUAGES C CXX)

# Achilles builds both on its own (its examples, tests and lint targets)
# and as a library inside another project (add_subdirectory or
# FetchContent, e.g. achilles-hw-emulator). Everything that would reach
# into the including project -- the build type, test switches, compiler
# launchers -- is set only when Achilles is the top-level project.

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ACHILLES_BUILD_EXAMPLES "Build the examples and the achilles runner" ${PROJECT_IS_TOP_LEVEL})
# BUILD_TESTS is the old name, still honoured.
if (DEFINED BUILD_TESTS)
    set(_achilles_tests_default ${BUILD_TESTS})
else ()
    set(_achilles_tests_default ${PROJECT_IS_TOP_LEVEL})
endif ()
option(ACHILLES_BUILD_TESTS "Build the test suite" ${_achilles_tests_default})
# The GPU backs camera sensors (util/gl/gpu.hpp): scenes are rendered
# headless, through EGL, with no window or display. Off builds no OpenGL at
# all; a scene with a camera then fails its first sample, and every other
# scene is unaffected.
option(ACHILLES_WITH_GPU "Render camera sensors on the GPU (OpenGL + EGL)" ON)
# The sensor display (display/): one window, through GLFW and Dear ImGui,
# with a panel per display showing a sensor's readings or driving an
# actuator. Outside the sim -- nothing under domain/, engine/, algorithms/,
# interface/ or util/ includes it -- and built as its own library
# (achilles::display), so a project including Achilles gets it only if it
# asks: off there by default, and nothing is fetched for it.
option(ACHILLES_WITH_DISPLAY "Build the sensor display (GLFW + Dear ImGui)" ${PROJECT_IS_TOP_LEVEL})
if (ACHILLES_WITH_DISPLAY AND NOT ACHILLES_WITH_GPU)
    message(FATAL_ERROR "ACHILLES_WITH_DISPLAY needs ACHILLES_WITH_GPU (it draws with OpenGL)")
endif ()

if (PROJECT_IS_TOP_LEVEL)
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

    # Debug keeps its asserts and debug info but is still optimized: the
    # engine's layers (views, cursors, assemblers, SIMD wrappers) are meant
    # to inline away, and at -O0 they don't, which leaves a sim far too slow
    # to debug in real time. -O1 gets nearly all of Release's speed; a
    # debugger may show some locals as optimized out. Turn this off for a
    # build to step through line by line, at far below real time -- in its
    # own build directory, since it recompiles everything.
    option(ACHILLES_OPTIMIZE_DEBUG "Optimize Debug builds (-O1); off is -O0" ON)
    if (ACHILLES_OPTIMIZE_DEBUG)
        set(_achilles_debug_flags "-g -O1 -fno-omit-frame-pointer")
    else ()
        set(_achilles_debug_flags "-g -O0 -fno-omit-frame-pointer")
    endif ()
    set(CMAKE_C_FLAGS_DEBUG "${_achilles_debug_flags}")
    set(CMAKE_CXX_FLAGS_DEBUG "${_achilles_debug_flags}")
endif ()

include(FetchContent)
# Keeps every fetched dependency below from building its own test suite --
# a normal variable, not a cache entry, so it ends with this directory and
# never switches testing off for a project that includes Achilles. The
# policy makes the dependencies' own option()s honour it (yaml-cpp's
# older cmake_minimum_required would otherwise clear it).
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(BUILD_TESTING OFF)

# xsimd (header-only) backs the SIMD activation masks
# (domain/math/activation_mask.hpp). The devcontainer installs 13.0.0;
# anywhere else it's fetched.
find_package(xsimd 13 QUIET)
if (NOT xsimd_FOUND)
    FetchContent_Declare(
        xsimd
        GIT_REPOSITORY https://github.com/xtensor-stack/xsimd.git
        GIT_TAG 13.0.0
        GIT_SHALLOW TRUE
    )
    # xsimd's own test switch shares Achilles's old BUILD_TESTS name; a
    # normal variable shadows any cached one while xsimd configures.
    set(BUILD_TESTS OFF)
    FetchContent_MakeAvailable(xsimd)
    unset(BUILD_TESTS)
endif ()

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

# OpenGL through EGL (libglvnd's libOpenGL + libEGL, no GLX): the camera
# sensors render on it headless (src/util/gl/gpu_egl.cpp). On Linux
# the dev headers come with libgl1-mesa-dev (see .devcontainer/Dockerfile);
# ACHILLES_WITH_GPU=OFF skips it altogether.
if (ACHILLES_WITH_GPU)
    find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
endif ()

# Which build this is, as constants code branches on with if constexpr
# (achilles/build_config.hpp) -- never a macro: no header changes shape
# with the build. What differs is only which sources below are compiled.
set(ACHILLES_CONFIG_WITH_GPU false)
if (ACHILLES_WITH_GPU)
    set(ACHILLES_CONFIG_WITH_GPU true)
endif ()
set(ACHILLES_CONFIG_WITH_DISPLAY false)
if (ACHILLES_WITH_DISPLAY)
    set(ACHILLES_CONFIG_WITH_DISPLAY true)
endif ()
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/include/achilles/build_config.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/achilles/build_config.hpp"
    @ONLY
)

file(GLOB_RECURSE ACHILLES_LIB_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
list(FILTER ACHILLES_LIB_SOURCES EXCLUDE REGEX ".*/main\\.cpp$")
# The display is its own library (below), never part of the sim's.
list(FILTER ACHILLES_LIB_SOURCES EXCLUDE REGEX ".*/src/display/.*")
# The GL helpers and the camera's GPU backend with the GPU; only its
# stand-in (gpu_none.cpp) without.
if (ACHILLES_WITH_GPU)
    list(FILTER ACHILLES_LIB_SOURCES EXCLUDE REGEX ".*/gpu_none\\.cpp$")
else ()
    list(FILTER ACHILLES_LIB_SOURCES EXCLUDE REGEX ".*/src/util/gl/.*")
    list(APPEND ACHILLES_LIB_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/util/gl/gpu_none.cpp"
    )
endif ()

add_library(achilles_lib ${ACHILLES_LIB_SOURCES})
# The name a consumer links: target_link_libraries(app PRIVATE achilles::achilles).
add_library(achilles::achilles ALIAS achilles_lib)

target_include_directories(achilles_lib PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_BINARY_DIR}/generated"
)
target_link_libraries(achilles_lib PUBLIC yaml-cpp::yaml-cpp xsimd)
if (ACHILLES_WITH_GPU)
    target_link_libraries(achilles_lib PUBLIC OpenGL::OpenGL OpenGL::EGL)
endif ()

if (ACHILLES_WITH_DISPLAY)
    # GLFW makes the window, on X11 only (the devcontainer has no Wayland
    # headers), with an EGL context like the camera's (see
    # src/display/window/screen.cpp).
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
    set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(glfw)

    # Dear ImGui draws everything in it. It ships no CMake build of its own:
    # its sources and the two backends used (GLFW, OpenGL 3) are built here.
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.92.9b-docking
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(imgui)
    add_library(achilles_imgui STATIC
        "${imgui_SOURCE_DIR}/imgui.cpp"
        "${imgui_SOURCE_DIR}/imgui_draw.cpp"
        "${imgui_SOURCE_DIR}/imgui_tables.cpp"
        "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
    )
    target_include_directories(achilles_imgui SYSTEM PUBLIC
        "${imgui_SOURCE_DIR}"
        "${imgui_SOURCE_DIR}/backends"
    )
    target_link_libraries(achilles_imgui PUBLIC glfw OpenGL::OpenGL)

    file(GLOB_RECURSE ACHILLES_DISPLAY_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/display/*.cpp"
    )
    add_library(achilles_display ${ACHILLES_DISPLAY_SOURCES})
    add_library(achilles::display ALIAS achilles_display)
    # ImGui and GLFW stay private: no display/ header names them.
    target_link_libraries(achilles_display
        PUBLIC achilles_lib
        PRIVATE achilles_imgui glfw OpenGL::OpenGL
    )
endif ()

if (ACHILLES_BUILD_EXAMPLES)
    add_executable(achilles "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
    target_link_libraries(achilles PRIVATE achilles_lib)
    if (ACHILLES_WITH_DISPLAY)
        target_link_libraries(achilles PRIVATE achilles_display)
        # The pan-tilt camera: a camera you steer from its display.
        add_executable(pan_tilt
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/pan_tilt/pan_tilt.cpp"
        )
        target_link_libraries(pan_tilt PRIVATE achilles_display)
    endif ()

    # Examples that bring their own program (a controller, a custom loop) --
    # each is its own executable on the library, like achilles itself.
    foreach (_example two_joint_arm dominoes waypoint_robot ball_balance maze)
        add_executable(${_example}
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/${_example}/${_example}.cpp"
        )
        target_link_libraries(${_example} PRIVATE achilles_lib)
    endforeach ()
    # Each shows its camera and the sensors it steers by when there's a
    # display (examples/shared/real_time.hpp).
    if (ACHILLES_WITH_DISPLAY)
        foreach (_example two_joint_arm dominoes waypoint_robot ball_balance maze)
            target_link_libraries(${_example} PRIVATE achilles_display)
        endforeach ()
    endif ()
endif ()

if (ACHILLES_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif ()

if (NOT PROJECT_IS_TOP_LEVEL)
    return()
endif ()

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