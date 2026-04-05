cmake_minimum_required(VERSION 3.16)
project(achilles)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Recursively find all .cpp files under src/
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_executable(achilles ${SOURCES})

target_include_directories(achilles PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)