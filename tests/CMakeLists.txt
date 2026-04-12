# tests/CMakeLists.txt
find_package(GTest REQUIRED)

foreach(target test_helpers test_kinematics)
    add_executable(${target} ${target}.cpp)
    target_include_directories(${target}
        PRIVATE
            ${CMAKE_SOURCE_DIR}/src
    )
    target_link_libraries(${target}
        PRIVATE
            GTest::GTest
            GTest::Main
            achilles_lib
            Eigen3::Eigen
    )

    gtest_discover_tests(${target})
endforeach()