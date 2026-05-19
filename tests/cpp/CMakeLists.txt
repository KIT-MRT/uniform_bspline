find_package(GTest REQUIRED)

set(TEST_SOURCES
    control_points_container.cpp
    control_points_trait.cpp
    examples.cpp
    multi_array.cpp
    uniform_bspline.cpp
)

add_executable(uniform_bspline_tests ${TEST_SOURCES})

target_link_libraries(uniform_bspline_tests
    PRIVATE
        uniform_bspline::uniform_bspline
        GTest::GTest
        GTest::Main
)

# tests/cpp/ is on the include path so sources can use:
#   #include "tests/cpp/test_utility.hpp"  (bare test-internal headers)
target_include_directories(uniform_bspline_tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}
)

# Pass project source root so test_utility.hpp can resolve test_data paths.
target_compile_definitions(uniform_bspline_tests
    PRIVATE
        UBS_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

include(GoogleTest)
gtest_discover_tests(uniform_bspline_tests)
