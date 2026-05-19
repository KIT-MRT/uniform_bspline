cmake_minimum_required(VERSION 3.16)
project(uniform_bspline VERSION 1.1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(BUILD_PYTHON_BINDINGS  "Build Python bindings via pybind11" OFF)
option(BUILD_TESTS            "Build C++ unit tests"               OFF)
option(BUILD_DOCUMENTATION    "Build Doxygen HTML documentation"   OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost 1.40 REQUIRED)

# ---------------------------------------------------------------------------
# Library target (header-only interface)
# ---------------------------------------------------------------------------
add_library(uniform_bspline INTERFACE)
add_library(uniform_bspline::uniform_bspline ALIAS uniform_bspline)

# Both include paths are required:
#  - include/              so consumers can do #include <uniform_bspline/...>
#  - include/uniform_bspline/  so the headers can include each other with bare names
target_include_directories(uniform_bspline
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/uniform_bspline>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/uniform_bspline>
)

target_link_libraries(uniform_bspline
    INTERFACE
        Eigen3::Eigen
        Boost::boost
)

target_compile_features(uniform_bspline INTERFACE cxx_std_17)

# ---------------------------------------------------------------------------
# Python bindings
# ---------------------------------------------------------------------------
if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings/python)
endif()

# ---------------------------------------------------------------------------
# Documentation
# ---------------------------------------------------------------------------
if(BUILD_DOCUMENTATION)
    find_package(Doxygen REQUIRED)
    add_custom_target(docs
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/build/docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating Doxygen HTML documentation"
        VERBATIM
    )
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests/cpp)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS uniform_bspline
    EXPORT uniform_bsplineTargets
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT uniform_bsplineTargets
    FILE uniform_bsplineTargets.cmake
    NAMESPACE uniform_bspline::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/uniform_bspline
)

configure_package_config_file(
    cmake/uniform_bsplineConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/uniform_bsplineConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/uniform_bspline
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/uniform_bsplineConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/uniform_bsplineConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/uniform_bsplineConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/uniform_bspline
)
