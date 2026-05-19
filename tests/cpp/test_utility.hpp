#pragma once
#include <filesystem>

namespace uniform_bspline {
namespace test {
// UBS_PROJECT_SOURCE_DIR is injected as a compile definition by CMake.
const std::filesystem::path projectRootDir{UBS_PROJECT_SOURCE_DIR};
} // namespace test
} // namespace uniform_bspline
