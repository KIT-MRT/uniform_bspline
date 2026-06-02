find_package(pybind11 REQUIRED)

pybind11_add_module(uniform_bspline_pyext
    uniform_bspline_py.cpp
)

# Keep the importable module name as 'uniform_bspline' (avoids clash with the
# INTERFACE library target of the same name defined in the top-level CMakeLists.txt)
set_target_properties(uniform_bspline_pyext PROPERTIES OUTPUT_NAME uniform_bspline)

target_link_libraries(uniform_bspline_pyext
    PRIVATE
        uniform_bspline::uniform_bspline
)

# Install the extension module alongside the Python package.
install(TARGETS uniform_bspline_pyext
    LIBRARY DESTINATION ${SKBUILD_PLATLIB_DIR}
)
