# Uniform B-spline

[![CI](https://github.com/KIT-MRT/uniform_bspline/actions/workflows/ci.yml/badge.svg)](https://github.com/KIT-MRT/uniform_bspline/actions/workflows/ci.yml)
[![GitHub release](https://img.shields.io/github/v/release/KIT-MRT/uniform_bspline)](https://github.com/KIT-MRT/uniform_bspline/releases)
[![PyPI](https://img.shields.io/pypi/v/uniform_bspline)](https://pypi.org/project/uniform_bspline/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![License: BSL-1.0](https://img.shields.io/badge/License-BSL_1.0-lightblue.svg)](https://www.boost.org/LICENSE_1_0.txt)

A header-only C++ library implementing uniform B-splines $` f: \mathbb{R}^n \rightarrow \mathbb{R}^m `$ with Python bindings.
Uniform means the knot vector is equally distributed, which allows the basis functions to be precomputed and makes evaluation significantly more efficient.

**Features:**

- **Flexible mapping** — arbitrary input $` n `$ and output $` m `$ dimensions, degrees 1–5
- **Control point types** — scalar, Eigen vectors, or custom types via traits
- **Full calculus** — evaluation, derivatives (arbitrary order), smoothness (integral of squared derivatives)
- **Header-only** — just add to your include path, no compilation required
- **Python bindings** — easily install via pybind11

## Quick Start

### C++

```cpp
#include <uniform_bspline/uniform_bspline.hpp>

// 1D → 1D scalar spline, degree 3
ubs::UniformBSpline11d<3> spline;
spline.setControlPoints({0.0, 1.0, 2.0, 3.0, 4.0});
spline.setBounds(-2.0, 5.0);               // set input range [−2, 5]

double val    = spline.evaluate(0.5);      // evaluate at t = 0.5
double deriv  = spline.derivative(0.5, 1); // 1st derivative
double smooth = spline.smoothness<1>();    // integral of squared velocity

// 1D → 3D Eigen spline (e.g. trajectory), degree 3
ubs::UniformBSpline<double, 3, double, Eigen::Vector3d,
                    EigenAlignedVec<Eigen::Vector3d>> spline1d3d;
spline1d3d.setControlPoints({{0,0,0}, {1,0,0}, {2,1,0}, {3,1,0}, {4,0,0}});

Eigen::Vector3d pos = spline1d3d.evaluate(0.5);
Eigen::Vector3d vel = spline1d3d.derivative(0.5, 1);

// 3D → 1D scalar field (e.g. occupancy / cost field), degree 3
using Field3d1d = ubs::UniformBSpline<double, 3,
    Eigen::Vector3d, double, ubs::EigenAlignedMultiArray<double, 3>>;
Field3d1d field;
// set up a 3D grid of control points, then:
double cost = field.evaluate({0.5, 0.2, 0.8});

// 3D → 2D mapping (e.g. projection / feature map), degree 3
using Field3d2d = ubs::UniformBSpline<double, 3,
    Eigen::Vector3d, Eigen::Vector2d,
    ubs::EigenAlignedMultiArray<Eigen::Vector2d, 3>>;
Field3d2d proj;
// set up a 3D grid of Vector2d control points, then:
Eigen::Vector2d uv = proj.evaluate({0.5, 0.2, 0.8});
```

### Python

```python
import uniform_bspline as ubs
import numpy as np

# 1D → 1D scalar spline, degree 3
spline = ubs.UniformBSpline1d1d3()
spline.set_control_points([0.0, 1.0, 2.0, 3.0, 4.0])
spline.set_bounds(-2.0, 5.0)           # set input range [−2, 5]

val    = spline.evaluate(0.5)          # evaluate at t = 0.5
deriv  = spline.derivative(0.5, 1)     # 1st derivative
smooth = spline.smoothness(1)          # integral of squared velocity

# 1D → 3D vector spline, degree 3
spline3d = ubs.UniformBSpline1d3d3()
cp = np.array([[0,0,0],[1,0,0],[2,1,0],[3,1,0],[4,0,0]], dtype=float)
spline3d.set_control_points(cp)

pos = spline3d.evaluate(0.5)          # numpy array, shape (3,)
vel = spline3d.derivative(0.5, 1)     # 1st derivative, shape (3,)

# 3D → 1D scalar field, degree 3  (e.g. occupancy / cost volume)
field = ubs.UniformBSpline3d1d3()
cp3 = np.ones((4, 4, 4), dtype=float)       # 3D grid of scalar control points
field.set_control_points(cp3)
field.set_bounds(np.zeros(3), np.ones(3))   # input range [0,1]³

val = field.evaluate(np.array([0.5, 0.5, 0.5]))                # scalar
grad = field.derivative(np.array([0.5, 0.5, 0.5]), [1, 0, 0])  # df/dx

# 3D → 2D mapping, degree 3  (e.g. deformation / projection field)
proj = ubs.UniformBSpline3d2d3()
cp4 = np.zeros((4, 4, 4, 2), dtype=float)             # 3D grid of 2D control points
proj.set_control_points(cp4)

uv = proj.evaluate(np.array([0.5, 0.2, 0.8]))         # numpy array, shape (2,)
```

For the full usage guide see the **[documentation](https://kit-mrt.github.io/uniform_bspline)**.

## Dependencies

| Dependency | Version | Notes |
|---|---|---|
| CMake | ≥ 3.16 | required |
| Eigen3 | ≥ 3.3 | required |
| Boost | ≥ 1.40 | required — multi_array |
| pybind11 | ≥ 2.11 | optional — Python bindings only |
| Python | ≥ 3.8 | optional — Python bindings only |
| numpy | ≥ 1.21 | optional — Python bindings only |

### Option A — shell script (Ubuntu/Debian)

```bash
./install_dependencies.sh            # core only
./install_dependencies.sh --tests    # core + C++ test dependencies
./install_dependencies.sh --python   # core + Python bindings
```

### Option B — vcpkg (cross-platform: Linux / macOS / Windows)

```bash
vcpkg install                              # reads vcpkg.json, installs Eigen3 + boost-multi-array
vcpkg install --x-feature=python-bindings  # also installs pybind11
```

Then configure CMake with:
```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
```

### Option C — Dev Container (zero-setup)

Open in VS Code → **Reopen in Container**.
All dependencies (including Python bindings) are installed automatically via `.devcontainer/devcontainer.json`.

## Installation

### C++ -- CMake

```bash
git clone https://github.com/KIT-MRT/uniform_bspline.git  # clone the repository
cd uniform_bspline
cmake -S . -B build                                       # configure
cmake --build build --parallel $(nproc)                   # compile
sudo cmake --install build                                # install system-wide
```

To also build the Doxygen HTML documentation:
```bash
cmake -S . -B build -DBUILD_DOCUMENTATION=ON  # configure with docs enabled
cmake --build build --target docs             # generate documentation
```

Then in your own project:
```cmake
find_package(uniform_bspline REQUIRED)
target_link_libraries(my_target PRIVATE uniform_bspline::uniform_bspline)
```

### C++ -- FetchContent (no install needed)

```cmake
include(FetchContent)
FetchContent_Declare(
    uniform_bspline
    GIT_REPOSITORY https://github.com/KIT-MRT/uniform_bspline.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(uniform_bspline)
target_link_libraries(my_target PRIVATE uniform_bspline::uniform_bspline)
```

### Python -- pip

#### 1. Install from PyPI

```bash
pip install uniform_bspline
```

#### 2. Install from GitHub

```bash
pip install git+https://github.com/KIT-MRT/uniform_bspline.git
```

#### 3. Install from a local clone

```bash
git clone https://github.com/KIT-MRT/uniform_bspline.git  # clone the repository
cd uniform_bspline
pip install .                                             # build and install
```

## Testing

### C++

Build and run the C++ unit tests with:

```bash
cmake -S . -B build -DBUILD_TESTS=ON               # configure with tests enabled
cmake --build build --parallel $(nproc)            # compile
ctest --test-dir build --output-on-failure         # run all tests
```

### Python

Install the package into a virtual environment first, then run the Python tests with pytest:

```bash
python -m venv .venv          # create a virtual environment
source .venv/bin/activate     # activate it
pip install .[test]           # build and install the package + pytest
pytest tests/python/          # run Python tests
```

## Citation

If you use this library in academic work, please cite:

```bibtex
@article{Beck2021_1000131090,
    author       = {Beck, Johannes},
    year         = {2021},
    title        = {Camera Calibration with Non-Central Local Camera Models},
    doi          = {10.5445/IR/1000131090},
    publisher    = {{Karlsruher Institut für Technologie (KIT)}},
    school       = {Karlsruher Institut für Technologie (KIT)}
}
```
