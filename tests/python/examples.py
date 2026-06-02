"""
Runnable Python examples used as Doxygen snippets in docs/mainpage.md.

Each named region between  ## [Tag]  /  ## [Tag]  markers is embedded
verbatim into the documentation via  \snippet examples.py Tag.
"""

import numpy as np
import uniform_bspline as ubs

# ---------------------------------------------------------------------------
# See the Doxygen documentation for a more detailed explanation of the following example.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# 1D -> 1D example
# ---------------------------------------------------------------------------

## [Spline1d1d_Python]
s = ubs.UniformBSpline1d1d3()
s.set_control_points([0.0, 1.0, 2.0, 3.0, 4.0])
s.set_bounds(-2.0, 5.0)

val    = s.evaluate(0.5)
deriv  = s.derivative(0.5, 1)
smooth = s.smoothness(1)
## [Spline1d1d_Python]

# ---------------------------------------------------------------------------
# 1D -> 3D trajectory example
# ---------------------------------------------------------------------------

## [Spline1d3d_Python]
traj = ubs.UniformBSpline1d3d3()
cp = np.array([
    [0.0, 0.0, 0.0],
    [1.0, 0.5, 0.0],
    [2.0, 0.0, 0.5],
    [3.0, 0.5, 1.0],
    [4.0, 0.0, 0.0],
])
traj.set_control_points(cp)
traj.set_bounds(0.0, 10.0)

pos    = traj.evaluate(5.0)       # Eigen::Vector3d -> np.ndarray shape (3,)
vel    = traj.derivative(5.0, 1)
smooth = traj.smoothness(1)       # component-wise; sum for scalar penalty
## [Spline1d3d_Python]

# ---------------------------------------------------------------------------
# 3D -> 1D scalar field example
# ---------------------------------------------------------------------------

## [Spline3d1d_Python]
field = ubs.UniformBSpline3d1d3()
cp = np.ones((4, 4, 4), dtype=float)
field.set_control_points(cp)
field.set_bounds(np.zeros(3), np.ones(3))

val  = field.evaluate(np.array([0.5, 0.5, 0.5]))
dfdx = field.derivative(np.array([0.5, 0.5, 0.5]), [1, 0, 0])
## [Spline3d1d_Python]

# ---------------------------------------------------------------------------
# 3D -> 2D mapping example
# ---------------------------------------------------------------------------

## [Spline3d2d_Python]
proj = ubs.UniformBSpline3d2d3()
cp = np.zeros((4, 4, 4, 2), dtype=float)
proj.set_control_points(cp)
proj.set_bounds(np.zeros(3), np.ones(3))

uv   = proj.evaluate(np.array([0.5, 0.5, 0.5]))   # shape (2,)
duv  = proj.derivative(np.array([0.5, 0.5, 0.5]), [1, 0, 0])
## [Spline3d2d_Python]
