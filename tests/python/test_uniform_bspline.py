"""
Python binding tests for uniform_bspline.

These tests mirror the C++ examples.cpp / uniform_bspline.cpp tests so that
the Python API is verified to be consistent with the C++ implementation.

Run with:
    pytest tests/python/
"""

import math
import pytest
import numpy as np
import uniform_bspline as ubs


# ---------------------------------------------------------------------------
# 1D → 1D  (UniformBSpline1d1d3)
# ---------------------------------------------------------------------------

class TestUniformBSpline1d1d_Degree3:
    """R¹→R¹ cubic spline — covers the most common trajectory use-case."""

    def test_default_construction(self):
        s = ubs.UniformBSpline1d1d3()
        assert s.get_lower_bound() == pytest.approx(0.0)
        assert s.get_upper_bound() == pytest.approx(1.0)

    def test_bounds_constructor(self):
        s = ubs.UniformBSpline1d1d3(-2.0, 5.0)
        assert s.get_lower_bound() == pytest.approx(-2.0)
        assert s.get_upper_bound() == pytest.approx(5.0)

    def test_set_bounds(self):
        s = ubs.UniformBSpline1d1d3()
        s.set_bounds(-2.0, 5.0)
        assert s.get_lower_bound() == pytest.approx(-2.0)
        assert s.get_upper_bound() == pytest.approx(5.0)

    def test_control_points_roundtrip(self):
        s = ubs.UniformBSpline1d1d3()
        cp = [0.0, 1.0, 2.0, 3.0, 4.0]
        s.set_control_points(cp)
        assert s.get_control_points() == pytest.approx(cp)

    def test_evaluate_linear_spline(self):
        """Control points [0,1,2,3,4] define a straight line f(t)=4t on [0,1]."""
        s = ubs.UniformBSpline1d1d3()
        s.set_control_points([0.0, 1.0, 2.0, 3.0, 4.0])
        # Expected values from C++ examples.cpp
        assert s.evaluate(0.0) == pytest.approx(1.0, abs=1e-10)
        assert s.evaluate(1.0) == pytest.approx(3.0, abs=1e-10)

    def test_constructor_with_control_points(self):
        cp = [0.0, 1.0, 2.0, 3.0, 4.0]
        s = ubs.UniformBSpline1d1d3(cp)
        assert s.evaluate(0.0) == pytest.approx(1.0, abs=1e-10)

    def test_derivative(self):
        s = ubs.UniformBSpline1d1d3()
        s.set_control_points([0.0, 1.0, 2.0, 3.0, 4.0])
        # From C++ examples.cpp
        assert s.derivative(0.0, 1) == pytest.approx(2.0, abs=1e-9)
        assert s.derivative(1.0, 2) == pytest.approx(0.0, abs=1e-9)

    def test_smoothness(self):
        s = ubs.UniformBSpline1d1d3()
        s.set_control_points([0.0, 1.0, 2.0, 3.0, 4.0])
        # From C++ examples.cpp
        assert s.smoothness(1) == pytest.approx(4.0, abs=1e-9)

    def test_smoothness_zero_derivative(self):
        s = ubs.UniformBSpline1d1d3()
        s.set_control_points([0.0, 1.0, 2.0, 3.0, 4.0])
        val = s.smoothness(0)
        assert val > 0.0  # integral of f² > 0 for non-zero spline

    def test_in_range(self):
        s = ubs.UniformBSpline1d1d3(-1.0, 3.0)
        assert s.in_range(-1.0)
        assert s.in_range(3.0)
        assert s.in_range(1.0)
        assert not s.in_range(-1.5)
        assert not s.in_range(3.5)

    def test_get_num_control_points(self):
        s = ubs.UniformBSpline1d1d3()
        s.set_control_points([0.0, 1.0, 2.0, 3.0, 4.0])
        assert s.get_num_control_points() == 5

    def test_extrapolation_disabled_by_default(self):
        s = ubs.UniformBSpline1d1d3()
        assert not s.is_extrapolating()

    def test_extrapolation_toggle(self):
        s = ubs.UniformBSpline1d1d3()
        s.set_extrapolate(True)
        assert s.is_extrapolating()
        s.set_extrapolate(False)
        assert not s.is_extrapolating()

    def test_repr(self):
        s = ubs.UniformBSpline1d1d3()
        r = repr(s)
        assert "UniformBSpline1d1d3" in r
        assert "lower=" in r
        assert "upper=" in r

    def test_smoothness_invalid_derivative_raises(self):
        s = ubs.UniformBSpline1d1d3()
        with pytest.raises(Exception):
            s.smoothness(10)

    def test_bounds_with_control_points_constructor(self):
        cp = [1.0, 2.0, 3.0, 4.0, 5.0]
        s = ubs.UniformBSpline1d1d3(0.0, 2.0, cp)
        assert s.get_lower_bound() == pytest.approx(0.0)
        assert s.get_upper_bound() == pytest.approx(2.0)


class TestUniformBSpline1d1d_OtherDegrees:
    """Verify degree-1 through degree-5 splines construct and evaluate."""

    def test_construct_degree1(self):
        s = ubs.UniformBSpline1d1d1()
        assert s.get_lower_bound() == pytest.approx(0.0)

    def test_evaluate_degree1(self):
        cp = [0.0, 1.0, 2.0]
        s = ubs.UniformBSpline1d1d1(cp)
        val = s.evaluate(0.5)
        assert math.isfinite(val)

    def test_construct_degree2(self):
        s = ubs.UniformBSpline1d1d2()
        assert s.get_lower_bound() == pytest.approx(0.0)

    def test_evaluate_degree2(self):
        cp = [0.0, 1.0, 2.0, 3.0]
        s = ubs.UniformBSpline1d1d2(cp)
        val = s.evaluate(0.5)
        assert math.isfinite(val)

    def test_construct_degree4(self):
        s = ubs.UniformBSpline1d1d4()
        assert s.get_lower_bound() == pytest.approx(0.0)
        assert s.get_upper_bound() == pytest.approx(1.0)

    def test_evaluate_degree4(self):
        cp = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
        s = ubs.UniformBSpline1d1d4(cp)
        val = s.evaluate(0.5)
        assert math.isfinite(val)

    def test_construct_degree5(self):
        s = ubs.UniformBSpline1d1d5()
        assert s.get_lower_bound() == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# 1D → 3D  (UniformBSpline1d3d3)
# ---------------------------------------------------------------------------

class TestUniformBSpline1d3d_Degree3:
    """R¹→R³ cubic spline — 3D position trajectory."""

    def test_default_construction(self):
        s = ubs.UniformBSpline1d3d3()
        assert s.get_lower_bound() == pytest.approx(0.0)
        assert s.get_upper_bound() == pytest.approx(1.0)

    def test_evaluate_returns_3d_vector(self):
        s = ubs.UniformBSpline1d3d3()
        val = s.evaluate(0.5)
        assert val.shape == (3,)

    def test_evaluate_zero_spline(self):
        s = ubs.UniformBSpline1d3d3()
        val = s.evaluate(0.0)
        np.testing.assert_allclose(val, np.zeros(3), atol=1e-15)

    def test_control_points_roundtrip(self):
        cp = np.random.default_rng(0).standard_normal((8, 3))
        s = ubs.UniformBSpline1d3d3()
        s.set_control_points(cp)
        cp_back = s.get_control_points()
        assert cp_back.shape == (8, 3)
        np.testing.assert_allclose(cp_back, cp, atol=1e-15)

    def test_bounds_constructor(self):
        cp = np.zeros((6, 3))
        s = ubs.UniformBSpline1d3d3(0.0, 10.0, cp)
        assert s.get_lower_bound() == pytest.approx(0.0)
        assert s.get_upper_bound() == pytest.approx(10.0)

    def test_derivative_returns_3d_vector(self):
        s = ubs.UniformBSpline1d3d3()
        d = s.derivative(0.5, 1)
        assert d.shape == (3,)

    def test_smoothness_returns_3d_vector(self):
        s = ubs.UniformBSpline1d3d3()
        sm = s.smoothness(1)
        assert sm.shape == (3,)

    def test_evaluate_non_trivial(self):
        """A straight line in 3D: control points along the x-axis."""
        rng = np.random.default_rng(42)
        cp = np.zeros((6, 3))
        cp[:, 0] = np.linspace(0.0, 5.0, 6)  # x goes 0→5, y,z=0
        s = ubs.UniformBSpline1d3d3(0.0, 1.0, cp)
        val = s.evaluate(0.5)
        assert val[0] > 0.0  # x component is positive
        assert val[1] == pytest.approx(0.0, abs=1e-12)
        assert val[2] == pytest.approx(0.0, abs=1e-12)

    def test_get_num_control_points(self):
        cp = np.zeros((7, 3))
        s = ubs.UniformBSpline1d3d3()
        s.set_control_points(cp)
        assert s.get_num_control_points() == 7


class TestUniformBSpline1d3d_OtherDegrees:
    def test_construct_degree1(self):
        s = ubs.UniformBSpline1d3d1()
        val = s.evaluate(0.5)
        assert val.shape == (3,)

    def test_construct_degree2(self):
        s = ubs.UniformBSpline1d3d2()
        val = s.evaluate(0.5)
        assert val.shape == (3,)

    def test_construct_degree4(self):
        s = ubs.UniformBSpline1d3d4()
        val = s.evaluate(0.5)
        assert val.shape == (3,)

    def test_construct_degree5(self):
        s = ubs.UniformBSpline1d3d5()
        val = s.evaluate(0.5)
        assert val.shape == (3,)


# ---------------------------------------------------------------------------
# 3D → 1D  (UniformBSpline3d1d3)
# ---------------------------------------------------------------------------

class TestUniformBSpline3d1d_Degree3:
    """R³→R¹ cubic spline — 3D scalar field (cost, occupancy, distance field)."""

    def _make_spline(self, n0=4, n1=4, n2=4):
        """Helper: constant-value grid so the spline evaluates to 1.0 everywhere."""
        s = ubs.UniformBSpline3d1d3()
        cp = np.ones((n0, n1, n2), dtype=float)
        s.set_control_points(cp)
        return s

    def test_default_construction(self):
        s = ubs.UniformBSpline3d1d3()
        assert s is not None

    def test_evaluate_returns_scalar(self):
        s = self._make_spline()
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        assert isinstance(val, float)

    def test_evaluate_constant_field(self):
        """A grid of all-ones should evaluate to 1.0 at any interior point."""
        s = self._make_spline()
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        assert val == pytest.approx(1.0, abs=1e-10)

    def test_control_points_roundtrip(self):
        rng = np.random.default_rng(7)
        cp = rng.standard_normal((4, 4, 4))
        s = ubs.UniformBSpline3d1d3()
        s.set_control_points(cp)
        cp_back = s.get_control_points()
        assert cp_back.shape == (4, 4, 4)
        np.testing.assert_allclose(cp_back, cp, atol=1e-15)

    def test_set_bounds(self):
        s = ubs.UniformBSpline3d1d3()
        lb = np.array([0.0, 0.0, 0.0])
        ub = np.array([2.0, 3.0, 4.0])
        s.set_bounds(lb, ub)
        np.testing.assert_allclose(s.get_lower_bound(), lb)
        np.testing.assert_allclose(s.get_upper_bound(), ub)

    def test_in_range(self):
        s = self._make_spline()
        assert s.in_range(np.array([0.5, 0.5, 0.5]))
        assert not s.in_range(np.array([2.0, 0.5, 0.5]))

    def test_partial_derivative(self):
        """df/dx of a constant field is 0."""
        s = self._make_spline()
        d = s.derivative(np.array([0.5, 0.5, 0.5]), [1, 0, 0])
        assert d == pytest.approx(0.0, abs=1e-10)

    def test_smoothness(self):
        s = self._make_spline()
        sm = s.smoothness(1)
        assert isinstance(sm, float)

    def test_wrong_shape_raises(self):
        s = ubs.UniformBSpline3d1d3()
        with pytest.raises(Exception):
            s.set_control_points(np.ones((4, 4)))  # 2D array, should be 3D

    def test_degree1_constructs(self):
        s = ubs.UniformBSpline3d1d1()
        s.set_control_points(np.ones((2, 2, 2)))
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        assert val == pytest.approx(1.0, abs=1e-10)

    def test_degree2_constructs(self):
        s = ubs.UniformBSpline3d1d2()
        s.set_control_points(np.ones((3, 3, 3)))
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        assert val == pytest.approx(1.0, abs=1e-10)

    def test_degree4_constructs(self):
        s = ubs.UniformBSpline3d1d4()
        s.set_control_points(np.ones((5, 5, 5)))
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        assert val == pytest.approx(1.0, abs=1e-10)

    def test_degree5_constructs(self):
        s = ubs.UniformBSpline3d1d5()
        s.set_control_points(np.ones((6, 6, 6)))
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        assert val == pytest.approx(1.0, abs=1e-10)


# ---------------------------------------------------------------------------
# 3D → 2D  (UniformBSpline3d2d3)
# ---------------------------------------------------------------------------

class TestUniformBSpline3d2d_Degree3:
    """R³→R² cubic spline — 3D→2D projection / UV mapping."""

    def _make_spline(self, n0=4, n1=4, n2=4):
        s = ubs.UniformBSpline3d2d3()
        cp = np.ones((n0, n1, n2, 2), dtype=float)
        s.set_control_points(cp)
        return s

    def test_default_construction(self):
        s = ubs.UniformBSpline3d2d3()
        assert s is not None

    def test_evaluate_returns_2d_vector(self):
        s = self._make_spline()
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        assert val.shape == (2,)

    def test_evaluate_constant_field(self):
        s = self._make_spline()
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        np.testing.assert_allclose(val, np.ones(2), atol=1e-10)

    def test_control_points_roundtrip(self):
        rng = np.random.default_rng(8)
        cp = rng.standard_normal((4, 4, 4, 2))
        s = ubs.UniformBSpline3d2d3()
        s.set_control_points(cp)
        cp_back = s.get_control_points()
        assert cp_back.shape == (4, 4, 4, 2)
        np.testing.assert_allclose(cp_back, cp, atol=1e-15)

    def test_partial_derivative(self):
        s = self._make_spline()
        d = s.derivative(np.array([0.5, 0.5, 0.5]), [1, 0, 0])
        assert d.shape == (2,)
        np.testing.assert_allclose(d, np.zeros(2), atol=1e-10)

    def test_smoothness(self):
        s = self._make_spline()
        sm = s.smoothness(1)
        assert sm.shape == (2,)

    def test_wrong_shape_raises(self):
        s = ubs.UniformBSpline3d2d3()
        with pytest.raises(Exception):
            s.set_control_points(np.ones((4, 4, 4, 3)))  # OutputDims=3, not 2

    def test_degree1_constructs(self):
        s = ubs.UniformBSpline3d2d1()
        s.set_control_points(np.ones((2, 2, 2, 2)))
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        assert val.shape == (2,)

    def test_degree2_constructs(self):
        s = ubs.UniformBSpline3d2d2()
        s.set_control_points(np.ones((3, 3, 3, 2)))
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        assert val.shape == (2,)

    def test_degree4_constructs(self):
        s = ubs.UniformBSpline3d2d4()
        s.set_control_points(np.ones((5, 5, 5, 2)))
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        assert val.shape == (2,)

    def test_degree5_constructs(self):
        s = ubs.UniformBSpline3d2d5()
        s.set_control_points(np.ones((6, 6, 6, 2)))
        val = s.evaluate(np.array([0.5, 0.5, 0.5]))
        assert val.shape == (2,)


# ---------------------------------------------------------------------------
# Examples — one test per documentation snippet in examples.py
# ---------------------------------------------------------------------------

def test_example_spline1d1d():
    """## [Spline1d1d_Python] snippet."""
    s = ubs.UniformBSpline1d1d3()
    s.set_control_points([0.0, 1.0, 2.0, 3.0, 4.0])
    s.set_bounds(-2.0, 5.0)

    val    = s.evaluate(0.5)
    deriv  = s.derivative(0.5, 1)
    smooth = s.smoothness(1)

    assert math.isfinite(val)
    assert math.isfinite(deriv)
    assert math.isfinite(smooth)


def test_example_spline1d3d():
    """## [Spline1d3d_Python] snippet."""
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

    pos    = traj.evaluate(5.0)
    vel    = traj.derivative(5.0, 1)
    smooth = traj.smoothness(1)

    assert pos.shape == (3,)
    assert vel.shape == (3,)
    assert smooth.shape == (3,)


def test_example_spline3d1d():
    """## [Spline3d1d_Python] snippet."""
    field = ubs.UniformBSpline3d1d3()
    cp = np.ones((4, 4, 4), dtype=float)
    field.set_control_points(cp)
    field.set_bounds(np.zeros(3), np.ones(3))

    val  = field.evaluate(np.array([0.5, 0.5, 0.5]))
    dfdx = field.derivative(np.array([0.5, 0.5, 0.5]), [1, 0, 0])

    assert math.isfinite(val)
    assert math.isfinite(dfdx)


def test_example_spline3d2d():
    """## [Spline3d2d_Python] snippet."""
    proj = ubs.UniformBSpline3d2d3()
    cp = np.zeros((4, 4, 4, 2), dtype=float)
    proj.set_control_points(cp)
    proj.set_bounds(np.zeros(3), np.ones(3))

    uv  = proj.evaluate(np.array([0.5, 0.5, 0.5]))
    duv = proj.derivative(np.array([0.5, 0.5, 0.5]), [1, 0, 0])

    assert uv.shape == (2,)
    assert duv.shape == (2,)


# ---------------------------------------------------------------------------
# Real-data tests — evaluate and smoothness against Mathematica ground truth
# ---------------------------------------------------------------------------
# Only files whose (inputDims, outputDims, degree) map to an exposed Python class
# are tested:
#
#   Evaluate:   1d1d5  → test_points_1,5,8,12.txt
#               3d2d2  → test_points_4,11.txt
#
#   Smoothness: 1d1d3  → test_points_smoothness_1,2,4,5,13,14,16,17.txt
#               3d2d3  → test_points_smoothness_12,24.txt
# ---------------------------------------------------------------------------

import os

_TEST_DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "cpp", "test_data")


def _spline_class(input_dims: int, output_dims: int, degree: int):
    name = f"UniformBSpline{input_dims}d{output_dims}d{degree}"
    return getattr(ubs, name)


def _build_spline(tokens, idx, input_dims, output_dims, degree, derivatives):
    """Parse control points + bounds from token list, return (spline, idx)."""
    # Number of control points per dimension
    num_cps = [int(tokens[idx + j]) for j in range(input_dims)]
    idx += input_dims
    total_cps = 1
    for n in num_cps:
        total_cps *= n
    # Control point values (flat)
    n_vals = total_cps * output_dims
    cp_flat = [float(tokens[idx + j]) for j in range(n_vals)]
    idx += n_vals
    # Bounds — stored interleaved: lb[0], ub[0], lb[1], ub[1], ...
    lower = []; upper = []
    for _ in range(input_dims):
        lower.append(float(tokens[idx])); upper.append(float(tokens[idx + 1])); idx += 2

    cls = _spline_class(input_dims, output_dims, degree)
    s = cls()
    if input_dims == 1:
        cp_arr = cp_flat if output_dims == 1 else np.array(cp_flat).reshape(num_cps[0], output_dims)
        s.set_control_points(cp_arr)
        s.set_bounds(lower[0], upper[0])
    else:
        shape = num_cps + ([output_dims] if output_dims > 1 else [])
        cp_arr = np.array(cp_flat).reshape(shape)
        s.set_control_points(cp_arr)
        s.set_bounds(np.array(lower, dtype=float), np.array(upper, dtype=float))
    return s, idx


def _load_evaluate_file(file_idx: int):
    """
    Parse tests/cpp/test_data/test_points_{file_idx}.txt.

    Returns (spline, has_derivative, derivatives, test_cases) where
    test_cases = [(input_pos, gt_output), ...].
    """
    path = os.path.join(_TEST_DATA_DIR, f"test_points_{file_idx}.txt")
    tokens = open(path).read().split()
    idx = 0
    degree     = int(tokens[idx]);     input_dims  = int(tokens[idx + 1])
    output_dims = int(tokens[idx + 2]); idx += 3
    # Read per-dimension derivative orders
    derivatives = [int(tokens[idx + j]) for j in range(input_dims)]
    idx += input_dims
    has_derivative = any(d > 0 for d in derivatives)

    s, idx = _build_spline(tokens, idx, input_dims, output_dims, degree, derivatives)

    num_tests = int(tokens[idx]); idx += 1
    test_cases = []
    for _ in range(num_tests):
        pos = [float(tokens[idx + j]) for j in range(input_dims)]; idx += input_dims
        gt  = [float(tokens[idx + j]) for j in range(output_dims)]; idx += output_dims
        test_cases.append((pos, gt))
    return s, has_derivative, derivatives, test_cases


def _load_smoothness_file(file_idx: int):
    """
    Parse tests/cpp/test_data/test_points_smoothness_{file_idx}.txt.

    Returns (spline, derivative_order, gt_smoothness_scalar).
    """
    path = os.path.join(_TEST_DATA_DIR, f"test_points_smoothness_{file_idx}.txt")
    tokens = open(path).read().split()
    idx = 0
    degree     = int(tokens[idx]);     input_dims  = int(tokens[idx + 1])
    output_dims = int(tokens[idx + 2]); derivative = int(tokens[idx + 3]); idx += 4

    s, idx = _build_spline(tokens, idx, input_dims, output_dims, degree, [derivative])
    gt_val = float(tokens[idx])
    return s, derivative, gt_val


class TestRealDataEvaluate:
    """Verify evaluate() and derivative() against Mathematica ground truth.

    Tolerance mirrors the C++ test suite: 1e-10.
    """

    @pytest.mark.parametrize("file_idx", [1, 5, 8, 12])
    def test_1d1d5_evaluate(self, file_idx):
        s, has_deriv, derivs, cases = _load_evaluate_file(file_idx)
        for pos, gt in cases:
            if not has_deriv:
                val = s.evaluate(pos[0])
                assert val == pytest.approx(gt[0], abs=1e-10), f"file {file_idx} pos={pos}"
            else:
                val = s.derivative(pos[0], derivs[0])
                assert val == pytest.approx(gt[0], abs=1e-10), f"file {file_idx} pos={pos}"

    @pytest.mark.parametrize("file_idx", [4, 11])
    def test_3d2d2_evaluate(self, file_idx):
        s, has_deriv, derivs, cases = _load_evaluate_file(file_idx)
        for pos, gt in cases:
            p = np.array(pos)
            if not has_deriv:
                val = s.evaluate(p)
            else:
                val = s.derivative(p, derivs)
            np.testing.assert_allclose(val, gt, atol=1e-10,
                err_msg=f"file {file_idx} pos={pos}")


class TestRealDataSmoothness:
    """Verify smoothness() against Mathematica ground truth.

    Tolerance: relative 1e-5 (matching the C++ test bound: 1e-5 * (1 + |gt|)).
    """

    @pytest.mark.parametrize("file_idx", [1, 2, 4, 5, 13, 14, 16, 17])
    def test_1d1d3_smoothness(self, file_idx):
        s, deriv, gt = _load_smoothness_file(file_idx)
        val = s.smoothness(deriv)
        tol = 1e-5 * (1.0 + abs(gt))
        assert val == pytest.approx(gt, abs=tol), f"file {file_idx}"

    @pytest.mark.parametrize("file_idx", [12, 24])
    def test_3d2d3_smoothness(self, file_idx):
        s, deriv, gt = _load_smoothness_file(file_idx)
        val = s.smoothness(deriv).sum()   # component-wise → scalar, as C++ does
        tol = 1e-5 * (1.0 + abs(gt))
        assert val == pytest.approx(gt, abs=tol), f"file {file_idx}"
