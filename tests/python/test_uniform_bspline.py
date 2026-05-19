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
