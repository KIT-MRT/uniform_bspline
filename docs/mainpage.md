# Uniform B-spline {#mainpage}

A header-only C++ library implementing **uniform B-splines**
@f$ f \colon \mathbb{R}^n \rightarrow \mathbb{R}^m @f$ with optional Python bindings.

- **Source**: https://github.com/KIT-MRT/uniform_bspline
- **License**: Boost Software License 1.0
- **Language**: C++17 (Python &ge; 3.8 for bindings)

---

## Background

A **B-spline** is a piecewise-polynomial curve defined by a sequence of *control points*
and a *knot vector*.
In the **uniform** case the knots are equally spaced, which allows the basis matrices to
be precomputed at compile time and stored as `constexpr` arrays.
This makes evaluation significantly faster than general B-spline evaluation and avoids
any heap allocation at run time.

The library supports **arbitrary input and output dimensions**:

| Use case | Input type | Output type | Example |
|---|---|---|---|
| Scalar trajectory | @f$\mathbb{R}@f$ | @f$\mathbb{R}@f$ | time &rarr; cost |
| 3D trajectory | @f$\mathbb{R}@f$ | @f$\mathbb{R}^3@f$ | time &rarr; position |
| Height map | @f$\mathbb{R}^2@f$ | @f$\mathbb{R}@f$ | (x,y) &rarr; elevation |
| Parametric surface | @f$\mathbb{R}^2@f$ | @f$\mathbb{R}^3@f$ | (u,v) &rarr; point on surface |
| Cost volume | @f$\mathbb{R}^3@f$ | @f$\mathbb{R}@f$ | (x,y,z) &rarr; cost |
| Deformation field | @f$\mathbb{R}^3@f$ | @f$\mathbb{R}^3@f$ | (x,y,z) &rarr; displacement |

Supported degrees: **1** (linear) through **5** (quintic).

---

## Design overview

The central class is ubs::UniformBSpline, parameterised as:

```cpp
template<
    typename Scalar,                   // floating-point type, e.g. double
    int      Degree,                   // polynomial degree: 1 ... 5
    typename InputType,                // e.g. double or Eigen::Vector2d
    typename OutputType,               // e.g. double or Eigen::Vector3d
    typename ControlPointsContainer    // storage for the control-point grid
>
class UniformBSpline;
```

The basis matrices are generated at compile time via a recursive `constexpr` algorithm,
so there is no runtime overhead for selecting a degree.

**Convenience aliases** exist for the most common combinations:

| Alias | Meaning |
|---|---|
| `ubs::UniformBSpline11d<Degree>` | @f$\mathbb{R} \rightarrow \mathbb{R}@f$ |
| `ubs::EigenUniformBSpline<Scalar,Degree,In,Out>` | Eigen input/output with `boost::multi_array` grid |

---

## 1D &rarr; 1D spline

Explicit full type:

\snippet examples.cpp Spline1d1d_Definition

Because this combination is so common there is a short alias:

\snippet examples.cpp Spline1d1d_DefinitionShort

The default constructor creates the minimum number of control points (all zero) and
places the spline over the interval @f$[0, 1]@f$.

**Setting control points:**

\snippet examples.cpp Spline1d1d_ControlPoints

**Adjusting the input range:**

\snippet examples.cpp Spline1d1d_Bounds

Now the spline is defined over @f$[-2, 5]@f$.

**Evaluation:**

\snippet examples.cpp Spline1d1d_Eval

**Derivatives:**

\snippet examples.cpp Spline1d1d_Derivative

**Smoothness** (integral of squared derivative — see UniformBSpline::smoothness):

\snippet examples.cpp Spline1d1d_Smoothness

---

## 1D &rarr; 3D Eigen spline

A spline @f$ f \colon \mathbb{R} \rightarrow \mathbb{R}^3 @f$ (e.g. a 3D position trajectory):

\snippet examples.cpp Spline1d3d_Definition

Evaluation, derivatives and smoothness all follow the same API;
the only difference is that results are now `Eigen::Vector3d`:

\snippet examples.cpp Spline1d3d_Eval

\snippet examples.cpp Spline1d3d_Derivative

\snippet examples.cpp Spline1d3d_Smoothness

> **Note:** The smoothness is returned component-wise.
> Sum the components to obtain the total smoothness of the curve.

---

## 2D &rarr; 1D Eigen spline

A spline @f$ f \colon \mathbb{R}^2 \rightarrow \mathbb{R} @f$
(e.g. a height map or a 2D cost field):

\snippet examples.cpp Spline2d1d_Definition

The control-point container is now a 2D grid (`Eigen::MatrixXd`).

**Evaluation** at a 2D query point:

\snippet examples.cpp Spline2d1d_Evaluate_Long

Using brace-initialised `Eigen::Vector2d` for brevity:

\snippet examples.cpp Spline2d1d_Evaluate_Short

**Partial derivatives:**

@f$ \frac{\partial f}{\partial x} @f$ at @f$(x, y)@f$:

\snippet examples.cpp Spline2d1d_Derivative_10

Mixed partial @f$ \frac{\partial^2 f}{\partial x \, \partial y} @f$:

\snippet examples.cpp Spline2d1d_Derivative_11

---

## 2D &rarr; 3D Eigen spline

A spline @f$ f \colon \mathbb{R}^2 \rightarrow \mathbb{R}^3 @f$
(e.g. a parametric surface):

\snippet examples.cpp Spline2d3d_Definition

The `EigenUniformBSpline` alias shortens the declaration:

\snippet examples.cpp Spline2d3d_Definition_Short

**Evaluation:**

\snippet examples.cpp Spline2d3d_Evaluate

---

## Control point types

The library is type-agnostic about control points.
Three trait headers are provided:

| Header | Container type | Typical use |
|---|---|---|
| `control_points_trait_std_container.hpp` | `std::vector`, `std::array` | 1D grids |
| `control_points_trait_eigen.hpp` | `Eigen::Matrix` / `Eigen::Array` | 2D grids, Eigen output |
| `control_points_trait_multi_array.hpp` | `boost::multi_array` / `ubs::EigenAlignedMultiArray` | &ge;2D grids, Eigen output |

To use a custom container type, specialise ubs::ControlPointsTrait for it.

---

## Derivatives

For **1D inputs** the derivative order is a plain `int`:

\snippet examples.cpp Spline1d1d_Derivative_Overview

For **multi-dimensional inputs** the derivative order is a `std::array<int, N>`,
where each element gives the order along the corresponding axis:

\snippet examples.cpp Spline2d1d_Derivative_Overview

The maximum total derivative order is limited by the spline degree.
Requesting an order &ge; degree returns zero.

---

## Smoothness

The smoothness functional is the integral of the squared @f$d@f$-th derivative over
the full input domain:

@f[
S_d = \int_{\mathbf{l}}^{\mathbf{u}}
      \left\| \frac{\partial^d f(\mathbf{x})}{\partial \mathbf{x}^d} \right\|^2
      \mathrm{d}\mathbf{x}
@f]

It is computed **analytically** (no numerical quadrature) from the control points:

\snippet examples.cpp Spline1d1d_Smoothness_Overview

For vector-valued splines the result is a vector; sum its components for a scalar
regularisation penalty.
This functional is widely used as a smoothness prior in trajectory optimisation and
spline fitting.

---

## Python bindings

The Python module `uniform_bspline` exposes pre-instantiated classes for the most
common type combinations:

| Class | Type |
|---|---|
| `UniformBSpline1d1d1/2/3/4/5` | @f$\mathbb{R} \rightarrow \mathbb{R}@f$, degree 1–5 |
| `UniformBSpline1d3d1/2/3/4/5` | @f$\mathbb{R} \rightarrow \mathbb{R}^3@f$, degree 1–5 |
| `UniformBSpline3d1d1/2/3/4/5` | @f$\mathbb{R}^3 \rightarrow \mathbb{R}@f$, degree 1–5 |
| `UniformBSpline3d2d1/2/3/4/5` | @f$\mathbb{R}^3 \rightarrow \mathbb{R}^2@f$, degree 1–5 |

**1D &rarr; 1D example:**

\snippet examples.py Spline1d1d_Python

**1D &rarr; 3D trajectory example:**

\snippet examples.py Spline1d3d_Python

**3D &rarr; 1D scalar field example:**

\snippet examples.py Spline3d1d_Python

**3D &rarr; 2D mapping example:**

\snippet examples.py Spline3d2d_Python

### Custom type combinations

The four `bind_*` helper templates cover every input/output dimension family.
To expose a type not included in the default module, call the appropriate
helper — or write a minimal `py::class_<>` directly — in your own pybind11
extension:

\snippet uniform_bspline_py.cpp CustomBinding_Example
