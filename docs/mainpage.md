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

- Scalar trajectory — @f$\mathbb{R} \rightarrow \mathbb{R}@f$; e.g. time &rarr; cost
- 3D trajectory — @f$\mathbb{R} \rightarrow \mathbb{R}^3@f$; e.g. time &rarr; position
- Height map — @f$\mathbb{R}^2 \rightarrow \mathbb{R}@f$; e.g. (x,y) &rarr; elevation
- Parametric surface — @f$\mathbb{R}^2 \rightarrow \mathbb{R}^3@f$; e.g. (u,v) &rarr; point on surface
- Cost volume — @f$\mathbb{R}^3 \rightarrow \mathbb{R}@f$; e.g. (x,y,z) &rarr; cost
- Deformation field — @f$\mathbb{R}^3 \rightarrow \mathbb{R}^3@f$; e.g. (x,y,z) &rarr; displacement

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

- `ubs::UniformBSpline11d<Degree>` — @f$\mathbb{R} \rightarrow \mathbb{R}@f$
- `ubs::EigenUniformBSpline<Scalar,Degree,In,Out>` — Eigen input/output with `boost::multi_array` grid

---

## Key components

- **ubs::UniformBSpline** — the central spline class; templated on scalar type, degree, input type, output type, and control-point container.
- **ubs::UniformBSpline::evaluate** — evaluates the spline at a given parameter position.
- **ubs::UniformBSpline::derivative** — evaluates any derivative order at a given parameter position.
- **ubs::UniformBSpline::smoothness** — computes the exact closed-form smoothness integral of a chosen derivative order.
- **ubs::ControlPointsTrait** — specialise this to plug in a custom control-point container type.
- **ubs::UniformBSpline11d** — convenience alias for @f$\mathbb{R} \rightarrow \mathbb{R}@f$ splines.
- **ubs::EigenUniformBSpline** — convenience alias for Eigen input/output splines with `boost::multi_array` grid.

---

## C++ Usage

### 1D &rarr; 1D spline

Explicit full type:

\snippet examples.cpp Spline1d1d_Definition

The first template argument is the scalar value type, the second is the degree of the spline, the third is the input type, and the fourth is the output type — so this defines a degree-3 uniform B-spline that takes a `double` as input and produces a `double` as output. The last template parameter is the container used to store the control points.

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

This evaluates the spline at positions zero and one.

**Derivatives:**

\snippet examples.cpp Spline1d1d_Derivative

The first statement evaluates the first derivative at zero; the second evaluates the second derivative at one.

**Smoothness** (integral of squared derivative — see UniformBSpline::smoothness):

\snippet examples.cpp Spline1d1d_Smoothness

This computes the smoothness using the first derivative.

---

### 1D &rarr; 3D Eigen spline

A spline @f$ f \colon \mathbb{R} \rightarrow \mathbb{R}^3 @f$ (e.g. a 3D position trajectory). The definition is very similar to the 1D&rarr;1D case; the only difference is that the output type is now `Eigen::Vector3d`:

\snippet examples.cpp Spline1d3d_Definition

As we still have a one-dimensional input, evaluation, derivatives and smoothness work the same way — the output type simply changes:

\snippet examples.cpp Spline1d3d_Eval

\snippet examples.cpp Spline1d3d_Derivative

\snippet examples.cpp Spline1d3d_Smoothness

\note The smoothness is returned component-wise. Sum the components to obtain the total smoothness of the curve.

---

### 2D &rarr; 1D Eigen spline

A spline @f$ f \colon \mathbb{R}^2 \rightarrow \mathbb{R} @f$
(e.g. a height map or a 2D cost field). The input type is now `Eigen::Vector2d` and the output type is `double`. Because the control points must be arranged on a two-dimensional grid, an `Eigen::MatrixXd` is used as the container:

\snippet examples.cpp Spline2d1d_Definition

**Evaluation** requires a 2D query point:

\snippet examples.cpp Spline2d1d_Evaluate_Long

Using brace-initialised `Eigen::Vector2d` for brevity:

\snippet examples.cpp Spline2d1d_Evaluate_Short

**Partial derivatives:** for multi-dimensional inputs the derivative order is a `std::array<int, N>`, where each element specifies the order along the corresponding axis.

@f$ \frac{\partial f}{\partial x} @f$ at @f$(x, y)@f$:

\snippet examples.cpp Spline2d1d_Derivative_10

Mixed partial @f$ \frac{\partial^2 f}{\partial x \, \partial y} @f$:

\snippet examples.cpp Spline2d1d_Derivative_11

---

### 2D &rarr; 3D Eigen spline

A spline @f$ f \colon \mathbb{R}^2 \rightarrow \mathbb{R}^3 @f$
(e.g. a parametric surface). Here `Eigen::Vector2d` is the input, `Eigen::Vector3d` the output, and a `boost::multi_array` is used to store the 2D control-point grid:

\snippet examples.cpp Spline2d3d_Definition

Because such full type declarations become lengthy, the `EigenUniformBSpline` alias shortens them:

\snippet examples.cpp Spline2d3d_Definition_Short

**Evaluation** takes a two-dimensional input and returns a three-dimensional output:

\snippet examples.cpp Spline2d3d_Evaluate

Derivatives and smoothness follow the same API as the other multi-dimensional cases.

---

### Control point types

The library is type-agnostic about control points.
Three trait headers are provided:

- `control_points_trait_std_container.hpp` — `std::vector`, `std::array`; for 1D grids
- `control_points_trait_eigen.hpp` — `Eigen::Matrix` / `Eigen::Array`; for 2D grids with Eigen output
- `control_points_trait_multi_array.hpp` — `boost::multi_array` / `ubs::EigenAlignedMultiArray`; for ≥2D grids with Eigen output

To use a custom container type, specialise ubs::ControlPointsTrait for it.

---

### Derivatives

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

To see the full C++ examples see \ref examples.cpp "examples.cpp".

---

## Python bindings

The Python module `uniform_bspline` (imported as `ubs`) exposes pre-instantiated classes for the most
common type combinations:

- `UniformBSpline1d1d1`–`5` — @f$\mathbb{R} \rightarrow \mathbb{R}@f$, degree 1–5
- `UniformBSpline1d3d1`–`5` — @f$\mathbb{R} \rightarrow \mathbb{R}^3@f$, degree 1–5
- `UniformBSpline3d1d1`–`5` — @f$\mathbb{R}^3 \rightarrow \mathbb{R}@f$, degree 1–5
- `UniformBSpline3d2d1`–`5` — @f$\mathbb{R}^3 \rightarrow \mathbb{R}^2@f$, degree 1–5

All classes share the same interface: construct with `()`, set control points with `set_control_points()`, adjust the domain with `set_bounds()`, then call `evaluate()`, `derivative()`, or `smoothness()`. The `smoothness()` method takes the derivative order as an integer argument.

### 1D &rarr; 1D

A scalar spline over a custom interval. Control points are a plain Python list. After setting the domain to @f$[-2, 5]@f$, `evaluate()` returns a `float`, `derivative()` returns the @f$n@f$-th derivative, and `smoothness()` returns the scalar smoothness integral:

\snippet examples.py Spline1d1d_Python

### 1D &rarr; 3D trajectory

A 3D position trajectory @f$f \colon \mathbb{R} \rightarrow \mathbb{R}^3@f$. Control points are passed as a NumPy array of shape `(N, 3)`. `evaluate()` and `derivative()` each return a NumPy array of shape `(3,)`. The `smoothness()` result is also shape `(3,)` — component-wise; sum the components for a scalar regularisation penalty:

\snippet examples.py Spline1d3d_Python

### 3D &rarr; 1D scalar field

A scalar-valued function over a 3D domain @f$f \colon \mathbb{R}^3 \rightarrow \mathbb{R}@f$ (e.g. a cost volume or signed-distance field). Control points form a 3D grid of shape `(Nx, Ny, Nz)`. The query position and derivative order are each passed as a NumPy array / Python list of length 3:

\snippet examples.py Spline3d1d_Python

### 3D &rarr; 2D mapping

A vector-valued function @f$f \colon \mathbb{R}^3 \rightarrow \mathbb{R}^2@f$ (e.g. a projection or deformation field). Control points have shape `(Nx, Ny, Nz, 2)`. `evaluate()` returns a NumPy array of shape `(2,)`:

\snippet examples.py Spline3d2d_Python

To see the full examples see \ref examples.py "examples.py".

### Custom type combinations

The four `bind_*` helper templates (`bind_1d1d`, `bind_1dNd`, `bind_3d1d`, `bind_3dNd`) cover every supported input/output dimension family.
To expose a type not included in the default module, copy the relevant helper into your own pybind11 extension. As an example, `bind_3dNd` handles @f$\mathbb{R}^3 \rightarrow \mathbb{R}^N@f$ splines and can be instantiated for any degree and output dimension:

\snippet uniform_bspline_py.cpp CustomBinding_Example

To see the full binding source see \ref uniform_bspline_py.cpp "uniform_bspline_py.cpp".
