#include <array>
#include <stdexcept>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <uniform_bspline/multi_array.hpp>
#include <uniform_bspline/uniform_bspline.hpp>

namespace py = pybind11;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Helper: call smoothness<N> only when N is within the spline's degree;
// otherwise return zero (derivative >= degree is always zero analytically).
template <int N, typename Spline>
typename Spline::OutputType smoothness_at(const Spline& s) {
    if constexpr (N < Spline::Order) {
        return s.template smoothness<N>();
    } else {
        typename Spline::OutputType result{};
        return result;
    }
}

// Dispatch smoothness<N> at runtime (compile-time template → runtime int).
template <typename Spline>
typename Spline::OutputType smoothness_dispatch(const Spline& s, int derivative) {
    switch (derivative) {
        case 0: return smoothness_at<0>(s);
        case 1: return smoothness_at<1>(s);
        case 2: return smoothness_at<2>(s);
        case 3: return smoothness_at<3>(s);
        default: throw std::invalid_argument("smoothness derivative must be 0, 1, 2, or 3");
    }
}

// ---------------------------------------------------------------------------
// Bind a 1D→1D spline: R¹→R¹ with std::vector<double> control points.
// Template parameter: Degree.
// Python class name passed as argument.
// ---------------------------------------------------------------------------
template <int Degree>
void bind_1d1d(py::module& m, const char* name) {
    using Spline = ubs::UniformBSpline11d<Degree>;

    py::class_<Spline>(m, name)
        // Constructors
        .def(py::init<>(),
             "Default constructor: [0,1] bounds, minimum control points (all zero).")
        .def(py::init<double, double>(), py::arg("lower_bound"), py::arg("upper_bound"),
             "Construct with specified bounds; control points initialised to zero.")
        .def(py::init<double, double, const std::vector<double>&>(),
             py::arg("lower_bound"), py::arg("upper_bound"), py::arg("control_points"),
             "Construct with bounds and control points.")
        .def(py::init([](const std::vector<double>& cp) {
                 return std::make_unique<Spline>(cp);
             }),
             py::arg("control_points"),
             "Construct with control points; bounds default to [0, 1].")

        // Control points
        .def("set_control_points",
             [](Spline& s, const std::vector<double>& cp) { s.setControlPoints(cp); },
             py::arg("control_points"))
        .def("get_control_points",
             [](const Spline& s) -> std::vector<double> { return s.getControlPoints(); })

        // Bounds
        .def("set_bounds",
             [](Spline& s, double lb, double ub) { s.setBounds(lb, ub); },
             py::arg("lower_bound"), py::arg("upper_bound"))
        .def("get_lower_bound",
             [](const Spline& s) -> double { return s.getLowerBound(); })
        .def("get_upper_bound",
             [](const Spline& s) -> double { return s.getUpperBound(); })
        .def("get_num_control_points",
             [](const Spline& s) { return s.getNumControlPoints(0); })
        .def("in_range",
             [](const Spline& s, double pos) { return s.inRange(pos); },
             py::arg("pos"))

        // Evaluation
        .def("evaluate",
             [](const Spline& s, double pos) -> double { return s.evaluate(pos); },
             py::arg("pos"),
             "Evaluate the spline at pos.")
        .def("derivative",
             [](const Spline& s, double pos, int d) -> double { return s.derivative(pos, d); },
             py::arg("pos"), py::arg("order"),
             "Evaluate the derivative of order `order` at pos.")
        .def("smoothness",
             [](const Spline& s, int d) -> double { return smoothness_dispatch(s, d); },
             py::arg("derivative"),
             "Compute the smoothness integral for the given derivative order.")

        // Extrapolation
        .def("set_extrapolate", &Spline::setExtrapolate, py::arg("enable"))
        .def("is_extrapolating", &Spline::isExtrapolating)

        // Repr
        .def("__repr__", [name](const Spline& s) {
            return std::string(name) + "(lower=" + std::to_string(s.getLowerBound()) +
                   ", upper=" + std::to_string(s.getUpperBound()) +
                   ", n_ctrl=" + std::to_string(s.getNumControlPoints(0)) + ")";
        });
}

// ---------------------------------------------------------------------------
// Bind a 1D→Nd spline: R¹→R^OutputDims.
// Control points type: std::vector<Eigen::Matrix<double, OutputDims, 1>, aligned_allocator>.
// ---------------------------------------------------------------------------
template <int Degree, int OutputDims>
void bind_1dNd(py::module& m, const char* name) {
    using Vec = Eigen::Matrix<double, OutputDims, 1>;
    using AlignedVec = std::vector<Vec, Eigen::aligned_allocator<Vec>>;
    using Spline = ubs::UniformBSpline<double, Degree, double, Vec, AlignedVec>;

    py::class_<Spline>(m, name)
        // Constructors
        .def(py::init<>(),
             "Default constructor: [0,1] bounds, minimum control points (all zero).")
        .def(py::init<double, double>(), py::arg("lower_bound"), py::arg("upper_bound"),
             "Construct with specified bounds; control points initialised to zero.")
        // Construct from numpy array (N × OutputDims)
        .def(py::init([](double lb, double ub,
                         const Eigen::Matrix<double, Eigen::Dynamic, OutputDims>& cp_mat) {
                 AlignedVec cp(cp_mat.rows());
                 for (int i = 0; i < cp_mat.rows(); ++i) {
                     cp[i] = cp_mat.row(i).transpose();
                 }
                 return std::make_unique<Spline>(lb, ub, cp);
             }),
             py::arg("lower_bound"), py::arg("upper_bound"), py::arg("control_points"),
             "Construct with bounds and control points (shape N x OutputDims).")

        // Control points: accept / return as (N × OutputDims) numpy array
        .def("set_control_points",
             [](Spline& s, const Eigen::Matrix<double, Eigen::Dynamic, OutputDims>& cp_mat) {
                 AlignedVec cp(cp_mat.rows());
                 for (int i = 0; i < cp_mat.rows(); ++i) {
                     cp[i] = cp_mat.row(i).transpose();
                 }
                 s.setControlPoints(cp);
             },
             py::arg("control_points"),
             "Set control points from an (N x OutputDims) array.")
        .def("get_control_points",
             [](const Spline& s) {
                 const auto& cp = s.getControlPoints();
                 Eigen::Matrix<double, Eigen::Dynamic, OutputDims> mat(cp.size(), OutputDims);
                 for (int i = 0; i < (int)cp.size(); ++i) {
                     mat.row(i) = cp[i].transpose();
                 }
                 return mat;
             },
             "Return control points as an (N x OutputDims) array.")

        // Bounds
        .def("set_bounds",
             [](Spline& s, double lb, double ub) { s.setBounds(lb, ub); },
             py::arg("lower_bound"), py::arg("upper_bound"))
        .def("get_lower_bound",
             [](const Spline& s) -> double { return s.getLowerBound(); })
        .def("get_upper_bound",
             [](const Spline& s) -> double { return s.getUpperBound(); })
        .def("get_num_control_points",
             [](const Spline& s) { return s.getNumControlPoints(0); })
        .def("in_range",
             [](const Spline& s, double pos) { return s.inRange(pos); },
             py::arg("pos"))

        // Evaluation
        .def("evaluate",
             [](const Spline& s, double pos) -> Vec { return s.evaluate(pos); },
             py::arg("pos"),
             "Evaluate the spline at pos; returns a (OutputDims,) array.")
        .def("derivative",
             [](const Spline& s, double pos, int d) -> Vec { return s.derivative(pos, d); },
             py::arg("pos"), py::arg("order"),
             "Evaluate the derivative of order `order`; returns a (OutputDims,) array.")
        .def("smoothness",
             [](const Spline& s, int d) -> Vec { return smoothness_dispatch(s, d); },
             py::arg("derivative"),
             "Compute the smoothness integral; returns a (OutputDims,) array.")

        // Extrapolation
        .def("set_extrapolate", &Spline::setExtrapolate, py::arg("enable"))
        .def("is_extrapolating", &Spline::isExtrapolating)

        .def("__repr__", [name](const Spline& s) {
            return std::string(name) + "(lower=" + std::to_string(s.getLowerBound()) +
                   ", upper=" + std::to_string(s.getUpperBound()) +
                   ", n_ctrl=" + std::to_string(s.getNumControlPoints(0)) + ")";
        });
}

// ---------------------------------------------------------------------------
// Bind a 3D→1D spline: R³→R¹ with EigenAlignedMultiArray<double,3> control points.
// Python control points: numpy array of shape (N0, N1, N2).
// Python input: numpy array of shape (3,) i.e. Eigen::Vector3d.
// ---------------------------------------------------------------------------
template <int Degree>
void bind_3d1d(py::module& m, const char* name) {
    using Input = Eigen::Vector3d;
    using Grid  = ubs::EigenAlignedMultiArray<double, 3>;
    using Spline = ubs::UniformBSpline<double, Degree, Input, double, Grid>;

    py::class_<Spline>(m, name)
        .def(py::init<>(),
             "Default constructor: [0,1]³ bounds, minimum control points (all zero).")

        // Control points: accept / return as (N0×N1×N2) numpy array
        .def("set_control_points",
             [](Spline& s, py::array_t<double, py::array::c_style> arr) {
                 if (arr.ndim() != 3)
                     throw std::invalid_argument("control_points must be a 3-D array (N0, N1, N2)");
                 Grid grid(boost::extents[arr.shape(0)][arr.shape(1)][arr.shape(2)]);
                 auto r = arr.unchecked<3>();
                 for (py::ssize_t i = 0; i < arr.shape(0); ++i)
                     for (py::ssize_t j = 0; j < arr.shape(1); ++j)
                         for (py::ssize_t k = 0; k < arr.shape(2); ++k)
                             grid[i][j][k] = r(i, j, k);
                 s.setControlPoints(grid);
             },
             py::arg("control_points"),
             "Set control points from a (N0, N1, N2) numpy array.")
        .def("get_control_points",
             [](const Spline& s) {
                 const auto& g = s.getControlPoints();
                 const auto& shape = g.shape();
                 py::array_t<double> arr({(py::ssize_t)shape[0],
                                          (py::ssize_t)shape[1],
                                          (py::ssize_t)shape[2]});
                 auto w = arr.mutable_unchecked<3>();
                 for (py::ssize_t i = 0; i < (py::ssize_t)shape[0]; ++i)
                     for (py::ssize_t j = 0; j < (py::ssize_t)shape[1]; ++j)
                         for (py::ssize_t k = 0; k < (py::ssize_t)shape[2]; ++k)
                             w(i, j, k) = g[i][j][k];
                 return arr;
             },
             "Return control points as a (N0, N1, N2) numpy array.")

        // Bounds
        .def("set_bounds",
             [](Spline& s, const Eigen::Vector3d& lb, const Eigen::Vector3d& ub) {
                 s.setBounds(lb, ub);
             },
             py::arg("lower_bound"), py::arg("upper_bound"),
             "Set bounds as (3,) arrays [x_min, y_min, z_min] and [x_max, y_max, z_max].")
        .def("get_lower_bound",
             [](const Spline& s) -> Eigen::Vector3d { return s.getLowerBound(); })
        .def("get_upper_bound",
             [](const Spline& s) -> Eigen::Vector3d { return s.getUpperBound(); })
        .def("in_range",
             [](const Spline& s, const Eigen::Vector3d& pos) { return s.inRange(pos); },
             py::arg("pos"))

        // Evaluation
        .def("evaluate",
             [](const Spline& s, const Eigen::Vector3d& pos) -> double {
                 return s.evaluate(pos);
             },
             py::arg("pos"), "Evaluate the spline at pos (shape (3,)); returns scalar.")
        .def("derivative",
             [](const Spline& s, const Eigen::Vector3d& pos,
                const std::array<int, 3>& orders) -> double {
                 return s.derivative(pos, orders);
             },
             py::arg("pos"), py::arg("orders"),
             "Partial derivative. orders = [dx, dy, dz] e.g. [1,0,0] for df/dx.")
        .def("smoothness",
             [](const Spline& s, int d) -> double { return smoothness_dispatch(s, d); },
             py::arg("derivative"))

        .def("set_extrapolate", &Spline::setExtrapolate, py::arg("enable"))
        .def("is_extrapolating", &Spline::isExtrapolating)

        .def("__repr__", [name](const Spline&) {
            return std::string(name) + "()";
        });
}

// ---------------------------------------------------------------------------
// Bind a 3D→Nd spline: R³→R^OutputDims with EigenAlignedMultiArray control points.
// Python control points: numpy array of shape (N0, N1, N2, OutputDims).
// Python input: numpy array of shape (3,).
// ---------------------------------------------------------------------------
template <int Degree, int OutputDims>
void bind_3dNd(py::module& m, const char* name) {
    using Input  = Eigen::Vector3d;
    using Vec    = Eigen::Matrix<double, OutputDims, 1>;
    using Grid   = ubs::EigenAlignedMultiArray<Vec, 3>;
    using Spline = ubs::UniformBSpline<double, Degree, Input, Vec, Grid>;

    py::class_<Spline>(m, name)
        .def(py::init<>(),
             "Default constructor: [0,1]³ bounds, minimum control points (all zero).")

        // Control points: accept / return as (N0×N1×N2×OutputDims) numpy array
        .def("set_control_points",
             [](Spline& s, py::array_t<double, py::array::c_style> arr) {
                 if (arr.ndim() != 4 || arr.shape(3) != OutputDims)
                     throw std::invalid_argument(
                         "control_points must be shape (N0, N1, N2, " +
                         std::to_string(OutputDims) + ")");
                 Grid grid(boost::extents[arr.shape(0)][arr.shape(1)][arr.shape(2)]);
                 auto r = arr.unchecked<4>();
                 for (py::ssize_t i = 0; i < arr.shape(0); ++i)
                     for (py::ssize_t j = 0; j < arr.shape(1); ++j)
                         for (py::ssize_t k = 0; k < arr.shape(2); ++k) {
                             Vec v;
                             for (int d = 0; d < OutputDims; ++d) v[d] = r(i, j, k, d);
                             grid[i][j][k] = v;
                         }
                 s.setControlPoints(grid);
             },
             py::arg("control_points"),
             "Set control points from a (N0, N1, N2, OutputDims) numpy array.")
        .def("get_control_points",
             [](const Spline& s) {
                 const auto& g = s.getControlPoints();
                 const auto& shape = g.shape();
                 py::array_t<double> arr({(py::ssize_t)shape[0], (py::ssize_t)shape[1],
                                          (py::ssize_t)shape[2], (py::ssize_t)OutputDims});
                 auto w = arr.mutable_unchecked<4>();
                 for (py::ssize_t i = 0; i < (py::ssize_t)shape[0]; ++i)
                     for (py::ssize_t j = 0; j < (py::ssize_t)shape[1]; ++j)
                         for (py::ssize_t k = 0; k < (py::ssize_t)shape[2]; ++k)
                             for (int d = 0; d < OutputDims; ++d)
                                 w(i, j, k, d) = g[i][j][k][d];
                 return arr;
             },
             "Return control points as a (N0, N1, N2, OutputDims) numpy array.")

        // Bounds
        .def("set_bounds",
             [](Spline& s, const Eigen::Vector3d& lb, const Eigen::Vector3d& ub) {
                 s.setBounds(lb, ub);
             },
             py::arg("lower_bound"), py::arg("upper_bound"))
        .def("get_lower_bound",
             [](const Spline& s) -> Eigen::Vector3d { return s.getLowerBound(); })
        .def("get_upper_bound",
             [](const Spline& s) -> Eigen::Vector3d { return s.getUpperBound(); })
        .def("in_range",
             [](const Spline& s, const Eigen::Vector3d& pos) { return s.inRange(pos); },
             py::arg("pos"))

        // Evaluation
        .def("evaluate",
             [](const Spline& s, const Eigen::Vector3d& pos) -> Vec {
                 return s.evaluate(pos);
             },
             py::arg("pos"), "Evaluate; returns array of shape (OutputDims,).")
        .def("derivative",
             [](const Spline& s, const Eigen::Vector3d& pos,
                const std::array<int, 3>& orders) -> Vec {
                 return s.derivative(pos, orders);
             },
             py::arg("pos"), py::arg("orders"),
             "Partial derivative. orders = [dx, dy, dz].")
        .def("smoothness",
             [](const Spline& s, int d) -> Vec { return smoothness_dispatch(s, d); },
             py::arg("derivative"))

        .def("set_extrapolate", &Spline::setExtrapolate, py::arg("enable"))
        .def("is_extrapolating", &Spline::isExtrapolating)

        .def("__repr__", [name](const Spline&) {
            return std::string(name) + "()";
        });
}

// ---------------------------------------------------------------------------
// Module definition
// ---------------------------------------------------------------------------

//! [CustomBinding_Example]
// To expose a type combination that is not pre-instantiated, copy the
// relevant bind_* call into your own pybind11 module and register it under
// any name you like.  The helper templates (bind_1d1d, bind_1dNd, bind_3d1d,
// bind_3dNd) are defined above and cover the four supported input/output
// dimension families.
//
// Example: add R¹→R² (e.g. a 2D trajectory) at degree 3
//
//   bind_1dNd<3, 2>(m, "UniformBSpline1d2d3");
//
// Example: add R²→R¹ (height map) at degree 4 — requires a custom bind
// function for 2D input because bind_3d1d/bind_3dNd are hard-coded to R³:
//
//   using Grid2  = ubs::EigenAlignedMultiArray<double, 2>;
//   using Input2 = Eigen::Vector2d;
//   using S = ubs::UniformBSpline<double, 4, Input2, double, Grid2>;
//   py::class_<S>(m, "UniformBSpline2d1d4")
//       .def(py::init<>())
//       .def("evaluate",
//            [](const S& s, const Eigen::Vector2d& p) { return s.evaluate(p); },
//            py::arg("pos"));
//! [CustomBinding_Example]

PYBIND11_MODULE(uniform_bspline, m) {
    m.doc() = R"doc(
Python bindings for the uniform_bspline C++ library.

Exposed concrete types follow the naming convention:
  UniformBSpline<InputDims>d<OutputDims>d<Degree>

Supported types (degrees 1–5 for all):
  UniformBSpline1d1d{1..5} -- R¹ → R¹
  UniformBSpline1d3d{1..5} -- R¹ → R³
  UniformBSpline3d1d{1..5} -- R³ → R¹
  UniformBSpline3d2d{1..5} -- R³ → R²
)doc";

    // ------------------------------------------------------------------
    // R¹ → R¹  (scalar-valued splines)
    // ------------------------------------------------------------------
    bind_1d1d<1>(m, "UniformBSpline1d1d1");
    bind_1d1d<2>(m, "UniformBSpline1d1d2");
    bind_1d1d<3>(m, "UniformBSpline1d1d3");
    bind_1d1d<4>(m, "UniformBSpline1d1d4");
    bind_1d1d<5>(m, "UniformBSpline1d1d5");

    // ------------------------------------------------------------------
    // R¹ → R³  (3D position trajectories)
    // ------------------------------------------------------------------
    bind_1dNd<1, 3>(m, "UniformBSpline1d3d1");
    bind_1dNd<2, 3>(m, "UniformBSpline1d3d2");
    bind_1dNd<3, 3>(m, "UniformBSpline1d3d3");
    bind_1dNd<4, 3>(m, "UniformBSpline1d3d4");
    bind_1dNd<5, 3>(m, "UniformBSpline1d3d5");

    // ------------------------------------------------------------------
    // R³ → R¹  (3D scalar fields: cost, occupancy, distance fields)
    // ------------------------------------------------------------------
    bind_3d1d<1>(m, "UniformBSpline3d1d1");
    bind_3d1d<2>(m, "UniformBSpline3d1d2");
    bind_3d1d<3>(m, "UniformBSpline3d1d3");
    bind_3d1d<4>(m, "UniformBSpline3d1d4");
    bind_3d1d<5>(m, "UniformBSpline3d1d5");

    // ------------------------------------------------------------------
    // R³ → R²  (3D → 2D mappings: projections, UV maps, feature maps)
    // ------------------------------------------------------------------
    bind_3dNd<1, 2>(m, "UniformBSpline3d2d1");
    bind_3dNd<2, 2>(m, "UniformBSpline3d2d2");
    bind_3dNd<3, 2>(m, "UniformBSpline3d2d3");
    bind_3dNd<4, 2>(m, "UniformBSpline3d2d4");
    bind_3dNd<5, 2>(m, "UniformBSpline3d2d5");
}
