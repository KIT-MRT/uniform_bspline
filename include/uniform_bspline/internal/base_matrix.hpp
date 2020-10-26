// Do not edit this file. This file was generated by a script:
// 'generate_bspline_basis_cpp_files 0 5 3'
#pragma once
#include <Eigen/Core>

namespace ubs {
/** @brief Internal namespace */
namespace internal {
template <int Order_>
struct UniformBSplineBasis;

template <>
struct UniformBSplineBasis<1> {
    static Eigen::Matrix<double, 1, 1> matrix() {
        Eigen::Matrix<double, 1, 1> m{};
        m << 1.;
        return m;
    }
};

template <>
struct UniformBSplineBasis<2> {
    static Eigen::Matrix<double, 2, 2> matrix() {
        Eigen::Matrix<double, 2, 2> m{};
        m << -1., 1., 1., 0;
        return m;
    }
};

template <>
struct UniformBSplineBasis<3> {
    static Eigen::Matrix<double, 3, 3> matrix() {
        Eigen::Matrix<double, 3, 3> m{};
        m << 0.5, -1., 0.5, -1., 1., 0.5, 0.5, 0, 0;
        return m;
    }
};

template <>
struct UniformBSplineBasis<4> {
    static Eigen::Matrix<double, 4, 4> matrix() {
        Eigen::Matrix<double, 4, 4> m{};
        m << -0.16666666666666666667, 0.5, -0.5, 0.16666666666666666667, 0.5, -1., 0, 0.66666666666666666667, -0.5, 0.5,
            0.5, 0.16666666666666666667, 0.16666666666666666667, 0, 0, 0;
        return m;
    }
};

template <>
struct UniformBSplineBasis<5> {
    static Eigen::Matrix<double, 5, 5> matrix() {
        Eigen::Matrix<double, 5, 5> m{};
        m << 0.041666666666666666667, -0.16666666666666666667, 0.25, -0.16666666666666666667, 0.041666666666666666667,
            -0.16666666666666666667, 0.5, -0.25, -0.5, 0.45833333333333333333, 0.25, -0.5, -0.25, 0.5,
            0.45833333333333333333, -0.16666666666666666667, 0.16666666666666666667, 0.25, 0.16666666666666666667,
            0.041666666666666666667, 0.041666666666666666667, 0, 0, 0, 0;
        return m;
    }
};

template <>
struct UniformBSplineBasis<6> {
    static Eigen::Matrix<double, 6, 6> matrix() {
        Eigen::Matrix<double, 6, 6> m{};
        m << -0.0083333333333333333333, 0.041666666666666666667, -0.083333333333333333333, 0.083333333333333333333,
            -0.041666666666666666667, 0.0083333333333333333333, 0.041666666666666666667, -0.16666666666666666667,
            0.16666666666666666667, 0.16666666666666666667, -0.41666666666666666667, 0.21666666666666666667,
            -0.083333333333333333333, 0.25, 0, -0.5, 0, 0.55, 0.083333333333333333333, -0.16666666666666666667,
            -0.16666666666666666667, 0.16666666666666666667, 0.41666666666666666667, 0.21666666666666666667,
            -0.041666666666666666667, 0.041666666666666666667, 0.083333333333333333333, 0.083333333333333333333,
            0.041666666666666666667, 0.0083333333333333333333, 0.0083333333333333333333, 0, 0, 0, 0, 0;
        return m;
    }
};

} // namespace internal
} // namespace ubs