#include "uniform_bspline.hpp"

#include <iostream>

#include <Eigen/Core>
#include <gtest/gtest.h>

template <typename T>
using EigenAlignedVec = std::vector<T, Eigen::aligned_allocator<T>>;

TEST(UniformBSplineExample, Spline1d1d) {
    // clang-format off
    {
    //! [Spline1d1d_Definition]
    ubs::UniformBSpline<double, 3, double, double, std::vector<double>> spline;
    //! [Spline1d1d_Definition]
    }
    // clang-format on

    //! [Spline1d1d_DefinitionShort]
    ubs::UniformBSpline11d<3> spline;
    //! [Spline1d1d_DefinitionShort]

    //! [Spline1d1d_ControlPoints]
    std::vector<double> controlPoints{0.0, 1.0, 2.0, 3.0, 4.0};
    spline.setControlPoints(controlPoints);
    //! [Spline1d1d_ControlPoints]

    //! [Spline1d1d_Eval]
    double val0 = spline.evaluate(0.0);
    double val1 = spline.evaluate(1.0);
    //! [Spline1d1d_Eval]
    EXPECT_DOUBLE_EQ(1.0, val0);
    EXPECT_DOUBLE_EQ(3.0, val1);

    //! [Spline1d1d_Derivative]
    double deriv1 = spline.derivative(0.0, 1);
    double deriv2 = spline.derivative(1.0, 2);
    //! [Spline1d1d_Derivative]
    EXPECT_DOUBLE_EQ(2.0, deriv1);
    EXPECT_DOUBLE_EQ(0.0, deriv2);

    //! [Spline1d1d_Smoothness]
    double smoothness = spline.smoothness<1>();
    //! [Spline1d1d_Smoothness]
    EXPECT_DOUBLE_EQ(4.0, smoothness);

    //! [Spline1d1d_Bounds]
    spline.setBounds(-2.0, 5.0);
    //! [Spline1d1d_Bounds]
    EXPECT_EQ(spline.getLowerBound(), -2.0);
    EXPECT_EQ(spline.getUpperBound(), 5.0);
}

TEST(UniformBSplineExample, Spline1d3d) {
    //! [Spline1d3d_Definition]
    ubs::UniformBSpline<double, 3, double, Eigen::Vector3d, EigenAlignedVec<Eigen::Vector3d>> spline;
    //! [Spline1d3d_Definition]

    //! [Spline1d3d_Eval]
    Eigen::Vector3d val0 = spline.evaluate(0.0);
    Eigen::Vector3d val1 = spline.evaluate(1.0);
    //! [Spline1d3d_Eval]
    EXPECT_DOUBLE_EQ(0.0, val0.norm());
    EXPECT_DOUBLE_EQ(0.0, val1.norm());

    //! [Spline1d3d_Derivative]
    Eigen::Vector3d deriv1 = spline.derivative(0.0, 1);
    Eigen::Vector3d deriv2 = spline.derivative(1.0, 2);
    //! [Spline1d3d_Derivative]
    EXPECT_DOUBLE_EQ(0.0, deriv1.norm());
    EXPECT_DOUBLE_EQ(0.0, deriv2.norm());

    //! [Spline1d3d_Smoothness]
    Eigen::Vector3d smoothness = spline.smoothness<1>();
    //! [Spline1d3d_Smoothness]
    EXPECT_DOUBLE_EQ(0.0, smoothness.sum());
}

TEST(UniformBSplineExample, Spline2d1d) {
    //! [Spline2d1d_Definition]
    ubs::UniformBSpline<double, 3, Eigen::Vector2d, double, Eigen::MatrixXd> spline;
    //! [Spline2d1d_Definition]

    //! [Spline2d1d_Evaluate_Long]
    double val = spline.evaluate(Eigen::Vector2d{0.0, 0.0});
    //! [Spline2d1d_Evaluate_Long]
    EXPECT_DOUBLE_EQ(0.0, val);

    //! [Spline2d1d_Evaluate_Short]
    val = spline.evaluate({0.0, 0.0});
    //! [Spline2d1d_Evaluate_Short]
    EXPECT_DOUBLE_EQ(0.0, val);

    //! [Spline2d1d_Derivative_10]
    double deriv10 = spline.derivative({0.0, 0.0}, {1, 0});
    //! [Spline2d1d_Derivative_10]
    EXPECT_DOUBLE_EQ(0.0, deriv10);

    //! [Spline2d1d_Derivative_11]
    double deriv11 = spline.derivative({0.0, 0.0}, {1, 1});
    //! [Spline2d1d_Derivative_11]
    EXPECT_DOUBLE_EQ(0.0, deriv11);
}

TEST(UniformBSplineExample, Spline2d3d) {
    //! [Spline2d3d_Definition]
    ubs::UniformBSpline<double, 3, Eigen::Vector2d, Eigen::Vector3d, ubs::EigenAlignedMultiArray<Eigen::Vector3d, 2>>
        spline;
    //! [Spline2d3d_Definition]

    //! [Spline2d3d_Definition_Short]
    ubs::EigenUniformBSpline<double, 3, 2, 3> splineS;
    //! [Spline2d3d_Definition_Short]

    //! [Spline2d3d_Evaluate]
    Eigen::Vector3d val = spline.evaluate({0.0, 0.2});
    //! [Spline2d3d_Evaluate]
    EXPECT_DOUBLE_EQ(0.0, val.norm());
}
