#include "uniform_bspline.hpp"

#include <fstream>
#include <Eigen/Core>

#include "gtest/gtest.h"
#include "test/test_utility.hpp"

namespace {
template <typename T>
using EigenAlignedVec = std::vector<T, Eigen::aligned_allocator<T>>;

auto testDataFolder() {
    return uniform_bspline::test::projectRootDir / "test" / "test_data";
}

std::string getSplineTestFile(int idx) {
    return (testDataFolder() / ("test_points_" + std::to_string(idx) + ".txt")).string();
}

std::string getSmoothnessTestFile(int idx) {
    return (testDataFolder() / ("test_points_smoothness_" + std::to_string(idx) + ".txt")).string();
}

template <int Derivative, typename Spline>
void readSpline(std::ifstream& file,
                Spline& spline,
                bool& hasDerivative,
                std::array<int, Spline::InputDims>& derivatives) {
    ASSERT_TRUE(file.is_open());

    // Read spline spline parameters.
    int degree = 0;
    int inputDims = 0;
    int outputDims = 0;
    int derivative = 0;
    file >> degree >> inputDims >> outputDims;

    ASSERT_EQ(Spline::Degree, degree) << "This is a compile time constant";
    ASSERT_EQ(Spline::InputDims, inputDims) << "This is a compile time constant";
    ASSERT_EQ(Spline::OutputDims, outputDims) << "This is a compile time constant";

    if (Derivative >= 0) {
        file >> derivative;
        ASSERT_EQ(Derivative, derivative) << "This is a compile time constant";
    } else {
        hasDerivative = false;
        derivatives = std::array<int, Spline::InputDims>{};
        for (int i = 0; i < Spline::InputDims; ++i) {
            file >> derivatives[i];
            ASSERT_GE(derivatives[i], 0);
            hasDerivative |= (derivatives[i] > 0);
        }
    }

    // Read control points.
    std::array<int, Spline::InputDims> controlPointDims{};
    for (auto& dims : controlPointDims) {
        file >> dims;
    }

    typename Spline::ControlPointsType points(controlPointDims);
    std::for_each(points.flat_begin(), points.flat_end(), [&file](typename Spline::OutputType& val) {
        for (int i = 0; i < val.size(); ++i) {
            file >> val[i];
        }
    });

    // Read lower and upper bounds.
    typename Spline::InputType lowerBound{};
    typename Spline::InputType upperBound{};
    for (int i = 0; i < Spline::InputDims; ++i) {
        file >> lowerBound[i];
        file >> upperBound[i];
    }

    // Construct the B Spline
    spline = Spline{lowerBound, upperBound, points};
}

template <typename Spline>
void readSpline(std::ifstream& file,
                Spline& spline,
                bool& hasDerivative,
                std::array<int, Spline::InputDims>& derivatives) {
    readSpline<-1>(file, spline, hasDerivative, derivatives);
}

template <int Derivative, typename Spline>
void readSpline(std::ifstream& file, Spline& spline) {
    std::array<int, Spline::InputDims> derivatives{};
    bool hasDerivative{};
    readSpline<Derivative>(file, spline, hasDerivative, derivatives);
}

template <typename Spline>
void testEvaluate(const std::string& filePath) {
    std::ifstream file(filePath);
    Spline spline{};
    bool hasDerivative{};
    std::array<int, Spline::InputDims> derivatives{};
    readSpline(file, spline, hasDerivative, derivatives);

    // Read test points and compare with Mathematica result.
    int numTestPoints = 0;
    file >> numTestPoints;
    ASSERT_GT(numTestPoints, 0);

    for (int testPointIdx = 0; testPointIdx < numTestPoints; ++testPointIdx) {
        typename Spline::InputType pos;
        for (int i = 0; i < pos.size(); ++i) {
            file >> pos[i];
        }

        ASSERT_TRUE(file.good());

        typename Spline::OutputType gtVal;
        for (int i = 0; i < gtVal.size(); ++i) {
            file >> gtVal[i];
        }

        typename Spline::OutputType splineVal{};

        if (!hasDerivative) {
            splineVal = spline.evaluate(pos);
        } else {
            splineVal = spline.derivative(pos, derivatives);
        }

        ASSERT_EQ(splineVal.size(), gtVal.size());

        for (int i = 0; i < splineVal.size(); ++i) {
            EXPECT_NEAR(gtVal[i], splineVal[i], 1e-10);
        }
    }
}

template <typename Spline, int Derivative>
void testSmoothnessEvaluation(const std::string& filePath) {
    std::ifstream file(filePath);
    Spline spline{};
    readSpline<Derivative>(file, spline);

    // Read test points and compare with mathematica result.
    using T = typename Spline::ValueType;
    T gtSmoothnessValue;
    file >> gtSmoothnessValue;
    T smoothnessValue = spline.template smoothness<Derivative>().sum();

    const auto bound = T(1e-5) * (T(1.0) + gtSmoothnessValue);
    EXPECT_NEAR(gtSmoothnessValue, smoothnessValue, bound) << "input file: " << filePath;
}
} // anonymous namespace

namespace ubs {
// This test needs to be in the ubs namespace because of the friend declaration.
TEST(UniformBSpline, DerivativeFactors) {
    ubs::EigenUniformBSpline<double, 4, 1, 1> spline;

    Eigen::Matrix<double, 4, 4> gtDerivativeFactors{};
    gtDerivativeFactors << 1, 0, 0, 0, 2, 2, 0, 0, 3, 6, 6, 0, 4, 12, 24, 24;

    Eigen::Matrix<double, 4, 4> derivativeFactors = spline.getDerivativeFactors();

    // The derivate factors are stored with reversed columns.
    derivativeFactors.colwise().reverseInPlace();

    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_EQ(gtDerivativeFactors(r, c), derivativeFactors(r, c));
        }
    }
}
} // namespace ubs

TEST(UniformBSpline, Evaluate) {
    testEvaluate<ubs::EigenUniformBSpline<double, 5, 1, 1>>(getSplineTestFile(1));
    testEvaluate<ubs::EigenUniformBSpline<double, 4, 1, 2>>(getSplineTestFile(2));
    testEvaluate<ubs::EigenUniformBSpline<double, 3, 2, 3>>(getSplineTestFile(3));
    testEvaluate<ubs::EigenUniformBSpline<double, 2, 3, 2>>(getSplineTestFile(4));
    testEvaluate<ubs::EigenUniformBSpline<double, 5, 1, 1>>(getSplineTestFile(5));
    testEvaluate<ubs::EigenUniformBSpline<double, 3, 2, 1>>(getSplineTestFile(6));
    testEvaluate<ubs::EigenUniformBSpline<double, 3, 2, 3>>(getSplineTestFile(7));
    testEvaluate<ubs::EigenUniformBSpline<double, 5, 1, 1>>(getSplineTestFile(8));
    testEvaluate<ubs::EigenUniformBSpline<double, 4, 1, 2>>(getSplineTestFile(9));
    testEvaluate<ubs::EigenUniformBSpline<double, 3, 2, 3>>(getSplineTestFile(10));
    testEvaluate<ubs::EigenUniformBSpline<double, 2, 3, 2>>(getSplineTestFile(11));
    testEvaluate<ubs::EigenUniformBSpline<double, 5, 1, 1>>(getSplineTestFile(12));
    testEvaluate<ubs::EigenUniformBSpline<double, 3, 2, 1>>(getSplineTestFile(13));
    testEvaluate<ubs::EigenUniformBSpline<double, 3, 2, 3>>(getSplineTestFile(14));
}

TEST(UniformBSpline, Smoothness) {
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 1, 1>, 0>(getSmoothnessTestFile(1));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 1, 1>, 1>(getSmoothnessTestFile(2));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 1, 2>, 1>(getSmoothnessTestFile(3));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 1, 1>, 1>(getSmoothnessTestFile(4));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 1, 1>, 2>(getSmoothnessTestFile(5));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 1>, 1>(getSmoothnessTestFile(6));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 1>, 1>(getSmoothnessTestFile(7));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 1>, 1>(getSmoothnessTestFile(8));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 1>, 0>(getSmoothnessTestFile(9));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 2>, 1>(getSmoothnessTestFile(10));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 2>, 2>(getSmoothnessTestFile(11));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 3, 2>, 2>(getSmoothnessTestFile(12));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 1, 1>, 0>(getSmoothnessTestFile(13));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 1, 1>, 1>(getSmoothnessTestFile(14));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 1, 2>, 1>(getSmoothnessTestFile(15));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 1, 1>, 1>(getSmoothnessTestFile(16));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 1, 1>, 2>(getSmoothnessTestFile(17));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 1>, 1>(getSmoothnessTestFile(18));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 1>, 1>(getSmoothnessTestFile(19));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 1>, 1>(getSmoothnessTestFile(20));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 1>, 0>(getSmoothnessTestFile(21));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 2>, 1>(getSmoothnessTestFile(22));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 2, 2>, 2>(getSmoothnessTestFile(23));
    testSmoothnessEvaluation<ubs::EigenUniformBSpline<double, 3, 3, 2>, 2>(getSmoothnessTestFile(24));
}

TEST(UniformBSpline, EigenControlPoints) {
    using Spline = ubs::UniformBSpline<double, 5, double, double, Eigen::VectorXd>;

    Spline spline(Eigen::VectorXd::Ones(50));

    const auto splineVal = spline.evaluate(0.5);
    EXPECT_NEAR(splineVal, 1.0, 1e-10);
}

TEST(UniformBSpline, EigenControlPoints2) {
    using Spline = ubs::UniformBSpline<double, 5, Eigen::Vector2d, double, Eigen::MatrixXd>;

    Spline spline(Eigen::MatrixXd::Ones(20, 15));

    const auto splineVal = spline.evaluate(Eigen::Vector2d(0.5, 0.1));
    EXPECT_NEAR(splineVal, 1.0, 1e-10);
}

TEST(UniformBSpline, DefaultConstructor) {
    ubs::UniformBSpline11d<4> spline;
    EXPECT_NEAR(0.0, spline.evaluate(0.0), 1e-10);
    EXPECT_NEAR(0.0, spline.evaluate(0.5), 1e-10);
    EXPECT_NEAR(0.0, spline.evaluate(1.0), 1e-10);
    EXPECT_EQ(spline.getLowerBound(0), 0.0);
    EXPECT_EQ(spline.getUpperBound(0), 1.0);
}

TEST(UniformBSpline, UniformBSpline1d) {
    using Spline = ubs::UniformBSpline<double, 5, double, double, Eigen::VectorXd>;
    Spline spline(Eigen::VectorXd::Constant(50, 20.0));

    const double val = spline.evaluate(0.3);
    const double derivative = spline.derivative(0.2, 2);
    const double smoothness = spline.smoothness<2>();

    EXPECT_NEAR(val, 20.0, 1e-10);
    EXPECT_NEAR(derivative, 0.0, 1e-10);
    EXPECT_NEAR(smoothness, 0.0, 1e-6);

    spline.setBounds(2.0, 9.5);

    const double lowerBound = spline.getLowerBound();
    const double upperBound = spline.getUpperBound();
    const double scaleVal = spline.getScale(0);
    const double scaleValGt = (50.0 - 5.0) / (9.5 - 2.0);

    EXPECT_EQ(lowerBound, 2.0);
    EXPECT_EQ(upperBound, 9.5);
    EXPECT_NEAR(scaleVal, scaleValGt, 1e-10);
}

namespace {
template <typename Spline>
void checkSplineBounds(const Spline& spline, const Eigen::Vector2d& lowerBound, const Eigen::Vector2d& upperBound) {
    static_assert(Spline::InputDims == 2, "Unsupported input dimensions.");
    for (int i = 0; i < Spline::InputDims; ++i) {
        EXPECT_EQ(spline.getLowerBound(i), lowerBound[i]);
        EXPECT_EQ(spline.getUpperBound(i), upperBound[i]);

        EXPECT_EQ(spline.getLowerBound()[i], lowerBound[i]);
        EXPECT_EQ(spline.getUpperBound()[i], upperBound[i]);
    }
}
} // namespace

TEST(UniformBSpline, Bounds) {
    {
        ubs::EigenUniformBSpline<double, 4, 2, 3> spline;
        checkSplineBounds(spline, {0.0, 0.0}, {1.0, 1.0});

        spline.setBounds({-2.0, -3.0}, {7.0, 8.0});
        checkSplineBounds(spline, {-2.0, -3.0}, {7.0, 8.0});

        EXPECT_TRUE(spline.inRange({0.0, 0.0}));
        EXPECT_TRUE(spline.inRange({-2.0, -3.0}));
        EXPECT_TRUE(spline.inRange({7.0, 8.0}));

        EXPECT_FALSE(spline.inRange({-3.0, 0.0}));
        EXPECT_FALSE(spline.inRange({8.0, 0.0}));
        EXPECT_FALSE(spline.inRange({0.0, -4.0}));
        EXPECT_FALSE(spline.inRange({0.0, 10.0}));
        EXPECT_FALSE(spline.inRange({-10.0, 20.0}));
    }
    {
        ubs::EigenUniformBSpline<double, 4, 2, 3> spline({-2.0, -3.0}, {7.0, 8.0});
        checkSplineBounds(spline, {-2.0, -3.0}, {7.0, 8.0});
    }
    {
        using Spline = ubs::UniformBSpline<double, 4, Eigen::Vector2d, double, Eigen::MatrixXd>;
        Eigen::MatrixXd controlPoints = Eigen::MatrixXd::Random(30, 25);
        Spline spline({-2.0, -3.0}, {7.0, 8.0}, controlPoints);
        checkSplineBounds(spline, {-2.0, -3.0}, {7.0, 8.0});
    }
}

TEST(UniformBSpline, Extrapolation) {
    using Spline = ubs::UniformBSpline<double, 4, double, double, std::vector<double>>;
    const std::vector<double> controlPoints{1.0, 4.0, 3.0, 10.0, 15.0};

    Spline spline(controlPoints);
    EXPECT_FALSE(spline.isExtrapolating());
    spline.setExtrapolate(true);
    EXPECT_TRUE(spline.isExtrapolating());

    {
        auto res = spline.getSpanIndexAndValue(0, -1.0);
        EXPECT_EQ(res.first, 0);
        EXPECT_DOUBLE_EQ(res.second, -1.0);
    }
    {
        auto res = spline.getSpanIndexAndValue(0, 1.5);
        EXPECT_EQ(res.first, 0);
        EXPECT_DOUBLE_EQ(res.second, 1.5);
    }
}

TEST(UniformBSpline, Cast) {
    using SplineIn = ubs::UniformBSpline<double, 4, double, Eigen::Vector2d, EigenAlignedVec<Eigen::Vector2d>>;

    EigenAlignedVec<Eigen::Vector2d> controlPoints(10);
    for (auto& c : controlPoints) {
        c.setRandom();
        c = (1000.0 * c).array().floor();
    }
    SplineIn splineIn(2.0, 10.0, controlPoints);

    using SplineOut = ubs::UniformBSpline<int, 3, int, Eigen::Vector2i, EigenAlignedVec<Eigen::Vector2i>>;

    splineIn.setExtrapolate(true);
    const auto splineOut = splineIn.cast<SplineOut>();

    EXPECT_EQ(splineIn.getLowerBound(0), splineOut.getLowerBound(0));
    EXPECT_EQ(splineIn.getUpperBound(0), splineOut.getUpperBound(0));
    EXPECT_EQ(splineIn.isExtrapolating(), splineOut.isExtrapolating());

    const auto& controlPointsIn = splineIn.getControlPoints();
    const auto& controlPointsOut = splineOut.getControlPoints();

    ASSERT_EQ(controlPointsIn.size(), controlPointsOut.size());

    for (int i = 0; i < int(controlPointsIn.size()); ++i) {
        EXPECT_EQ(controlPointsIn[i].cast<int>(), controlPointsOut[i]);
    }

    // Check self cast.
    EXPECT_EQ(&splineIn.cast<SplineIn>(), &splineIn);
}

TEST(UniformBSpline, GetSpanIndices) {
    using Spline = ubs::UniformBSpline<double, 5, Eigen::Vector2d, double, Eigen::MatrixXd>;
    Spline spline(Eigen::MatrixXd::Ones(20, 15));

    const Eigen::Vector2d pos{0.3, 0.5};
    const auto indices = spline.getSpanIndices(pos);

    ASSERT_EQ(indices.size(), pos.size());
    for (int i = 0; i < int(pos.size()); ++i) {
        EXPECT_EQ(indices[i], spline.getSpanIndex(i, pos[i]));
    }
}
