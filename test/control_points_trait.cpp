#include "control_points_trait.hpp"
#include "control_points_trait_eigen.hpp"
#include "control_points_trait_multi_array.hpp"
#include "control_points_trait_std_container.hpp"

#include <random>

#include <gtest/gtest.h>

TEST(EigenControlPointsTrait, Resize) {
    {
        Eigen::VectorXd m{};

        std::array<int, 1> dims{10};
        ubs::ControlPointsTrait<Eigen::VectorXd>::resize(m, dims);

        EXPECT_EQ(m.size(), dims[0]);
    }

    {
        Eigen::MatrixXd m{};

        std::array<int, 2> dims{10, 15};
        ubs::ControlPointsTrait<Eigen::MatrixXd>::resize(m, dims);

        EXPECT_EQ(m.cols(), dims[0]);
        EXPECT_EQ(m.rows(), dims[1]);
    }
}

TEST(ControlPointsTrait, EigenVectorStride) {
    using Container = Eigen::VectorXd;
    Container m = Eigen::VectorXd::Random(20);

    int stride = ubs::ControlPointsTrait<Container>::getStride(m, 0);
    double* data = ubs::ControlPointsTrait<Container>::data(m);
    ASSERT_EQ(data, m.data());

    int idx = 0;
    for (int i = 0; i < m.cols(); ++i, idx += stride) {
        ASSERT_LT(idx, m.size());
        EXPECT_EQ(m[i], data[idx]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
}

namespace {
void testEigenMatrixStride(int numPointsX, int numPointsY) {
    using Container = Eigen::MatrixXd;

    Container m;
    ubs::ControlPointsTrait<Container>::resize(m, std::array<int, 2>{numPointsX, numPointsY});
    m.setRandom();

    ASSERT_EQ(m.cols(), numPointsX);
    ASSERT_EQ(m.rows(), numPointsY);

    std::array<int, 2> strides{};
    strides[0] = ubs::ControlPointsTrait<Container>::getStride(m, 0);
    strides[1] = ubs::ControlPointsTrait<Container>::getStride(m, 1);

    double* data = ubs::ControlPointsTrait<Container>::data(m);
    ASSERT_EQ(data, m.data());

    int idx1 = 0;
    for (int c = 0; c < m.cols(); ++c, idx1 += strides[0]) {
        int idx2 = idx1;

        for (int r = 0; r < m.rows(); ++r, idx2 += strides[1]) {
            ASSERT_LT(idx2, m.size());
            EXPECT_EQ(m(r, c), data[idx2]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
    }
}
} // namespace

TEST(ControlPointsTrait, EigenMatrixStride) {
    testEigenMatrixStride(13, 17);
    testEigenMatrixStride(17, 13);
    testEigenMatrixStride(19, 19);
}

TEST(ControlPointsTrait, MultiArrayMatrixStride) {
    using Container = ubs::MultiArray<double, 2>;
    Container m(boost::extents[10][20]);

    std::mt19937 rng;
    std::uniform_real_distribution<> dist(-1.0, 1.0);
    std::for_each(m.flat_begin(), m.flat_end(), [&](double& elem) { elem = dist(rng); });

    std::array<int, 2> strides{};
    strides[0] = ubs::ControlPointsTrait<Container>::getStride(m, 0);
    strides[1] = ubs::ControlPointsTrait<Container>::getStride(m, 1);

    double* data = ubs::ControlPointsTrait<Container>::data(m);
    ASSERT_EQ(data, m.data());

    int idx1 = 0;
    for (int r = 0; r < int(m.get_size(0)); ++r, idx1 += strides[0]) {
        int idx2 = idx1;

        for (int c = 0; c < int(m.get_size(1)); ++c, idx2 += strides[1]) {
            ASSERT_LT(idx2, m.num_elements());
            EXPECT_EQ(m[r][c], data[idx2]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
    }
}
