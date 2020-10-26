#include "control_points_container.hpp"

#include <gtest/gtest.h>

TEST(ControlPointsContainer, ForEachVector) {
    const std::vector<double> controlPoints{1.0, 4.0, 3.0, 2.0, 5.0};
    {
        const ubs::ControlPointsContainer<double, 4, double, double, std::vector<double>> controlPointsContainer(
            controlPoints);
        const auto& controlPointRef = controlPointsContainer.get();

        int counter = 0;
        controlPointsContainer.forEach([&](const double& val) {
            EXPECT_EQ(val, controlPoints[counter]);
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            EXPECT_EQ(&val, controlPointRef.data() + counter);
            ++counter;
        });
        EXPECT_EQ(counter, (int)controlPoints.size());
    }

    {
        ubs::ControlPointsContainer<double, 4, double, double, std::vector<double>> controlPointsContainer(
            controlPoints);
        const auto& controlPointRef = controlPointsContainer.get();

        int counter = 0;
        controlPointsContainer.forEach([&](double& val) {
            EXPECT_EQ(val, controlPoints[counter]);
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            EXPECT_EQ(&val, controlPointRef.data() + counter);
            ++counter;
        });
        EXPECT_EQ(counter, (int)controlPoints.size());
    }
}

TEST(ControlPointsContainer, ForEachEigenMatrix) {
    const Eigen::MatrixXd controlPoints = Eigen::MatrixXd::Random(10, 20);
    ubs::ControlPointsContainer<double, 4, Eigen::Vector2d, double, Eigen::MatrixXd> controlPointsContainer(
        controlPoints);
    const auto& controlPointRef = controlPointsContainer.get();

    int counter = 0;
    controlPointsContainer.forEach([&](double& val) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        EXPECT_EQ(val, controlPoints.data()[counter]);
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        EXPECT_EQ(&val, controlPointRef.data() + counter);
        ++counter;
    });
    EXPECT_EQ(counter, (int)controlPoints.size());
}

TEST(ControlPointsContainer, ForEachBoostMultiArray) {
    using ControlPointsContainer = ubs::ControlPointsContainer<double,
                                                               2,
                                                               Eigen::Vector3d,
                                                               Eigen::Vector4d,
                                                               ubs::EigenAlignedMultiArray<Eigen::Vector4d, 3>>;

    ControlPointsContainer::ControlPointsType controlPoints(boost::extents[10][12][8]);
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 12; ++j) {
            for (int k = 0; k < 8; ++k) {
                controlPoints[i][j][k].setRandom();
            }
        }
    }

    ControlPointsContainer controlPointsContainer(controlPoints);
    const auto& controlPointRef = controlPointsContainer.get();

    int counter = 0;
    controlPointsContainer.forEach([&](Eigen::Vector4d& val) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        EXPECT_EQ(val, controlPoints.data()[counter]);
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        EXPECT_EQ(&val, controlPointRef.data() + counter);
        ++counter;
    });
    EXPECT_EQ(counter, (int)controlPoints.num_elements());
}

TEST(ControlPointsContainer, Transform) {
    using ControlPointContainerInput = ubs::ControlPointsContainer<double,
                                                                   2,
                                                                   Eigen::Vector3d,
                                                                   Eigen::Vector4d,
                                                                   ubs::EigenAlignedMultiArray<Eigen::Vector4d, 3>>;

    ControlPointContainerInput::ControlPointsType controlPointsInput(boost::extents[10][12][8]);
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 12; ++j) {
            for (int k = 0; k < 8; ++k) {
                controlPointsInput[i][j][k].setRandom();
            }
        }
    }

    ControlPointContainerInput controlPointContainerInput(controlPointsInput);

    using ControlPointContainerOutput = ubs::ControlPointsContainer<float,
                                                                    2,
                                                                    Eigen::Vector3d,
                                                                    Eigen::Vector2d,
                                                                    ubs::EigenAlignedMultiArray<Eigen::Vector2d, 3>>;

    ControlPointContainerOutput::ControlPointsType controlPointsOutput(boost::extents[10][12][8]);
    std::for_each(
        controlPointsOutput.flat_begin(), controlPointsOutput.flat_end(), [](Eigen::Vector2d& v) { v.setZero(); });

    ControlPointContainerOutput controlPointContainerOutput(controlPointsOutput);

    controlPointContainerInput.transform(controlPointContainerOutput, [&](const Eigen::Vector4d& val) {
        return Eigen::Vector2d{val[0] + val[1], val[2] + val[3]};
    });

    int counter = 0;
    controlPointContainerOutput.forEach([&](const Eigen::Vector2d& v) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        const Eigen::Vector4d& inputVec = controlPointsInput.data()[counter];
        EXPECT_FLOAT_EQ(inputVec[0] + inputVec[1], v[0]);
        EXPECT_FLOAT_EQ(inputVec[2] + inputVec[3], v[1]);
        ++counter;
    });
}

namespace ubs {
// This test needs to be in the ubs namespace because of the friend declaration.
TEST(ControlPointsContainer, IsIdxValid) {
    using ControlPointContainerInput = ubs::ControlPointsContainer<double,
                                                                   2,
                                                                   Eigen::Vector3d,
                                                                   Eigen::Vector4d,
                                                                   ubs::EigenAlignedMultiArray<Eigen::Vector4d, 3>>;

    constexpr int SizeX = 10;
    constexpr int SizeY = 12;
    constexpr int SizeZ = 8;
    ControlPointContainerInput::ControlPointsType controlPointsInput(boost::extents[SizeX][SizeY][SizeZ]);
    for (int i = 0; i < SizeX; ++i) {
        for (int j = 0; j < SizeY; ++j) {
            for (int k = 0; k < SizeZ; ++k) {
                controlPointsInput[i][j][k].setRandom();
            }
        }
    }

    ControlPointContainerInput controlPointContainerInput(controlPointsInput);

    for (int i = 0; i < SizeX * SizeY * SizeZ; ++i) {
        EXPECT_TRUE(controlPointContainerInput.isIdxValid(i));
    }

    EXPECT_FALSE(controlPointContainerInput.isIdxValid(-1));
    EXPECT_FALSE(controlPointContainerInput.isIdxValid(SizeX * SizeY * SizeZ));
}
} // namespace ubs
