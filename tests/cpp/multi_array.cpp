#include "multi_array.hpp"

#include <random>

#include <gtest/gtest.h>

namespace {
class MultiArrayTest : public ::testing::Test {
public:
    MultiArrayTest() : controlPoints(boost::extents[NumU_][NumV_]), gtData_(NumU_ * NumV_) {
        std::mt19937 rng;
        std::for_each(gtData_.begin(), gtData_.end(), [&rng](double& v) { v = rng(); });
        std::copy(gtData_.begin(), gtData_.end(), controlPoints.data());
    }

protected:
    void check(const ubs::MultiArray<double, 2>& other) const {
        ASSERT_EQ(NumU_, int(other.get_size(0)));
        ASSERT_EQ(NumV_, int(other.get_size(1)));

        for (int i = 0; i < int(gtData_.size()); ++i) {
            EXPECT_EQ(gtData_[i], other.data()[i]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
    }

    // NOLINTNEXTLINE(cppcoreguidelines-non-private-member-variables-in-classes)
    ubs::MultiArray<double, 2> controlPoints{};

private:
    static constexpr int NumU_ = 10;
    static constexpr int NumV_ = 15;
    std::vector<double> gtData_;
};

#if not defined(__cplusplus) or __cplusplus < 201703L
constexpr int MultiArrayTest::NumU_;
constexpr int MultiArrayTest::NumV_;
#endif

TEST_F(MultiArrayTest, CopyConstructor) {
    ubs::MultiArray<double, 2> other(controlPoints);
    check(other);
}

TEST_F(MultiArrayTest, CopyAssignmentOperator) {
    ubs::MultiArray<double, 2> other;
    other = controlPoints;
    check(other);
}

TEST_F(MultiArrayTest, MoveConstructor) {
    ubs::MultiArray<double, 2> other(std::move(controlPoints));
    check(other);
}

TEST_F(MultiArrayTest, MoveAssignmentOperator) {
    ubs::MultiArray<double, 2> other;
    other = std::move(controlPoints);
    check(other);
}

TEST_F(MultiArrayTest, IsContinuous) {
    {
        ubs::MultiArray<double, 1> c(boost::extents[7]);
        EXPECT_TRUE(c.is_continuous());
    }
    {
        ubs::MultiArray<double, 2> c(boost::extents[7][13]);
        EXPECT_TRUE(c.is_continuous());
    }
    {
        ubs::MultiArray<double, 3> c(boost::extents[17][13][29]);
        EXPECT_TRUE(c.is_continuous());
    }
}

TEST_F(MultiArrayTest, FlatIterators) {
    const auto& constControlPoints = controlPoints;

    EXPECT_EQ(controlPoints[0][0], *controlPoints.flat_begin());
    EXPECT_EQ(controlPoints[0][0], *constControlPoints.flat_begin());
    EXPECT_EQ(controlPoints[0][0], *controlPoints.flat_cbegin());

    EXPECT_EQ(controlPoints[controlPoints.get_size(0) - 1][controlPoints.get_size(1) - 1],
              *(controlPoints.flat_end() - 1)); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    EXPECT_EQ(controlPoints[controlPoints.get_size(0) - 1][controlPoints.get_size(1) - 1],
              *(constControlPoints.flat_end() - 1)); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    EXPECT_EQ(controlPoints[controlPoints.get_size(0) - 1][controlPoints.get_size(1) - 1],
              *(controlPoints.flat_cend() - 1)); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

} // namespace
