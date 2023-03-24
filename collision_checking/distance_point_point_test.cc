#include "collision_checking/distance_point_point.h"

#include "collision_checking/test_utils.h"
#include "collision_checking/eigenmath.h"
#include "gtest/gtest.h"

namespace collision_checking {
namespace {
template <typename T>
class DistancePointPointTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistancePointPointTest);

TYPED_TEST_P(DistancePointPointTest, ValidCase) {
  using Scalar = TypeParam;
  constexpr Scalar kEpsilon = 100 * std::numeric_limits<Scalar>::epsilon();
  const Point<Scalar> point1{{1.1, 1.2, 1.3}};
  const Point<Scalar> point2{{5.1, 6.2, -7.3}};

  CC_MALLOC_COUNTER_INIT();
  const auto result = DistanceSquared(point1, point2);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  constexpr Scalar kExpectedDistance = Scalar{114.96};
  EXPECT_NEAR(result.distance_squared, kExpectedDistance, kEpsilon);
}

REGISTER_TYPED_TEST_SUITE_P(DistancePointPointTest, ValidCase);

typedef ::testing::Types<float, double> FPTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(DistancePrimitiveTestSuite,
                               DistancePointPointTest, FPTypes);

}  // namespace
}  // namespace collision_checking

