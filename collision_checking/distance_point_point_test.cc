// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "collision_checking/distance_point_point.h"

#include "collision_checking/eigenmath.h"
#include "collision_checking/test_utils.h"
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
