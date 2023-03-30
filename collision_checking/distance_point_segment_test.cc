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

#include "collision_checking/distance_point_segment.h"

#include <iterator>
#include <limits>

#include "absl/flags/flag.h"
#include "benchmark/benchmark.h"
#include "collision_checking/debug_options.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/test_utils.h"
#include "eigenmath/distribution.h"
#include "eigenmath/interpolation.h"
#include "eigenmath/sampling.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

ABSL_FLAG(bool, verbose, false, "Turn on verbose text logging.");

namespace collision_checking {
namespace {
template <typename Scalar>
using Vector3 = Vector3<Scalar>;

template <typename T>
class DistancePointSegmentTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistancePointSegmentTest);

TYPED_TEST_P(DistancePointSegmentTest, CompareAgainstQP) {
  // The squared distance between a point and a segment is the solution to the
  // following box constrained quadratic program:
  //   (segment_center + segment_direction*u-point)^2 -> min,
  //     s.t. |u| <= half_length.
  // This test compares the output of DistanceSquared with solutions
  // computed numerically using the QP solver.

  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  constexpr Scalar kQPTolerance = 100 * std::numeric_limits<Scalar>::epsilon();
  constexpr Scalar kDistanceSquaredTolerance = 10 * kQPTolerance;
  constexpr int kNumLoops = 100;

  eigenmath::TestGenerator gen(eigenmath::kGeneratorTestSeed);
  eigenmath::UniformDistributionVector<Scalar, 3> vector_dist;
  constexpr Scalar kMinHalfLength = 0.0;
  constexpr Scalar kMaxHalfLength = 0.5;
  std::uniform_real_distribution<Scalar> length_dist(kMinHalfLength,
                                                     kMaxHalfLength);
  const Vector3 kMinPointPos{-1.0, -1.0, -1.0};
  const Vector3 kMaxPointPos{1.0, 1.0, 1.0};

  eigenmath::MatrixXd cost_matrix(1, 1);
  eigenmath::VectorXd cost_vector(1);
  eigenmath::VectorXd lower_bound(1);
  eigenmath::VectorXd upper_bound(1);

  for (int loop = 0; loop < kNumLoops; loop++) {
    SCOPED_TRACE(::testing::Message() << "At loop " << loop);
    ABSL_LOG(INFO) << "==== loop= " << loop;

    Segment<Scalar> segment = {
        .center = eigenmath::InterpolateLinearInBox(vector_dist(gen),
                                                    kMinPointPos, kMaxPointPos),
        .direction = eigenmath::InterpolateLinearInBox(
                         vector_dist(gen), kMinPointPos, kMaxPointPos)
                         .normalized(),
        .half_length = length_dist(gen)};
    Point<Scalar> point = {eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos)};

    CC_MALLOC_COUNTER_INIT();
    const auto [distance_squared, u] = DistanceSquared(point, segment);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

    lower_bound[0] = -segment.half_length;
    upper_bound[0] = segment.half_length;

    cost_matrix(0, 0) = 2.0;
    cost_vector(0) =
        2.0 * (segment.center - point.center).dot(segment.direction);

    const auto qp_sol = testing::SolveBoxQPBruteForce(cost_matrix, cost_vector,
                                                      lower_bound, upper_bound);
    const Scalar qp_distance_squared =
        static_cast<Scalar>(qp_sol.minimum) +
        (segment.center - point.center).squaredNorm();

    ABSL_LOG(INFO) << "QP: solution: " << qp_sol.solution.transpose() << "\n"
                   << "QP: minimum: " << qp_sol.minimum << "\n"
                   << "QP: distance squared: " << qp_distance_squared << "\n"
                   << "distance_squared: " << distance_squared << "\n";
    ASSERT_NEAR(qp_distance_squared, distance_squared,
                kDistanceSquaredTolerance);
  }
}

REGISTER_TYPED_TEST_SUITE_P(DistancePointSegmentTest, CompareAgainstQP);

typedef ::testing::Types<float, double> FPTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(DistancePointSegmentTestsuite,
                               DistancePointSegmentTest, FPTypes);

template <typename T>
class DistancePointSegmentDeathTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistancePointSegmentDeathTest);

TYPED_TEST_P(DistancePointSegmentDeathTest, InvalidInput) {
  using Scalar = TypeParam;

  const Point<Scalar> kPoint{{0, 0, 0}};
  const Vector3<Scalar> kValidDirection(1, 0, 0);
  const Vector3<Scalar> kInValidDirection(10, 0, 0);
  constexpr Scalar kValidHalfLength = Scalar{1};
  constexpr Scalar kInValidHalfLength = Scalar{-1};

  // By default, input preconditions are not checked.
  // The following test with kInvalidHalfLength is disabled, as DistanceSquared
  // currently uses std::clamp internally, whose behavior is undefined for these
  // inputs and will assert if --features=enable_libcxx_assertions is set.
  // Enable it if the implementation stops using std::clamp.
  // DistanceSquared(kPoint, {kPoint.center, kValidDirection,
  // kInValidHalfLength});
  DistanceSquared(kPoint, {kPoint.center, kInValidDirection, kValidHalfLength});

  // If specified, preconditions on inputs are asserted.
  EXPECT_DEATH(
      (DistanceSquared<Scalar,
                       DebugOptions::kDebugOptionsPerformExpensiveInputChecks>(
          kPoint, {kPoint.center, kValidDirection, kInValidHalfLength})),
      "segment.half_length");
  EXPECT_DEATH(
      (DistanceSquared<Scalar,
                       DebugOptions::kDebugOptionsPerformExpensiveInputChecks>(
          kPoint, {kPoint.center, kInValidDirection, kValidHalfLength})),
      "squaredNorm");
}

REGISTER_TYPED_TEST_SUITE_P(DistancePointSegmentDeathTest, InvalidInput);

INSTANTIATE_TYPED_TEST_SUITE_P(DistancePointSegmentDeathTestSuite,
                               DistancePointSegmentDeathTest, FPTypes);

}  // namespace
}  // namespace collision_checking
