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

#include "collision_checking/distance_segment_segment.h"

#include <algorithm>
#include <iomanip>
#include <ios>
#include <iterator>
#include <string>

#include "absl/flags/flag.h"
#include "absl/log/absl_log.h"
#include "collision_checking/debug_options.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/test_utils.h"
#include "eigenmath/distribution.h"
#include "eigenmath/interpolation.h"
#include "eigenmath/sampling.h"
#include "gtest/gtest.h"

ABSL_FLAG(bool, verbose, false, "Turn on verbose text logging.");

namespace collision_checking {
namespace {

template <typename T>
class DistanceSegmentSegmentTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistanceSegmentSegmentTest);

TYPED_TEST_P(DistanceSegmentSegmentTest, CompareAgainstQP) {
  // The squared distance between two segments is the solution to the
  // following box constrained quadratic program:
  //   (segment_a.center + segment_a.direction*u -
  //    segment_b.center - segment_b.direction*v)^2 -> min,
  //   s.t. |u| <= segment_a.half_length, and
  //        |v| <= segment_b.half_length.
  // This test  compares the output of DistanceSquared with
  // solutions computed numerically using the QP solver.

  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  // This is driven by the error of the determinant compuated in
  // SegmentSegmentDistanceSquared.
  const Scalar kDistanceSquaredTolerance = 5e-4;

  // Number of quasi random setups to run. Chosen  ~10x what is required to
  // hit all cases in SegmentSegmentDistanceSquared().
  constexpr int kNumLoops = 10000;
  // Pseudo-random numbers take a lot of samples to hit the parallel lines
  // cases, so force those occasionally.
  constexpr int kParallelLinesEvery = 30;
  // A constant controlling generation of not-quite parallel directions.
  constexpr Scalar kSmallDirectionComponent = 1e-4;
  eigenmath::TestGenerator gen(eigenmath::kGeneratorTestSeed);
  eigenmath::UniformDistributionVector<Scalar, 3> vector_dist;
  constexpr double kMinHalfLength = 0.0;
  constexpr double kMaxHalfLength = 0.5;
  std::uniform_real_distribution<Scalar> length_dist(kMinHalfLength,
                                                     kMaxHalfLength);
  const Vector3 kMinPointPos{-1.0, -1.0, -1.0};
  const Vector3 kMaxPointPos{1.0, 1.0, 1.0};

  eigenmath::MatrixXd cost_matrix(2, 2);
  eigenmath::VectorXd cost_vector(2);
  eigenmath::VectorXd lower_bound(2);
  eigenmath::VectorXd upper_bound(2);

  double max_diff = 0.0;

  for (int loop = 0; loop < kNumLoops; loop++) {
    SCOPED_TRACE(::testing::Message() << "At loop " << loop);
    ABSL_LOG(INFO) << "==== loop= " << loop;
    const Segment<Scalar> segment_a{
        .center = eigenmath::InterpolateLinearInBox(vector_dist(gen),
                                                    kMinPointPos, kMaxPointPos),
        .direction = eigenmath::InterpolateLinearInBox(
                         vector_dist(gen), kMinPointPos, kMaxPointPos)
                         .normalized(),
        .half_length = length_dist(gen)};

    Segment<Scalar> segment_b;
    segment_b.half_length = length_dist(gen);
    if (loop % kParallelLinesEvery == 0) {
      // Force exactly parallel lines.
      segment_b.direction = segment_a.direction;
    } else if (loop % kParallelLinesEvery == 1) {
      // Force exactly anti-parallel lines.
      segment_b.direction = -segment_a.direction;
    } else if (loop % kParallelLinesEvery == 2) {
      // Force nearly parallel lines.
      segment_b.direction =
          (segment_a.direction +
           vector_dist(gen).normalized() * kSmallDirectionComponent)
              .normalized();
    } else if (loop % kParallelLinesEvery == 3) {
      // Force nearly anti-parallel lines.
      segment_b.direction =
          (-segment_a.direction +
           vector_dist(gen).normalized() * kSmallDirectionComponent)
              .normalized();
    } else {
      // Regular case: pseudo-random direction.
      segment_b.direction = vector_dist(gen).normalized();
    }
    segment_b.center = eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos);

    CC_MALLOC_COUNTER_INIT();
    const auto [distance_squared, u, v] =
        SegmentSegmentDistanceSquared(segment_a, segment_b);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

    lower_bound << -segment_a.half_length, -segment_b.half_length;
    upper_bound << segment_a.half_length, segment_b.half_length;

    Vector3d dir_a = segment_a.direction.template cast<double>().normalized();
    Vector3d cen_a = segment_a.center.template cast<double>();
    Vector3d dir_b = segment_b.direction.template cast<double>().normalized();
    Vector3d cen_b = segment_b.center.template cast<double>();

    const double off_diag = -2.0 * dir_a.dot(dir_b);
    cost_matrix << 2.0, off_diag, off_diag, 2.0;
    cost_vector << 2.0 * (cen_a - cen_b).dot(dir_a),
        -2.0 * (cen_a - cen_b).dot(dir_b);

    ABSL_LOG(INFO) << std::scientific << std::setprecision(18)
                   << "segment_a.center= " << segment_a.center.transpose()
                   << "\n"
                   << "segment_a.direction= " << segment_a.direction.transpose()
                   << "\n"
                   << "segment_a.half_length= " << segment_a.half_length << "\n"
                   << "segment_b.center= " << segment_b.center.transpose()
                   << "\n"
                   << "segment_b.direction= " << segment_b.direction.transpose()
                   << "\n"
                   << "segment_b.half_length= " << segment_b.half_length
                   << "\n";
    const auto qp_sol = testing::SolveBoxQPBruteForce(cost_matrix, cost_vector,
                                                      lower_bound, upper_bound);
    const double qp_distance_squared =
        qp_sol.minimum + (segment_a.center - segment_b.center).squaredNorm();
    ABSL_LOG(INFO) << std::scientific << std::setprecision(18)
                   << "QP: solution: " << qp_sol.solution.transpose() << "\n"
                   << "QP: minimum: " << qp_sol.minimum << "\n"
                   << "QP: distance squared: " << qp_distance_squared << "\n"
                   << "distance_squared: " << distance_squared << "\n";

    EXPECT_NEAR(qp_distance_squared, distance_squared,
                kDistanceSquaredTolerance)
        << std::scientific << std::setprecision(18)
        << "segment_a.center= " << segment_a.center.transpose() << "\n"
        << "segment_a.direction= " << segment_a.direction.transpose() << "\n"
        << "segment_a.half_length= " << segment_a.half_length << "\n"
        << "segment_b.center= " << segment_b.center.transpose() << "\n"
        << "segment_b.direction= " << segment_b.direction.transpose() << "\n"
        << "segment_b.half_length= " << segment_b.half_length << "\n"
        << "Mathematica Code: "
        << internal::SegmentSegmentDistanceSquaredMathematicaCode(segment_a,
                                                                  segment_b);

    ASSERT_GE(distance_squared, 0.0);
    max_diff = std::max(
        std::abs(qp_distance_squared - static_cast<double>(distance_squared)),
        max_diff);
  }

  ABSL_LOG(INFO) << std::scientific << std::setprecision(18)
                 << "max_diff= " << max_diff;
}

REGISTER_TYPED_TEST_SUITE_P(DistanceSegmentSegmentTest, CompareAgainstQP);

typedef ::testing::Types<float, double> FPTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(DistancePrimitiveTestSuite,
                               DistanceSegmentSegmentTest, FPTypes);

template <typename T>
class DistanceSegmentSegmentDeathTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistanceSegmentSegmentDeathTest);

TYPED_TEST_P(DistanceSegmentSegmentDeathTest, InvalidInput) {
  using Scalar = TypeParam;

  const Vector3<Scalar> kPoint(0, 0, 0);
  const Vector3<Scalar> kValidDirection(1, 0, 0);
  const Vector3<Scalar> kInValidDirection(10, 0, 0);
  const Vector3<Scalar> kInValidHalfLengths(1, 0, -1);

  constexpr Scalar kValidHalfLength = Scalar{1};
  constexpr Scalar kInValidHalfLength = Scalar{-1};

  // By default, input preconditions are not checked.
  // The tests with kInvalidHalfLength are disabled, as DistanceSquared
  // currently uses std::clamp internally, whose behavior is undefined for these
  // inputs and will assert if --features=enable_libcxx_assertions is set.
  // Enable it if the implementation stops using std::clamp.
  // SegmentSegmentDistanceSquared<Scalar>(
  //     {kPoint, kValidDirection, kValidHalfLength},
  //     {kPoint, kValidDirection, kInValidHalfLength});

  SegmentSegmentDistanceSquared<Scalar>(
      {kPoint, kValidDirection, kValidHalfLength},
      {kPoint, kInValidDirection, kValidHalfLength});

  // SegmentSegmentDistanceSquared<Scalar>(
  //     {kPoint, kValidDirection, kInValidHalfLength},
  //     {kPoint, kValidDirection, kValidHalfLength});

  SegmentSegmentDistanceSquared<Scalar>(
      {kPoint, kInValidDirection, kValidHalfLength},
      {kPoint, kValidDirection, kValidHalfLength});
  // If specified, they are.
  EXPECT_DEATH(
      (SegmentSegmentDistanceSquared<
          Scalar, DebugOptions::kDebugOptionsPerformExpensiveInputChecks>(
          {kPoint, kValidDirection, kValidHalfLength},
          {kPoint, kValidDirection, kInValidHalfLength})),
      "segment_b.half_length");

  EXPECT_DEATH(
      (SegmentSegmentDistanceSquared<
          Scalar, DebugOptions::kDebugOptionsPerformExpensiveInputChecks>(
          {kPoint, kValidDirection, kValidHalfLength},
          {kPoint, kInValidDirection, kValidHalfLength})),
      "segment_b.direction");

  EXPECT_DEATH(
      (SegmentSegmentDistanceSquared<
          Scalar, DebugOptions::kDebugOptionsPerformExpensiveInputChecks>(
          {kPoint, kValidDirection, kInValidHalfLength},
          {kPoint, kValidDirection, kValidHalfLength})),
      "segment_a.half_length");

  EXPECT_DEATH(
      (SegmentSegmentDistanceSquared<
          Scalar, DebugOptions::kDebugOptionsPerformExpensiveInputChecks>(
          {kPoint, kInValidDirection, kValidHalfLength},
          {kPoint, kValidDirection, kValidHalfLength})),
      "segment_a.direction");
}

REGISTER_TYPED_TEST_SUITE_P(DistanceSegmentSegmentDeathTest, InvalidInput);

INSTANTIATE_TYPED_TEST_SUITE_P(DistanceSegmentSegmentDeathTestSuite,
                               DistanceSegmentSegmentDeathTest, FPTypes);

TEST(DistanceSegmentSegmentTest,
     SegmentSegmentDistanceSquaredMathematicaCodeTest) {
  // Only tested for doubles, to avoid the need for float-precision golden data.
  using Scalar = double;

  const Segment<Scalar> segment_a{
      .center = Vector3<Scalar>(1, 2, 3),
      .direction = Vector3<Scalar>(4, 5, 6).normalized(),
      .half_length = 1};
  const Segment<Scalar> segment_b = {
      .center = Vector3<Scalar>(-1.1, 1.2, -1.3),
      .direction = Vector3<Scalar>(1.4, -5.1, 6.1).normalized(),
      .half_length = 1.23};

  const std::string mathematica_code =
      internal::SegmentSegmentDistanceSquaredMathematicaCode(segment_a,
                                                             segment_b);

  constexpr char kGoldenString[] =
      R"(Minimize[{(2.1000000000+(0.4558423058)*u-(0.1734086901)*v)^2+)"
      R"((0.8000000000+(0.5698028823)*u-(-0.6317030853)*v)^2)"
      R"(+(4.3000000000+(0.6837634588)*u-(0.7555664354)*v)^2,)"
      R"(u >= -1.0000000000, u <= 1.0000000000, )"
      R"(v >= -1.2300000000, v<=1.2300000000},{u,v}]// FullForm)";

  EXPECT_EQ(mathematica_code, kGoldenString);

  constexpr Scalar kMathematicaCodeSolution = 10.281187506245159;
  const auto [distance2, _, __] =
      SegmentSegmentDistanceSquared(segment_a, segment_b);
  EXPECT_NEAR(distance2, kMathematicaCodeSolution, 1e-6);
}

}  // namespace
}  // namespace collision_checking
