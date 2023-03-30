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

#include "collision_checking/distance_point_box.h"

#include <iterator>
#include <limits>

#include "absl/flags/flag.h"
#include "collision_checking/debug_options.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/test_utils.h"
#include "eigenmath/distribution.h"
#include "eigenmath/interpolation.h"
#include "eigenmath/rotation_utils.h"
#include "eigenmath/sampling.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

ABSL_FLAG(bool, verbose, false, "Turn on verbose text logging.");

namespace collision_checking {
namespace {
template <typename Scalar>
using Vector3 = Vector3<Scalar>;

template <typename T>
class DistancePointBoxTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistancePointBoxTest);

TYPED_TEST_P(DistancePointBoxTest, CompareAgainstQP) {
  // The squared distance between a point and a box is the solution to the
  // following box constrained quadratic program:
  //   (box_center + box_rotation*u-point)^2 -> min,
  //     s.t. |u_i| <= half_length_i.
  // This test compares the output of DistanceSquared with solutions
  // computed numerically using the qp solver.

  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  constexpr Scalar kQPTolerance = 100 * std::numeric_limits<Scalar>::epsilon();
  constexpr Scalar kDistanceSquaredTolerance = 10 * kQPTolerance;
  constexpr int kNumLoops = 100;

  eigenmath::TestGenerator gen(eigenmath::kGeneratorTestSeed);
  eigenmath::UniformDistributionVector<Scalar, 3> vector_dist;
  eigenmath::UniformDistributionSO3<Scalar> rotation_dist;
  const Vector3 kMinPointPos{-1.0, -1.0, -1.0};
  const Vector3 kMaxPointPos{1.0, 1.0, 1.0};

  eigenmath::MatrixXd cost_matrix(3, 3);
  eigenmath::VectorXd cost_vector(3);
  eigenmath::VectorXd lower_bound(3);
  eigenmath::VectorXd upper_bound(3);

  for (int loop = 0; loop < kNumLoops; loop++) {
    SCOPED_TRACE(::testing::Message() << "At loop " << loop);
    ABSL_LOG(INFO) << "==== loop= " << loop;
    Point<Scalar> point = {eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos)};
    Box<Scalar> box;
    box.half_lengths = vector_dist(gen);
    box.center = eigenmath::InterpolateLinearInBox(vector_dist(gen),
                                                   kMinPointPos, kMaxPointPos);

    box.box_rotation_world = rotation_dist(gen).matrix();

    CC_MALLOC_COUNTER_INIT();
    const auto [distance_squared, closest_point] = DistanceSquared(point, box);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

    lower_bound = -box.half_lengths.template cast<double>();
    upper_bound = box.half_lengths.template cast<double>();

    cost_matrix = eigenmath::MatrixXd::Identity(3, 3) * 2.0;
    cost_vector = (2.0 * box.box_rotation_world * (box.center - point.center))
                      .template cast<double>();

    const auto qp_sol = testing::SolveBoxQPBruteForce(cost_matrix, cost_vector,
                                                      lower_bound, upper_bound);
    const Scalar qp_distance_squared =
        qp_sol.minimum + (box.center - point.center).squaredNorm();
    ABSL_LOG(INFO) << "point= " << point.center.transpose() << "\n"
                   << "box.center= " << box.center.transpose() << "\n"
                   << "box.half_lengths= " << box.half_lengths.transpose()
                   << "\n"
                   << "box.box_rotation_world= " << box.box_rotation_world
                   << "\n"
                   << "closest_point= " << closest_point.transpose() << "\n";
    ABSL_LOG(INFO) << "OSQP: solution: " << qp_sol.solution.transpose() << "\n"
                   << "OSQP: minimum: " << qp_sol.minimum << "\n"
                   << "OSQP: distance squared: " << qp_distance_squared << "\n"
                   << "distance_squared: " << distance_squared;
    EXPECT_NEAR(qp_distance_squared, distance_squared,
                kDistanceSquaredTolerance);
  }
}

REGISTER_TYPED_TEST_SUITE_P(DistancePointBoxTest, CompareAgainstQP);

typedef ::testing::Types<float, double> FPTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(DistancePrimitiveTestSuite, DistancePointBoxTest,
                               FPTypes);
}  // namespace
}  // namespace collision_checking
