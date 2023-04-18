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

#include "collision_checking/distance_segment_box.h"

#include <iterator>
#include <limits>

#include "absl/flags/flag.h"
#include "absl/strings/str_format.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/test_utils.h"
#include "eigenmath/distribution.h"
#include "eigenmath/interpolation.h"
#include "eigenmath/rotation_utils.h"
#include "eigenmath/sampling.h"
#include "gtest/gtest.h"

ABSL_FLAG(bool, verbose, false, "Turn on verbose text logging.");

namespace collision_checking {
namespace {

template <typename T>
class DistanceSegmentBoxTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistanceSegmentBoxTest);

TYPED_TEST_P(DistanceSegmentBoxTest, CompareAgainstQP) {
  // The squared distance between a segment and a box is the solution to the
  // following box constrained quadratic program:
  //   (box.center +
  //   box.box_rotation_world*u-segment.center-segment.direction*t)^2 -> min,
  //     s.t. |u_i| <= box.half_length_i, |t|<= segment.half_length
  // This test compares the output of DistanceSquared with solutions
  // computed numerically using the QP solver.

  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  constexpr Scalar kDistanceSquaredTolerance =
      1000 * std::numeric_limits<Scalar>::epsilon();
  constexpr int kNumLoops = 5000;
  eigenmath::TestGenerator gen(eigenmath::kGeneratorTestSeed);
  eigenmath::UniformDistributionVector<Scalar, 3> vector_dist;
  eigenmath::UniformDistributionSO3<Scalar> rotation_dist;
  constexpr double kMinHalfLength = 0.0;
  constexpr double kMaxHalfLength = 0.5;
  std::uniform_real_distribution<Scalar> length_dist(kMinHalfLength,
                                                     kMaxHalfLength);
  const Vector3 kMinPointPos{-1.0, -1.0, -1.0};
  const Vector3 kMaxPointPos{1.0, 1.0, 1.0};

  eigenmath::MatrixXd cost_matrix(4, 4);
  eigenmath::VectorXd cost_vector(4);
  eigenmath::VectorXd lower_bound(4);
  eigenmath::VectorXd upper_bound(4);

  int edge_parallel_case_no = 0;
  int face_parallel_case_no = 0;
  Scalar max_error{0};

  for (int loop = 0; loop < kNumLoops; loop++) {
    SCOPED_TRACE(::testing::Message() << "At loop " << loop);
    ABSL_LOG(INFO) << "==== loop= " << loop << "\n";
    Segment<Scalar> segment;
    Box<Scalar> box;
    box.half_lengths = vector_dist(gen);
    box.center = eigenmath::InterpolateLinearInBox(vector_dist(gen),
                                                   kMinPointPos, kMaxPointPos);

    segment.half_length = length_dist(gen);
    segment.center = eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos);
    segment.direction = eigenmath::InterpolateLinearInBox(
                            vector_dist(gen), kMinPointPos, kMaxPointPos)
                            .normalized();

    box.box_rotation_world = rotation_dist(gen).matrix();

    // To ensure parallel cases are covered, enforce edge- or face-parallel
    // segment direction on some iterations.
    if (loop % 10 == 0) {
      // Edge-parallel direction.
      segment.direction = box.box_rotation_world.row(edge_parallel_case_no % 3);
      if (edge_parallel_case_no % 2) {
        segment.direction = -segment.direction;
      }
      edge_parallel_case_no++;
    } else if (loop % 5 == 0) {
      // Face-parallel direction.
      const int ui = face_parallel_case_no % 3;
      constexpr int kVi[3] = {1, 2, 0};
      constexpr int kWi[3] = {2, 0, 1};
      segment.direction =
          (box.box_rotation_world.row(kVi[ui]) * length_dist(gen) +
           box.box_rotation_world.row(kWi[ui]) * length_dist(gen))
              .normalized();
      if (face_parallel_case_no % 2) {
        segment.direction = -segment.direction;
      }
      face_parallel_case_no++;
    }

    CC_MALLOC_COUNTER_INIT();
    const auto [distance_squared, segment_parameter, box_parameters] =
        DistanceSquared(segment, box);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

    // Self-consistency.
    const Vector3 box_point =
        box.center + box.box_rotation_world.transpose() * box_parameters;
    const Vector3 segment_point =
        segment.center + segment.direction * segment_parameter;
    EXPECT_NEAR(distance_squared, (box_point - segment_point).squaredNorm(),
                kDistanceSquaredTolerance);

    lower_bound.head(3) = -box.half_lengths.template cast<double>();
    lower_bound(3) = -segment.half_length;
    upper_bound.head(3) = box.half_lengths.template cast<double>();
    upper_bound(3) = segment.half_length;

    const Vector3d off_diagonal =
        -2.0 * box.box_rotation_world.template cast<double>() *
        segment.direction.template cast<double>();
    cost_matrix = eigenmath::MatrixXd::Identity(4, 4) * 2.0;
    cost_matrix.block(0, 3, 3, 1) = off_diagonal;
    cost_matrix.block(3, 0, 1, 3) = off_diagonal.transpose();
    Vector3d delta = box.center.template cast<double>() -
                     segment.center.template cast<double>();
    cost_vector.head(3) = (2 * box.box_rotation_world.template cast<double>() *
                           delta.template cast<double>());
    cost_vector(3) =
        -2.0 * delta.dot(segment.direction.template cast<double>());

    const auto qp_sol = testing::SolveBoxQPBruteForce(cost_matrix, cost_vector,
                                                      lower_bound, upper_bound);
    const Scalar qp_distance_squared =
        static_cast<Scalar>(qp_sol.minimum) + delta.squaredNorm();

    EXPECT_NEAR(qp_distance_squared, distance_squared,
                kDistanceSquaredTolerance);
    max_error =
        std::max(std::abs(qp_distance_squared - distance_squared), max_error);

    ABSL_LOG(INFO) << absl::StreamFormat(
        "===LOOP %d qp: %.17e this %.17e diff %.17e max_diff: %.17e (thrs: "
        "%.17e, passed: %d); QP solution: %.17f %.17f %.17f\n"
        "solution: %.17f %.17f %.17f\n",
        loop, qp_distance_squared, distance_squared,
        std::abs(qp_distance_squared - distance_squared), max_error,
        kDistanceSquaredTolerance, max_error > kDistanceSquaredTolerance,
        qp_sol.solution[0], qp_sol.solution[1], qp_sol.solution[2],
        box_parameters[0], box_parameters[1], box_parameters[2]);
  }
}
REGISTER_TYPED_TEST_SUITE_P(DistanceSegmentBoxTest, CompareAgainstQP);

typedef ::testing::Types<float, double> FPTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(DistanceSegmentBoxTestSuite,
                               DistanceSegmentBoxTest, FPTypes);
}  // namespace
}  // namespace collision_checking
