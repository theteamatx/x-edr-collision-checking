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

#include "collision_checking/test_utils.h"
#include "eigenmath/distribution.h"
#include "eigenmath/interpolation.h"
#include "eigenmath/rotation_utils.h"
#include "eigenmath/sampling.h"
#include "collision_checking/eigenmath.h"
#include "absl/flags/flag.h"
#include "absl/strings/str_format.h"
#include "gtest/gtest.h"
#include "third_party/osqp_cpp/include/osqp++.h"

ABSL_FLAG(bool, verbose, false, "Turn on verbose text logging.");

namespace collision_checking {
namespace {
template <typename Scalar>
using Vector3 = Vector3<Scalar>;

template <typename T>
class DistanceSegmentBoxTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistanceSegmentBoxTest);

TYPED_TEST_P(DistanceSegmentBoxTest, CompareAgainstOSQP) {
  // The squared distance between a segment and a box is the solution to the
  // following box constrained quadratic program:
  //   (box.center +
  //   box.box_rotation_world*u-segment.center-segment.direction*t)^2 -> min,
  //     s.t. |u_i| <= box.half_length_i, |t|<= segment.half_length
  // This test compares the output of DistanceSquared with solutions
  // computed numerically using the osqp solver.

  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  constexpr Scalar kOSQPTolerance =
      100 * std::numeric_limits<Scalar>::epsilon();
  constexpr Scalar kDistanceSquaredTolerance = 10 * kOSQPTolerance;
  constexpr int kNumLoops = 5000;
  ::blue::eigenmath::TestGenerator gen(
      ::blue::eigenmath::kGeneratorTestSeed);
  ::blue::eigenmath::UniformDistributionVector<Scalar, 3> vector_dist;
  ::blue::eigenmath::UniformDistributionSO3<Scalar> rotation_dist;
  constexpr double kMinHalfLength = 0.0;
  constexpr double kMaxHalfLength = 0.5;
  std::uniform_real_distribution<Scalar> length_dist(kMinHalfLength,
                                                     kMaxHalfLength);
  const Vector3 kMinPointPos{-1.0, -1.0, -1.0};
  const Vector3 kMaxPointPos{1.0, 1.0, 1.0};

  osqp::OsqpSolver solver;
  osqp::OsqpInstance instance;
  osqp::OsqpSettings settings;
  settings.eps_abs = kOSQPTolerance;
  settings.eps_rel = 0.0;
  settings.warm_start = false;
  settings.max_iter = 1000000;
  settings.verbose = absl::GetFlag(FLAGS_verbose);
  // Turn of adaptive_rho, as it depends on run-time and makes the test
  // non-deterministic!
  settings.adaptive_rho = false;
  // This determines "num_variables" (cols) and "num_constraints" (rows).
  instance.constraint_matrix = Eigen::SparseMatrix<double>(4, 4);
  instance.lower_bounds.resize(4);
  instance.upper_bounds.resize(4);
  Eigen::SparseMatrix<double> box_constraint_matrix(4, 4);
  const Eigen::Triplet<double> kTripletsA[] = {
      {0, 0, 1.0}, {0, 1, 0.0}, {0, 2, 0.0}, {0, 3, 0.0},
      {1, 0, 0.0}, {1, 1, 1.0}, {1, 2, 0.0}, {1, 3, 0.0},
      {2, 0, 0.0}, {2, 1, 0.0}, {2, 2, 1.0}, {2, 3, 0.0},
      {3, 0, 0.0}, {3, 1, 0.0}, {3, 2, 0.0}, {3, 3, 1.0}};
  instance.constraint_matrix.setFromTriplets(std::begin(kTripletsA),
                                             std::end(kTripletsA));

  int edge_parallel_case_no = 0;
  int face_parallel_case_no = 0;
  Scalar max_error{0};

  for (int loop = 0; loop < kNumLoops; loop++) {
    SCOPED_TRACE(::testing::Message() << "At loop " << loop);
    VLOG(2) << "==== loop= " << loop << "\n";
    Segment<Scalar> segment;
    Box<Scalar> box;
    box.half_lengths = vector_dist(gen);
    box.center = ::blue::eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos);

    segment.half_length = length_dist(gen);
    segment.center = ::blue::eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos);
    segment.direction = ::blue::eigenmath::InterpolateLinearInBox(
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

    instance.lower_bounds.resize(4);
    instance.upper_bounds.resize(4);
    instance.lower_bounds.head(3) = -box.half_lengths.template cast<double>();
    instance.lower_bounds(3) = -segment.half_length;
    instance.upper_bounds.head(3) = box.half_lengths.template cast<double>();
    instance.upper_bounds(3) = segment.half_length;

    Vector3 off_diagonal = -2 * box.box_rotation_world * segment.direction;
    Eigen::SparseMatrix<double> objective_matrix(4, 4);
    const Eigen::Triplet<double> kTripletsP[] = {{0, 0, 2.0},
                                                 {0, 1, 0.0},
                                                 {0, 2, 0.0},
                                                 {0, 3, off_diagonal[0]},
                                                 {1, 0, 0.0},
                                                 {1, 1, 2.0},
                                                 {1, 2, 0.0},
                                                 {1, 3, off_diagonal[1]},
                                                 {2, 0, 0.0},
                                                 {2, 1, 0.0},
                                                 {2, 2, 2.0},
                                                 {2, 3, off_diagonal[2]},
                                                 {3, 0, off_diagonal[0]},
                                                 {3, 1, off_diagonal[1]},
                                                 {3, 2, off_diagonal[2]},
                                                 {3, 3, 2}};

    objective_matrix.setFromTriplets(std::begin(kTripletsP),
                                     std::end(kTripletsP));

    instance.objective_matrix = objective_matrix;
    Vector3 delta = box.center - segment.center;
    instance.objective_vector.resize(4);
    instance.objective_vector.head(3) =
        (2 * box.box_rotation_world * delta).template cast<double>();
    instance.objective_vector(3) = -2.0 * delta.dot(segment.direction);

    CC_ASSERT_OK(solver.Init(instance, settings));
    const osqp::OsqpExitCode exit_code = solver.Solve();

    ASSERT_TRUE(exit_code == osqp::OsqpExitCode::kOptimal ||
                exit_code == osqp::OsqpExitCode::kOptimalInaccurate)
        << " exit_code= " << osqp::ToString(exit_code);

    const Scalar osqp_distance_squared =
        static_cast<Scalar>(solver.objective_value()) + delta.squaredNorm();

    EXPECT_NEAR(osqp_distance_squared, distance_squared,
                kDistanceSquaredTolerance);
    max_error =
        std::max(std::abs(osqp_distance_squared - distance_squared), max_error);

    VLOG(2) << absl::StreamFormat(
        "===LOOP %d osqp: %.17e this %.17e diff %.17e max_diff: %.17e (thrs: "
        "%.17e, passed: %d); osqp solution: %.17f %.17f %.17f\n"
        "solution: %.17f %.17f %.17f",
        loop, osqp_distance_squared, distance_squared,
        std::abs(osqp_distance_squared - distance_squared), max_error,
        kDistanceSquaredTolerance, max_error > kDistanceSquaredTolerance,
        solver.primal_solution()[0], solver.primal_solution()[1],
        solver.primal_solution()[2], box_parameters[0], box_parameters[1],
        box_parameters[2]);
  }
}
REGISTER_TYPED_TEST_SUITE_P(DistanceSegmentBoxTest, CompareAgainstOSQP);

typedef ::testing::Types<float, double> FPTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(DistanceSegmentBoxTestSuite,
                               DistanceSegmentBoxTest, FPTypes);
}  // namespace
}  // namespace collision_checking

