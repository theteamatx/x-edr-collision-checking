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

#include "collision_checking/distance_box_box.h"

#include <iterator>

#include "collision_checking/debug_options.h"
#include "collision_checking/test_utils.h"
#include "eigenmath/distribution.h"
#include "eigenmath/interpolation.h"
#include "eigenmath/rotation_utils.h"
#include "eigenmath/sampling.h"
#include "collision_checking/eigenmath.h"
#include "absl/flags/flag.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

ABSL_FLAG(bool, verbose, false, "Turn on verbose text logging.");

namespace collision_checking {
namespace {
template <typename Scalar>
using Vector3 = Vector3<Scalar>;

template <typename T>
class DistanceBoxBoxTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistanceBoxBoxTest);

TYPED_TEST_P(DistanceBoxBoxTest, CompareAgainstOQSP) {
  // The squared distance between a box box_a and a box box_b is the solution to
  // the following box constrained quadratic program:
  //   (box_a.center +
  //   box_a.box_rotation_world*u-box_b.center-box_b.direction*v)^2 -> min,
  //     s.t. |u_i| <= box_a.half_length_i, |v_i| <= box_b.half_length_i
  // This test compares the output of DistanceSquared with solutions
  // computed numerically using the QP solver.

  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  constexpr Scalar kDistanceSquaredTolerance = 1e-5;
  constexpr int kNumLoops = 10000;

  eigenmath::TestGenerator gen(
      eigenmath::kGeneratorTestSeed);
  eigenmath::UniformDistributionVector<Scalar, 3> vector_dist;
  eigenmath::UniformDistributionSO3<Scalar> rotation_dist;
  constexpr double kMinHalfLength = 0.0;
  constexpr double kMaxHalfLength = 0.5;
  std::uniform_real_distribution<Scalar> length_dist(kMinHalfLength,
                                                     kMaxHalfLength);
  const Vector3 kMinPointPos{-1.0, -1.0, -1.0};
  const Vector3 kMaxPointPos{1.0, 1.0, 1.0};

  eigenmath::MatrixXd cost_matrix(6, 6);
  eigenmath::VectorXd cost_vector(6);
  eigenmath::VectorXd lower_bound(6);
  eigenmath::VectorXd upper_bound(6);
  for (int loop = 0; loop < kNumLoops; loop++) {
    SCOPED_TRACE(::testing::Message() << "At loop " << loop);
    ABSL_LOG(INFO) << "==== loop= " << loop << "\n";
    Box<Scalar> box_a, box_b;
    box_a.half_lengths = vector_dist(gen);
    box_a.center = eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos);
    box_a.box_rotation_world = rotation_dist(gen).matrix();

    box_b.half_lengths = vector_dist(gen);
    box_b.center = eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos);
    box_b.box_rotation_world = rotation_dist(gen).matrix();

    CC_MALLOC_COUNTER_INIT();
    const auto [distance_squared, box_point_a, box_point_b] =
        DistanceSquared(box_a, box_b);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();
    lower_bound.head(3) = -box_a.half_lengths.template cast<double>();
    lower_bound.tail(3) = -box_b.half_lengths.template cast<double>();
    upper_bound.head(3) = box_a.half_lengths.template cast<double>();
    upper_bound.tail(3) = box_b.half_lengths.template cast<double>();

    Matrix3d RaRbT =
        box_a.box_rotation_world.template cast<double>() *
        box_b.box_rotation_world.template cast<double>().transpose();
    Matrix3d RbRaT =
        box_b.box_rotation_world.template cast<double>() *
        box_a.box_rotation_world.template cast<double>().transpose();
    Vector3d delta_position = box_a.center.template cast<double>() -
                              box_b.center.template cast<double>();
    cost_matrix.setZero();
    cost_matrix.block(0, 0, 3, 3) =
        eigenmath::MatrixXd::Identity(3, 3) * 2.0;
    cost_matrix.block(0, 3, 3, 3) = -2.0 * RaRbT;
    cost_matrix.block(3, 0, 3, 3) = -2.0 * RbRaT;
    cost_matrix.block(3, 3, 3, 3) =
        eigenmath::MatrixXd::Identity(3, 3) * 2.0;

    cost_vector.head(3) =
        2 * box_a.box_rotation_world.template cast<double>() *
         delta_position;
    cost_vector.tail(3) =
        -2 * box_b.box_rotation_world.template cast<double>() * delta_position;

    const auto qp_sol = testing::SolveBoxQPBruteForce(cost_matrix, cost_vector,
                                                lower_bound, upper_bound);

    const Scalar qp_distance_squared =
        static_cast<Scalar>(qp_sol.minimum) + delta_position.squaredNorm();

    // Mathematica code for computing the minimum.
    if constexpr (false) {
    ABSL_LOG(INFO) << absl::StrFormat(
        "Minimize[{(%.10f+(%10f)*ux+(%10f)*uy+(%10f)*uz"
        "-(%10f)*vx-(%10f)*vy-(%10f)*vz)^2"
        "+(%.10f+(%10f)*ux+(%10f)*uy+(%10f)*uz"
        "-(%10f)*vx-(%10f)*vy-(%10f)*vz)^2"
        "+(%.10f+(%10f)*ux+(%10f)*uy+(%10f)*uz"
        "-(%10f)*vx-(%10f)*vy-(%10f)*vz)^2,"
        "ux >= %.10f, ux <= %.10f, "
        "uy >= %.10f, uy <= %.10f, "
        "uz >= %.10f, uz <= %.10f, "
        "vx >= %.10f, vx <= %.10f, "
        "vy >= %.10f, vy <= %.10f, "
        "vz >= %.10f, vz <= %.10f},"
        "{ux,uy,uz, vx,vy,vz}]",
        box_a.center[0] - box_b.center[0], box_a.box_rotation_world(0, 0),
        box_a.box_rotation_world(1, 0), box_a.box_rotation_world(2, 0),
        box_b.box_rotation_world(0, 0), box_b.box_rotation_world(1, 0),
        box_b.box_rotation_world(2, 0),
        //
        box_a.center[1] - box_b.center[1], box_a.box_rotation_world(0, 1),
        box_a.box_rotation_world(1, 1), box_a.box_rotation_world(2, 1),
        box_b.box_rotation_world(0, 1), box_b.box_rotation_world(1, 1),
        box_b.box_rotation_world(2, 1),
        //
        box_a.center[2] - box_b.center[2], box_a.box_rotation_world(0, 2),
        box_a.box_rotation_world(1, 2), box_a.box_rotation_world(2, 2),
        box_b.box_rotation_world(0, 2), box_b.box_rotation_world(1, 2),
        box_b.box_rotation_world(2, 2),
        //
        -box_a.half_lengths[0], box_a.half_lengths[0], -box_a.half_lengths[1],
        box_a.half_lengths[1], -box_a.half_lengths[2], box_a.half_lengths[2],
        -box_b.half_lengths[0], box_b.half_lengths[0], -box_b.half_lengths[1],
        box_b.half_lengths[1], -box_b.half_lengths[2], box_b.half_lengths[2]);
    // Mathematica code for the distance squared.
    ABSL_LOG(INFO) << absl::StrFormat(
        "g[ux_,uy_, uz_, vx_, vy_, "
        "vz_]:=(%.10f+(%10f)*ux+(%10f)*uy+(%10f)*uz"
        "-(%10f)*vx-(%10f)*vy-(%10f)*vz)^2"
        "+(%.10f+(%10f)*ux+(%10f)*uy+(%10f)*uz"
        "-(%10f)*vx-(%10f)*vy-(%10f)*vz)^2"
        "+(%.10f+(%10f)*ux+(%10f)*uy+(%10f)*uz"
        "-(%10f)*vx-(%10f)*vy-(%10f)*vz)^2",
        box_a.center[0] - box_b.center[0], box_a.box_rotation_world(0, 0),
        box_a.box_rotation_world(1, 0), box_a.box_rotation_world(2, 0),
        box_b.box_rotation_world(0, 0), box_b.box_rotation_world(1, 0),
        box_b.box_rotation_world(2, 0),
        //
        box_a.center[1] - box_b.center[1], box_a.box_rotation_world(0, 1),
        box_a.box_rotation_world(1, 1), box_a.box_rotation_world(2, 1),
        box_b.box_rotation_world(0, 1), box_b.box_rotation_world(1, 1),
        box_b.box_rotation_world(2, 1),
        //
        box_a.center[2] - box_b.center[2], box_a.box_rotation_world(0, 2),
        box_a.box_rotation_world(1, 2), box_a.box_rotation_world(2, 2),
        box_b.box_rotation_world(0, 2), box_b.box_rotation_world(1, 2),
        box_b.box_rotation_world(2, 2));
    }
    EXPECT_NEAR(qp_distance_squared, distance_squared,
                kDistanceSquaredTolerance)
        << "=== at loop: " << loop
        << " diff= " << std::abs(qp_distance_squared - distance_squared);
  }
}

TYPED_TEST_P(DistanceBoxBoxTest, FloatingPointRobustness) {
  using Scalar = TypeParam;

  Box<Scalar> box_a;
  Box<Scalar> box_b;
  box_a.center << Scalar(0), Scalar(0), Scalar(0);
  box_a.half_lengths << Scalar(3.75e-01), Scalar(3.75e-01), Scalar(1.25e-01);
  box_a.box_rotation_world.setIdentity();
  box_b.center << Scalar(-1.340640076568736516e-01),
      Scalar(-1.404928717704230223e-01), Scalar(0);
  box_b.half_lengths << Scalar(5.000000000000000278e-02),
      Scalar(5.000000000000000278e-02), Scalar(2.5e-01);
  box_b.box_rotation_world << Scalar(-1.411200080598671303e-01),
      Scalar(-9.899924966004454152e-01), Scalar(0),
      Scalar(9.899924966004454152e-01), Scalar(-1.411200080598671303e-01),
      Scalar(0), Scalar(0), Scalar(0), Scalar(1);

  const ObjectObjectResult<Scalar> result = DistanceSquared(box_a, box_b);
  EXPECT_LE(result.distance_squared, Scalar(0));
}

REGISTER_TYPED_TEST_SUITE_P(DistanceBoxBoxTest, CompareAgainstOQSP,
                            FloatingPointRobustness);

typedef ::testing::Types<float, double> FPTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(DistanceBoxBoxTestSuite, DistanceBoxBoxTest,
                               FPTypes);
}  // namespace
}  // namespace collision_checking
