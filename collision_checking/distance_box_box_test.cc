#include "experimental/users/buschmann/collision_checking/distance_box_box.h"

#include <iterator>

#include "experimental/users/buschmann/collision_checking/debug_options.h"
#include "experimental/users/buschmann/collision_checking/test_utils.h"
#include "googlex/proxy/eigenmath/distribution.h"
#include "googlex/proxy/eigenmath/interpolation.h"
#include "googlex/proxy/eigenmath/rotation_utils.h"
#include "googlex/proxy/eigenmath/sampling.h"
#include "experimental/users/buschmann/collision_checking/eigenmath.h"
#include "third_party/absl/flags/flag.h"
#include "third_party/googletest/googlemock/include/gmock/gmock.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"
#include "third_party/osqp_cpp/include/osqp++.h"

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
  // computed numerically using the osqp solver.

  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  using Matrix3 = Matrix3<Scalar>;
  constexpr Scalar kOSQPTolerance = 1e-7;
  constexpr Scalar kDistanceSquaredTolerance = 1e-5;
  constexpr int kNumLoops = 10000;

  ::blue::eigenmath::ProxyTestGenerator gen(
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
  settings.max_iter = 10000;
  settings.verbose = absl::GetFlag(FLAGS_verbose);
  // Turn of adaptive_rho, as it depends on run-time and makes the test
  // non-deterministic!
  settings.adaptive_rho = false;
  // This determines "num_variables" (cols) and "num_constraints" (rows).
  instance.constraint_matrix = Eigen::SparseMatrix<double>(6, 6);
  instance.lower_bounds.resize(6);
  instance.upper_bounds.resize(6);
  Eigen::SparseMatrix<double> box_constraint_matrix(6, 6);
  const Eigen::Triplet<double> kTripletsA[] = {
      {0, 0, 1.0}, {0, 1, 0.0}, {0, 2, 0.0}, {0, 3, 0.0}, {0, 4, 0.0},
      {0, 5, 0.0}, {1, 0, 0.0}, {1, 1, 1.0}, {1, 2, 0.0}, {1, 3, 0.0},
      {1, 4, 0.0}, {1, 5, 0.0}, {2, 0, 0.0}, {2, 1, 0.0}, {2, 2, 1.0},
      {2, 3, 0.0}, {2, 4, 0.0}, {2, 5, 0.0}, {3, 0, 0.0}, {3, 1, 0.0},
      {3, 2, 0.0}, {3, 3, 1.0}, {3, 4, 0.0}, {3, 5, 0.0}, {4, 0, 0.0},
      {4, 1, 0.0}, {4, 2, 0.0}, {4, 3, 0.0}, {4, 4, 1.0}, {4, 5, 0.0},
      {5, 0, 0.0}, {5, 1, 0.0}, {5, 2, 0.0}, {5, 3, 0.0}, {5, 4, 0.0},
      {5, 5, 1.0},
  };
  instance.constraint_matrix.setFromTriplets(std::begin(kTripletsA),
                                             std::end(kTripletsA));

  for (int loop = 0; loop < kNumLoops; loop++) {
    SCOPED_TRACE(::testing::Message() << "At loop " << loop);
    VLOG(2) << "==== loop= " << loop << "\n";
    Box<Scalar> box_a, box_b;
    box_a.half_lengths = vector_dist(gen);
    box_a.center = ::blue::eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos);
    box_a.box_rotation_world = rotation_dist(gen).matrix();

    box_b.half_lengths = vector_dist(gen);
    box_b.center = ::blue::eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos);
    box_b.box_rotation_world = rotation_dist(gen).matrix();

    CC_MALLOC_COUNTER_INIT();
    const auto [distance_squared, box_point_a, box_point_b] =
        DistanceSquared(box_a, box_b);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

    instance.lower_bounds.resize(6);
    instance.upper_bounds.resize(6);
    instance.lower_bounds.head(3) = -box_a.half_lengths.template cast<double>();
    instance.lower_bounds.tail(3) = -box_b.half_lengths.template cast<double>();
    instance.upper_bounds.head(3) = box_a.half_lengths.template cast<double>();
    instance.upper_bounds.tail(3) = box_b.half_lengths.template cast<double>();

    Matrix3 RaRbT =
        box_a.box_rotation_world * box_b.box_rotation_world.transpose();
    Matrix3 RbRaT =
        box_b.box_rotation_world * box_a.box_rotation_world.transpose();
    Vector3 delta_position = box_a.center - box_b.center;
    Eigen::SparseMatrix<double> objective_matrix(6, 6);
    const Eigen::Triplet<double> kTripletsP[] = {
        // top left block
        {0, 0, 2.0},
        {0, 1, 0.0},
        {0, 2, 0.0},
        {1, 0, 0.0},
        {1, 1, 2.0},
        {1, 2, 0.0},
        {2, 0, 0.0},
        {2, 1, 0.0},
        {2, 2, 2.0},
        // top right block
        {0, 3, -2 * RaRbT(0, 0)},
        {0, 4, -2 * RaRbT(0, 1)},
        {0, 5, -2 * RaRbT(0, 2)},
        {1, 3, -2 * RaRbT(1, 0)},
        {1, 4, -2 * RaRbT(1, 1)},
        {1, 5, -2 * RaRbT(1, 2)},
        {2, 3, -2 * RaRbT(2, 0)},
        {2, 4, -2 * RaRbT(2, 1)},
        {2, 5, -2 * RaRbT(2, 2)},
        // bottom left block
        {3, 0, -2 * RbRaT(0, 0)},
        {3, 1, -2 * RbRaT(0, 1)},
        {3, 2, -2 * RbRaT(0, 2)},
        {4, 0, -2 * RbRaT(1, 0)},
        {4, 1, -2 * RbRaT(1, 1)},
        {4, 2, -2 * RbRaT(1, 2)},
        {5, 0, -2 * RbRaT(2, 0)},
        {5, 1, -2 * RbRaT(2, 1)},
        {5, 2, -2 * RbRaT(2, 2)},
        // bottom right block
        {3, 3, 2.0},
        {3, 4, 0.0},
        {3, 5, 0.0},
        {4, 3, 0.0},
        {4, 4, 2.0},
        {4, 5, 0.0},
        {5, 3, 0.0},
        {5, 4, 0.0},
        {5, 5, 2.0},
    };

    objective_matrix.setFromTriplets(std::begin(kTripletsP),
                                     std::end(kTripletsP));

    instance.objective_matrix = objective_matrix;
    instance.objective_vector.resize(6);
    instance.objective_vector.head(3) =
        (2 * box_a.box_rotation_world * delta_position).template cast<double>();
    instance.objective_vector.tail(3) =
        (-2 * box_b.box_rotation_world * delta_position)
            .template cast<double>();

    CC_ASSERT_OK(solver.Init(instance, settings));
    const osqp::OsqpExitCode exit_code = solver.Solve();

    // For some test cases osqp doesn't return kOptimal. Which parameters need
    // to be tweaked?
    ASSERT_TRUE(exit_code == osqp::OsqpExitCode::kOptimal ||
                exit_code == osqp::OsqpExitCode::kOptimalInaccurate)
        << " exit_code= " << static_cast<int>(exit_code);
    const Scalar osqp_distance_squared =
        static_cast<Scalar>(solver.objective_value()) +
        delta_position.squaredNorm();

    // Mathematica code for computing the minimum.
    VLOG(3) << absl::StrFormat(
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
    VLOG(3) << absl::StrFormat(
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

    EXPECT_NEAR(osqp_distance_squared, distance_squared,
                kDistanceSquaredTolerance)
        << "=== at loop: " << loop
        << " diff= " << std::abs(osqp_distance_squared - distance_squared);
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
