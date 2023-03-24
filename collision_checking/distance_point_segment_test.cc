#include "collision_checking/distance_point_segment.h"

#include <iterator>
#include <limits>

#include "collision_checking/debug_options.h"
#include "collision_checking/test_utils.h"
#include "googlex/proxy/eigenmath/distribution.h"
#include "googlex/proxy/eigenmath/interpolation.h"
#include "googlex/proxy/eigenmath/sampling.h"
#include "collision_checking/eigenmath.h"
#include "third_party/absl/flags/flag.h"
#include "third_party/benchmark/include/benchmark/benchmark.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "third_party/osqp_cpp/include/osqp++.h"

ABSL_FLAG(bool, verbose, false, "Turn on verbose text logging.");

namespace collision_checking {
namespace {
template <typename Scalar>
using Vector3 = Vector3<Scalar>;

template <typename T>
class DistancePointSegmentTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistancePointSegmentTest);

TYPED_TEST_P(DistancePointSegmentTest, CompareAgainstQSQP) {
  // The squared distance between a point and a segment is the solution to the
  // following box constrained quadratic program:
  //   (segment_center + segment_direction*u-point)^2 -> min,
  //     s.t. |u| <= half_length.
  // This test compares the output of DistanceSquared with solutions
  // computed numerically using the osqp solver.

  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  constexpr Scalar kOSQPTolerance =
      100 * std::numeric_limits<Scalar>::epsilon();
  constexpr Scalar kDistanceSquaredTolerance = 10 * kOSQPTolerance;
  constexpr int kNumLoops = 100;

  ::blue::eigenmath::ProxyTestGenerator gen(
      ::blue::eigenmath::kGeneratorTestSeed);
  ::blue::eigenmath::UniformDistributionVector<Scalar, 3> vector_dist;
  constexpr Scalar kMinHalfLength = 0.0;
  constexpr Scalar kMaxHalfLength = 0.5;
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
  instance.constraint_matrix = Eigen::SparseMatrix<double>(1, 1);
  instance.lower_bounds.resize(1);
  instance.upper_bounds.resize(1);
  Eigen::SparseMatrix<double> box_constraint_matrix(1, 1);
  const Eigen::Triplet<double> kTripletsA[] = {{0, 0, 1.0}};
  instance.constraint_matrix.setFromTriplets(std::begin(kTripletsA),
                                             std::end(kTripletsA));

  for (int loop = 0; loop < kNumLoops; loop++) {
    SCOPED_TRACE(::testing::Message() << "At loop " << loop);
    VLOG(2) << "==== loop= " << loop;

    Segment<Scalar> segment = {
        .center = ::blue::eigenmath::InterpolateLinearInBox(
            vector_dist(gen), kMinPointPos, kMaxPointPos),
        .direction = ::blue::eigenmath::InterpolateLinearInBox(
                         vector_dist(gen), kMinPointPos, kMaxPointPos)
                         .normalized(),
        .half_length = length_dist(gen)};
    Point<Scalar> point = {::blue::eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos)};

    CC_MALLOC_COUNTER_INIT();
    const auto [distance_squared, u] =
        DistanceSquared(point, segment);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

    instance.lower_bounds << -segment.half_length;
    instance.upper_bounds << segment.half_length;

    Eigen::SparseMatrix<double> objective_matrix(1, 1);
    const Eigen::Triplet<double> kTripletsP[] = {{0, 0, 2.0}};

    objective_matrix.setFromTriplets(std::begin(kTripletsP),
                                     std::end(kTripletsP));

    instance.objective_matrix = objective_matrix;
    instance.objective_vector.resize(1);
    instance.objective_vector
        << 2.0 * (segment.center - point.center).dot(segment.direction);

    CC_ASSERT_OK(solver.Init(instance, settings));
    const osqp::OsqpExitCode exit_code = solver.Solve();

    EXPECT_EQ(exit_code, osqp::OsqpExitCode::kOptimal);

    const Scalar osqp_distance_squared =
        static_cast<Scalar>(solver.objective_value()) +
        (segment.center - point.center).squaredNorm();

    VLOG(2) << "OSQP: solution: " << solver.primal_solution().transpose()
            << "\n"
            << "OSQP: minimum: " << solver.objective_value() << "\n"
            << "OSQP: distance squared: " << osqp_distance_squared << "\n"
            << "distance_squared: " << distance_squared;
    ASSERT_NEAR(osqp_distance_squared, distance_squared,
                kDistanceSquaredTolerance);
  }
}

REGISTER_TYPED_TEST_SUITE_P(DistancePointSegmentTest, CompareAgainstQSQP);

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

