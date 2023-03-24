#include "experimental/users/buschmann/collision_checking/distance_segment_segment.h"

#include <algorithm>
#include <iomanip>
#include <ios>
#include <iterator>
#include <string>

#include "experimental/users/buschmann/collision_checking/debug_options.h"
#include "experimental/users/buschmann/collision_checking/test_utils.h"
#include "googlex/proxy/eigenmath/distribution.h"
#include "googlex/proxy/eigenmath/interpolation.h"
#include "googlex/proxy/eigenmath/sampling.h"
#include "experimental/users/buschmann/collision_checking/eigenmath.h"
#include "third_party/absl/flags/flag.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"
#include "third_party/osqp_cpp/include/osqp++.h"

ABSL_FLAG(bool, verbose, false, "Turn on verbose text logging.");

namespace collision_checking {
namespace {
template <typename Scalar>
using Vector3 = Vector3<Scalar>;

template <typename T>
class DistanceSegmentSegmentTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistanceSegmentSegmentTest);

TYPED_TEST_P(DistanceSegmentSegmentTest, CompareAgainstQSQP) {
  // The squared distance between two segments is the solution to the
  // following box constrained quadratic program:
  //   (segment_a.center + segment_a.direction*u -
  //    segment_b.center - segment_b.direction*v)^2 -> min,
  //   s.t. |u| <= segment_a.half_length, and
  //        |v| <= segment_b.half_length.
  // This test  compares the output of DistanceSquared with
  // solutions computed numerically using the osqp solver.

  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  // OSQP seems to have trouble converging for lower values in singular cases
  // (parallel line segments).
  constexpr double kOSQPTolerance = 1e-6;
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
  ::blue::eigenmath::ProxyTestGenerator gen(
      ::blue::eigenmath::kGeneratorTestSeed);
  ::blue::eigenmath::UniformDistributionVector<Scalar, 3> vector_dist;
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
  // Some configurations need a lot of iterations to converge ..
  settings.max_iter = 100000;
  settings.verbose = absl::GetFlag(FLAGS_verbose);
  // Turn of adaptive_rho, as it depends on run-time and makes the test
  // non-deterministic!
  settings.adaptive_rho = false;
  // This determines "num_variables" (cols) and "num_constraints" (rows).
  instance.constraint_matrix = Eigen::SparseMatrix<double>(2, 2);
  instance.lower_bounds.resize(2);
  instance.upper_bounds.resize(2);
  Eigen::SparseMatrix<double> box_constraint_matrix(2, 2);
  const Eigen::Triplet<double> kTripletsA[] = {
      {0, 0, 1.0}, {1, 0, 0.0}, {0, 1, 0.0}, {1, 1, 1.0}};
  instance.constraint_matrix.setFromTriplets(std::begin(kTripletsA),
                                             std::end(kTripletsA));

  double max_diff = 0.0;

  for (int loop = 0; loop < kNumLoops; loop++) {
    SCOPED_TRACE(::testing::Message() << "At loop " << loop);
    VLOG(2) << "==== loop= " << loop;
    const Segment<Scalar> segment_a{
        .center = ::blue::eigenmath::InterpolateLinearInBox(
            vector_dist(gen), kMinPointPos, kMaxPointPos),
        .direction = ::blue::eigenmath::InterpolateLinearInBox(
                         vector_dist(gen), kMinPointPos, kMaxPointPos)
                         .normalized(),
        .half_length = length_dist(gen)};

    Segment<Scalar> segment_b;
    segment_b.half_length = length_dist(gen);
    int type = 0;
    if (loop % kParallelLinesEvery == 0) {
      type = 1;
      // Force exactly parallel lines.
      segment_b.direction = segment_a.direction;
    } else if (loop % kParallelLinesEvery == 1) {
      type = 2;
      // Force exactly anti-parallel lines.
      segment_b.direction = -segment_a.direction;
    } else if (loop % kParallelLinesEvery == 2) {
      type = 3;

      // Force nearly parallel lines.
      segment_b.direction =
          (segment_a.direction +
           vector_dist(gen).normalized() * kSmallDirectionComponent)
              .normalized();
    } else if (loop % kParallelLinesEvery == 3) {
      type = 4;

      // Force nearly anti-parallel lines.
      segment_b.direction =
          (-segment_a.direction +
           vector_dist(gen).normalized() * kSmallDirectionComponent)
              .normalized();
    } else {
      type = 5;

      // Regular case: pseudo-random direction.
      segment_b.direction = vector_dist(gen).normalized();
    }
    segment_b.center = ::blue::eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos);

    CC_MALLOC_COUNTER_INIT();
    const auto [distance_squared, u, v] =
        SegmentSegmentDistanceSquared(segment_a, segment_b);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

    instance.lower_bounds << -segment_a.half_length, -segment_b.half_length;
    instance.upper_bounds << segment_a.half_length, segment_b.half_length;

    const Scalar off_diag = -2.0 * segment_a.direction.dot(segment_b.direction);
    Eigen::SparseMatrix<double> objective_matrix(2, 2);
    const Eigen::Triplet<double> kTripletsP[] = {
        {0, 0, 2.0}, {1, 0, off_diag}, {0, 1, off_diag}, {1, 1, 2.0}};

    objective_matrix.setFromTriplets(std::begin(kTripletsP),
                                     std::end(kTripletsP));

    instance.objective_matrix = objective_matrix;
    instance.objective_vector.resize(2);
    instance.objective_vector
        << 2.0 * (segment_a.center - segment_b.center).dot(segment_a.direction),
        -2.0 * (segment_a.center - segment_b.center).dot(segment_b.direction);

    VLOG(2) << std::scientific << std::setprecision(18)
            << "segment_a.center= " << segment_a.center.transpose() << "\n"
            << "segment_a.direction= " << segment_a.direction.transpose()
            << "\n"
            << "segment_a.half_length= " << segment_a.half_length << "\n"
            << "segment_b.center= " << segment_b.center.transpose() << "\n"
            << "segment_b.direction= " << segment_b.direction.transpose()
            << "\n"
            << "segment_b.half_length= " << segment_b.half_length << "\n";

    CC_ASSERT_OK(solver.Init(instance, settings));
    const osqp::OsqpExitCode exit_code = solver.Solve();

    ASSERT_EQ(exit_code, osqp::OsqpExitCode::kOptimal);

    const double osqp_distance_squared =
        solver.objective_value() +
        (segment_a.center - segment_b.center).squaredNorm();
    VLOG(2) << std::scientific << std::setprecision(18)
            << "OSQP: solution: " << solver.primal_solution().transpose()
            << "\n"
            << "OSQP: minimum: " << solver.objective_value() << "\n"
            << "OSQP: distance squared: " << osqp_distance_squared << "\n"
            << "distance_squared: " << distance_squared << "\n";

    EXPECT_NEAR(osqp_distance_squared, distance_squared,
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
        std::abs(osqp_distance_squared - static_cast<double>(distance_squared)),
        max_diff);
  }

  VLOG(2) << std::scientific << std::setprecision(18)
          << "max_diff= " << max_diff;
}

REGISTER_TYPED_TEST_SUITE_P(DistanceSegmentSegmentTest, CompareAgainstQSQP);

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

