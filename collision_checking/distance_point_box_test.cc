#include "experimental/users/buschmann/collision_checking/distance_point_box.h"

#include <iterator>
#include <limits>

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
class DistancePointBoxTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(DistancePointBoxTest);

TYPED_TEST_P(DistancePointBoxTest, CompareAgainstOSQP) {
  // The squared distance between a point and a box is the solution to the
  // following box constrained quadratic program:
  //   (box_center + box_rotation*u-point)^2 -> min,
  //     s.t. |u_i| <= half_length_i.
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
  ::blue::eigenmath::UniformDistributionSO3<Scalar> rotation_dist;
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
  instance.constraint_matrix = Eigen::SparseMatrix<double>(3, 3);
  instance.lower_bounds.resize(3);
  instance.upper_bounds.resize(3);
  Eigen::SparseMatrix<double> box_constraint_matrix(3, 3);
  const Eigen::Triplet<double> kTripletsA[] = {
      {0, 0, 1.0}, {0, 1, 0.0}, {0, 2, 0.0}, {1, 0, 0.0}, {1, 1, 1.0},
      {1, 2, 0.0}, {2, 0, 0.0}, {2, 1, 0.0}, {2, 2, 1.0}};
  instance.constraint_matrix.setFromTriplets(std::begin(kTripletsA),
                                             std::end(kTripletsA));

  for (int loop = 0; loop < kNumLoops; loop++) {
    SCOPED_TRACE(::testing::Message() << "At loop " << loop);
    VLOG(2) << "==== loop= " << loop;
    Point<Scalar> point = {::blue::eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos)};
    Box<Scalar> box;
    box.half_lengths = vector_dist(gen);
    box.center = ::blue::eigenmath::InterpolateLinearInBox(
        vector_dist(gen), kMinPointPos, kMaxPointPos);

    box.box_rotation_world = rotation_dist(gen).matrix();

    CC_MALLOC_COUNTER_INIT();
    const auto [distance_squared, closest_point] =
        DistanceSquared(point, box);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

    instance.lower_bounds = -box.half_lengths.template cast<double>();
    instance.upper_bounds = box.half_lengths.template cast<double>();

    Eigen::SparseMatrix<double> objective_matrix(3, 3);
    const Eigen::Triplet<double> kTripletsP[] = {
        {0, 0, 2.0}, {0, 1, 0.0}, {0, 2, 0.0}, {1, 0, 0.0}, {1, 1, 2.0},
        {1, 2, 0.0}, {2, 0, 0.0}, {2, 1, 0.0}, {2, 2, 2.0}};

    objective_matrix.setFromTriplets(std::begin(kTripletsP),
                                     std::end(kTripletsP));

    instance.objective_matrix = objective_matrix;
    instance.objective_vector.resize(3);
    instance.objective_vector =
        (2.0 * box.box_rotation_world * (box.center - point.center))
            .template cast<double>();

    CC_ASSERT_OK(solver.Init(instance, settings));
    const osqp::OsqpExitCode exit_code = solver.Solve();

    EXPECT_EQ(exit_code, osqp::OsqpExitCode::kOptimal);

    const Scalar osqp_distance_squared =
        static_cast<Scalar>(solver.objective_value()) +
        (box.center - point.center).squaredNorm();
    VLOG(2) << "point= " << point.center.transpose() << "\n"
            << "box.center= " << box.center.transpose() << "\n"
            << "box.half_lengths= " << box.half_lengths.transpose() << "\n"
            << "box.box_rotation_world= " << box.box_rotation_world << "\n"
            << "closest_point= " << closest_point.transpose() << "\n";
    VLOG(2) << "OSQP: solution: " << solver.primal_solution().transpose()
            << "\n"
            << "OSQP: minimum: " << solver.objective_value() << "\n"
            << "OSQP: distance squared: " << osqp_distance_squared << "\n"
            << "distance_squared: " << distance_squared;
    EXPECT_NEAR(osqp_distance_squared, distance_squared,
                kDistanceSquaredTolerance);
  }
}

REGISTER_TYPED_TEST_SUITE_P(DistancePointBoxTest, CompareAgainstOSQP);

typedef ::testing::Types<float, double> FPTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(DistancePrimitiveTestSuite, DistancePointBoxTest,
                               FPTypes);
}  // namespace
}  // namespace collision_checking
