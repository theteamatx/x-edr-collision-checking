#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_TEST_UTILS_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_TEST_UTILS_H_

#include <string>

#include "googlex/proxy/eigenmath/type_checks.h"
#include "third_party/absl/log/absl_check.h"
#include "third_party/absl/status/status.h"
#include "third_party/eigen3/Eigen/Core"
#include "gmock/gmock.h"
#include "third_party/protobuf/text_format.h"
#include "third_party/protobuf/util/message_differencer.h"

namespace blue {
namespace eigenmath {
namespace testing {
/// === begin eigenmath matchers.
// copied here because eigenmath::matchers is incompatible with
// gunit_no_google3. remove this when pushing to github.

namespace matchers_internal {
// Return whether the type T has a difference operator.
template <typename T, typename = void>
static constexpr bool HasDifferenceOp = false;
template <typename T>
static constexpr bool HasDifferenceOp<
    T, std::void_t<decltype(std::declval<T>() - std::declval<T>())>> = true;

template <typename ArgType, typename ExpectedType, typename Scalar>
bool GenericIsApprox(ArgType&& arg, ExpectedType&& expected,
                     Scalar first_tolerance, Scalar second_tolerance,
                     ::testing::MatchResultListener* result_listener) {
  using TestType = std::decay_t<ArgType>;
  if constexpr (IsPose<TestType>) {
    if (expected.isApprox(arg, first_tolerance, second_tolerance)) {
      return true;
    }
    decltype(arg) delta = arg * expected.inverse();
    *result_listener << "\n difference:\n" << delta;
    if constexpr (std::decay_t<
                      decltype(delta.translation())>::RowsAtCompileTime == 2) {
      const decltype(arg.translation()) translation_diff =
          (arg.translation() - expected.translation()).eval();
      *result_listener << "\n translation difference = "
                       << translation_diff.transpose()
                       << " (norm = " << translation_diff.norm() << " m)";
      const auto so2_diff = (expected.so2().inverse() * arg.so2());
      *result_listener << "\n rotation difference = " << so2_diff
                       << " (norm = " << so2_diff.norm() << " rad)";
    } else {
      const decltype(arg.translation()) translation_diff =
          (arg.translation() - expected.translation()).eval();
      *result_listener << "\n translation difference = "
                       << translation_diff.transpose()
                       << " (norm = " << translation_diff.norm() << " m)";
      const auto so3_diff = (expected.so3().inverse() * arg.so3());
      *result_listener << "\n rotation difference = " << so3_diff
                       << " (norm = " << so3_diff.norm() << " rad)";
    }
    return false;
  } else if constexpr (IsQuaternion<TestType>) {
    const TestType relative_quaternion = arg.inverse() * expected;
    if (relative_quaternion.isApprox(TestType::Identity(), first_tolerance)) {
      return true;
    }
    *result_listener << "relative quaternion: " << relative_quaternion;
    *result_listener << ", tolerance= " << first_tolerance;
    return false;
  } else if constexpr (HasDifferenceOp<TestType>) {
    // Ensure that a comparison against a zero matrix/vector does not fail.
    // This is, because isApprox() compares (arg - expected) to the minimum
    // norm of arg and expected, which is 0 in case of a zero vector/matrix.
    // See, for example,
    if ((arg - expected).cwiseAbs().maxCoeff() < first_tolerance) return true;
    return expected.isApprox(arg, first_tolerance);
  } else {
    return expected.isApprox(arg, first_tolerance);
  }
}
}  // namespace matchers_internal

// Returns true if the arg matches the given expected value within the given
// tolerance.
//
// Usage:
// const double kApproxTolerance = 1e-12;
// eigenmath::Pose3d actual_world_t_target;
// eigenmath::Pose3d expected_world_t_target;
//
// EXPECT_THAT(actual_world_t_target,
//             testing::IsApprox(expected_world_t_target, kApproxTolerance));
MATCHER_P2(IsApprox, expected, tolerance,
           "is approximately equal to:\n" + ::testing::PrintToString(expected) +
               "\n with tolerance " + ::testing::PrintToString(tolerance)) {
  return matchers_internal::GenericIsApprox(arg, expected, tolerance, tolerance,
                                            result_listener);
}

// Returns true if the arg matches the given expected value within the default
// tolerance.
//
// Usage:
// eigenmath::Pose3d actual_world_t_target;
// eigenmath::Pose3d expected_world_t_target;
//
// EXPECT_THAT(actual_world_t_target,
//             testing::IsApprox(expected_world_t_target));
MATCHER_P(IsApprox, expected,
          "is approximately equal to\n" + ::testing::PrintToString(expected)) {
  using Scalar = ScalarTypeOf<std::decay_t<decltype(arg)>>;
  return matchers_internal::GenericIsApprox(
      arg, expected, Eigen::NumTraits<Scalar>::dummy_precision(),
      Eigen::NumTraits<Scalar>::dummy_precision(), result_listener);
}

template <typename MeanAndCovarianceType>
auto IsApproxMeanAndCovariance(const MeanAndCovarianceType& expected) {
  return ::testing::AllOf(
      ::testing::Field("mean", &MeanAndCovarianceType::mean,
                       IsApprox(expected.mean)),
      ::testing::Field("covariance", &MeanAndCovarianceType::covariance,
                       IsApprox(expected.covariance)));
}

template <typename Func, typename Container>
auto TransformRangeToVector(Func&& f, const Container& orig) {
  using TransformedType = decltype(f(orig.front()));
  return CopyRange<std::vector<TransformedType>>(
      TransformRange(orig, std::cref(f)));
}

// Returns a matcher that checks if some actual range matches the expected
// range of values using the IsApprox matcher for each element.
template <typename Range>
auto ElementsAreApprox(Range&& expected, double tolerance) {
  return ::testing::ElementsAreArray(TransformRangeToVector(
      [tolerance](const auto& elem) { return IsApprox(elem, tolerance); },
      expected));
}

template <typename Range>
auto UnorderedElementsAreApprox(Range&& expected, double tolerance) {
  return ::testing::UnorderedElementsAreArray(TransformRangeToVector(
      [tolerance](const auto& elem) { return IsApprox(elem, tolerance); },
      expected));
}

// Returns true if the arg is approximately the same eigenvector as the given
// expected vector within the given tolerance.
//
// What makes two vectors the same eigenvector is if they:
//  - have the same magnitude
//  - are colinear
// In other words, a flip in sign of all components is allowed.
//
// Usage:
// const double kApproxTolerance = 1e-12;
// eigenmath::Vector3d actual_eigenvector;
// eigenmath::Vector3d expected_eigenvector;
//
// EXPECT_THAT(actual_eigenvector,
//     testing::IsApproxEigenVector(expected_eigenvector, kApproxTolerance));
MATCHER_P2(IsApproxEigenVector, expected, tolerance,
           "is approximately the same eigenvector as\n" +
               ::testing::PrintToString(expected) + " with tolerance " +
               ::testing::PrintToString(tolerance)) {
  if (arg.dot(expected) < 0.0) {
    return expected.isApprox(-arg, tolerance);
  } else {
    return expected.isApprox(arg, tolerance);
  }
}

// Returns true if the two-tuple arg's members match each other within the
// given tolerance.
//
// This is particularly useful for matching the contents of collections:
//
// const double kApproxTolerance = 1e-12;
// std::vector<eigenmath::Pose3d> actual_collection;
// std::vector<eigenmath::Pose3d> expected_collection;
//
// EXPECT_THAT(actual_collection,
//   ::testing::Pointwise(
//     ::IsApproxTuple(kApproxTolerance),
//     expected_collection))

MATCHER_P(IsApproxTuple, tolerance, "") {
  return matchers_internal::GenericIsApprox(std::get<0>(arg), std::get<1>(arg),
                                            tolerance, tolerance,
                                            result_listener);
}

// Returns true if the two-tuple arg's members match each other within the
// given tolerance.
//
// This is particularly useful for matching the contents of collections:
//
//  std::vector<eigenmath::Pose3d> actual_collection;
//  std::vector<eigenmath::Pose3d> expected_collection;
//
//  EXPECT_THAT(actual_collection,
//    ::testing::Pointwise(
//      ::IsApproxTuple(), expected_collection));
MATCHER(IsApproxTuple, "") {
  using Scalar = ScalarTypeOf<std::decay_t<decltype(std::get<0>(arg))>>;
  return matchers_internal::GenericIsApprox(
      std::get<0>(arg), std::get<1>(arg),
      Eigen::NumTraits<Scalar>::dummy_precision(),
      Eigen::NumTraits<Scalar>::dummy_precision(), result_listener);
}

// Matcher function to compare two poses. Two thresholds are provided, the
// first one determines the threshold for the norm of the delta translation.
// The second one determines the threshold of the delta absolute angle.
//
// eigenmath::Pose3d a;
// eigenmath::Pose3d b;
// double threshold_translation = 0.5;
// double threshold_angle = 0.4;
//
// EXPECT_THAT(a, testing::IsApprox(b,threshold_translation, threshold_angle));
MATCHER_P3(IsApprox, expected, threshold_norm_translation, threshold_angle,
           std::string(negation ? "isn't" : "is") +
               " approximately equal to:\n" +
               ::testing::PrintToString(expected) + "\n with tolerance " +
               ::testing::PrintToString(threshold_norm_translation) + "m and " +
               ::testing::PrintToString(threshold_angle) + "rad") {
  return matchers_internal::GenericIsApprox(arg, expected,
                                            threshold_norm_translation,
                                            threshold_angle, result_listener);
}

// Matches two poses that are part of a collection. This is useful in cases
// where IsApprox doesn't fit. For example:
//
// std::map<int, blue::eigenmath::Pose3d> a, b;
// EXPECT_THAT(a, UnorderedPointwise(FieldPairsAre(Eq(), ApproxEq()), b));
MATCHER(ApproxEq, "") {
  return ::testing::Value(::testing::get<0>(arg),
                          IsApprox(::testing::get<1>(arg)));
}

// Matches two undirected line segments (allows swapped endpoints).
MATCHER_P(IsApproxUndirected, tolerance,
          "has endpoints approximately equal to with tolerance " +
              ::testing::PrintToString(tolerance)) {
  using std::get;
  return (get<1>(arg).from.isApprox(get<0>(arg).from, tolerance) &&
          get<1>(arg).to.isApprox(get<0>(arg).to, tolerance)) ||
         (get<1>(arg).to.isApprox(get<0>(arg).from, tolerance) &&
          get<1>(arg).from.isApprox(get<0>(arg).to, tolerance));
}
template <typename T>
auto IsApproxUndirected(T&& expected, double tolerance)
    -> decltype(::testing::internal::MatcherBindSecond(
        IsApproxUndirected(tolerance), std::forward<T>(expected))) {
  return ::testing::internal::MatcherBindSecond(IsApproxUndirected(tolerance),
                                                std::forward<T>(expected));
}

// Matches two directed line segments.
MATCHER_P(IsApproxDirected, tolerance,
          "has endpoints approximately equal to with tolerance " +
              ::testing::PrintToString(tolerance)) {
  using std::get;
  return (get<1>(arg).from.isApprox(get<0>(arg).from, tolerance) &&
          get<1>(arg).to.isApprox(get<0>(arg).to, tolerance));
}
template <typename T>
auto IsApproxDirected(T&& expected, double tolerance)
    -> decltype(::testing::internal::MatcherBindSecond(
        IsApproxDirected(tolerance), std::forward<T>(expected))) {
  return ::testing::internal::MatcherBindSecond(IsApproxDirected(tolerance),
                                                std::forward<T>(expected));
}

///=== end eigenmath matchers
}  // namespace testing
}  // namespace eigenmath
} // namespace blue

namespace collision_checking {
namespace testing {

#define CC_MALLOC_COUNTER_INIT() \
  ::collision_checking::testing::MallocCounterInitAndClear()
#define CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS()                             \
  EXPECT_TRUE(::collision_checking::testing::MallocCounterGetAllocations() == \
                  0 &&                                                        \
              collision_checking::testing::MallocCounterGetFrees() == 0)      \
      << "Expected no allocs & frees, but got "                               \
      << ::collision_checking::testing::MallocCounterGetAllocations()         \
      << " allocations and "                                                  \
      << collision_checking::testing::MallocCounterGetFrees() << "frees"

// Initializes and clears alloc counter.
void MallocCounterInitAndClear();
// Returns the number of times an allocation function was called.
int MallocCounterGetAllocations();
// Return the number of times a free function was called.
int MallocCounterGetFrees();
// Returns true if malloc checking is available.
bool MallocCounterIsAvailable();

// Extra test utils
// Macros for status tests.201103L
// #define EXPECT_OK(statement) EXPECT_EQ(absl::OkStatus(), (statement))

#define CC_ASSERT_OK(expr)                                                   \
  CC_ASSERT_OK_IMPL(CC_STATUS_MACROS_CONCAT_NAME(_status_value, __COUNTER__), \
                   expr)

#define CC_ASSERT_OK_IMPL(status, expr) \
  auto status = (expr);                 \
  ASSERT_TRUE(status.ok()) << status

#define CC_ASSERT_OK_AND_ASSIGN(lhs, rexpr) \
  CC_ASSERT_OK_AND_ASSIGN_IMPL(             \
      CC_STATUS_MACROS_CONCAT_NAME(_status_or_value, __COUNTER__), lhs, rexpr)

#define CC_ASSERT_OK_AND_ASSIGN_IMPL(statusor, lhs, rexpr)  \
  auto statusor = (rexpr);                                  \
  ASSERT_TRUE(statusor.status().ok()) << statusor.status(); \
  lhs = std::move(statusor).value()

#define CC_STATUS_MACROS_CONCAT_NAME(x, y) CC_STATUS_MACROS_CONCAT_IMPL(x, y)
#define CC_STATUS_MACROS_CONCAT_IMPL(x, y) x##y

// Matches an absl::StatusOrs code to an expected code.
MATCHER_P(StatusCodeIs, expected_code, "") {
  *result_listener << "expected_code= " << expected_code << "status() "
                   << arg.status();
  return arg.status().code() == expected_code;
}

// Macros for geometry tests.
MATCHER_P2(MinimumGeometryInfoIs, expected_center, expected_index, "") {
  *result_listener << "arg.center= " << arg.center.transpose()
                   << "arg.index= " << arg.index
                   << ", arg.has_minimum_distance= "
                   << arg.has_minimum_distance;

  return (arg.has_minimum_distance == true) &&
         blue::eigenmath::testing::matchers_internal::GenericIsApprox(
             arg.center, expected_center, 0, 0, result_listener) &&
         (arg.index == expected_index);
}

namespace testing_details {
// Helper for proto matchers below.
class ProtoMatcher {
 public:
  ProtoMatcher(::testing::MatchResultListener* result_listener,
               const proto2::Message& arg, const proto2::Message& expected)
      : result_listener_(result_listener), arg_(arg), expected_(expected) {
    differencer_.ReportDifferencesToString(&diff_);
    differencer_.set_message_field_comparison(
        proto2::util::MessageDifferencer::EQUIVALENT);
    *result_listener_ << "expected= " << expected_.DebugString()
                      << "\narg= " << arg_.DebugString();
  }
  ProtoMatcher& SetIgnoreRepeatedFieldOrdering() {
    differencer_.set_repeated_field_comparison(
        proto2::util::MessageDifferencer::AS_SET);
    return *this;
  }

  bool MatchAndExplain() {
    if (!differencer_.Compare(arg_, expected_)) {
      *result_listener_ << "diff= " << diff_;
      return false;
    }
    return true;
  }

 private:
  ::testing::MatchResultListener* result_listener_;
  const proto2::Message& arg_;
  const proto2::Message& expected_;
  proto2::util::MessageDifferencer differencer_;
  std::string diff_;
};

}  // namespace testing_details

// Matches to protos using MessageDifferencer's Compare function.
MATCHER_P(ProtoIsEquivTo, expected, "") {
  testing_details::ProtoMatcher matcher(result_listener, arg, expected);
  return matcher.MatchAndExplain();
}

// Matches to protos using MessageDifferencer's Compare function while ignoring
// repeated field ordering ('treat_as_set').
MATCHER_P(ProtoIsEquivToIgnoringRepeatedFieldOrdering, expected, "") {
  testing_details::ProtoMatcher matcher(result_listener, arg, expected);
  matcher.SetIgnoreRepeatedFieldOrdering();
  return matcher.MatchAndExplain();
}

// Parses and returns a proto message from textformat, or returns a status
// if an error occurred.
template <typename Message>
Message ParseTextProtoOrDie(absl::string_view text) {
  Message message;
  ABSL_CHECK(proto2::TextFormat::ParseFromString(text, &message))
      << "Failed to parse proto: " << text;
  return message;
}

}  // namespace testing
}  // namespace collision_checking
#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_TEST_UTILS_H_
