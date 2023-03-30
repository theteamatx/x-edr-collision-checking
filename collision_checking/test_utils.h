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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_TEST_UTILS_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_TEST_UTILS_H_

#include <string>

#include "absl/log/absl_check.h"
#include "absl/status/status.h"
#include "Eigen/Core"
#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"
#include "eigenmath/matchers.h"

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
         eigenmath::testing::matchers_internal::GenericIsApprox(
             arg.center, expected_center, 0, 0, result_listener) &&
         (arg.index == expected_index);
}

namespace testing_details {
// Helper for proto matchers below.
class ProtoMatcher {
 public:
  ProtoMatcher(::testing::MatchResultListener* result_listener,
               const google::protobuf::Message& arg,
               const google::protobuf::Message& expected)
      : result_listener_(result_listener), arg_(arg), expected_(expected) {
    differencer_.ReportDifferencesToString(&diff_);
    differencer_.set_message_field_comparison(
        google::protobuf::util::MessageDifferencer::EQUIVALENT);
    *result_listener_ << "expected= " << expected_.DebugString()
                      << "\narg= " << arg_.DebugString();
  }
  ProtoMatcher& SetIgnoreRepeatedFieldOrdering() {
    differencer_.set_repeated_field_comparison(
        google::protobuf::util::MessageDifferencer::AS_SET);
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
  const google::protobuf::Message& arg_;
  const google::protobuf::Message& expected_;
  google::protobuf::util::MessageDifferencer differencer_;
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
  ABSL_CHECK(google::protobuf::TextFormat::ParseFromString(text, &message))
      << "Failed to parse proto: " << text;
  return message;
}

// Solution data for the function below.
struct SolveQuadraticResult {
  double minimum = std::numeric_limits<double>::max();
  eigenmath::VectorX<double> solution;
};

// Computes the solution of the box constrained quadratic program:
//   min{1/2*x^T*cost_matrix*x+x^T*cost_vector}
//   s.t. lower_bound <= x <= upper_bound.
// The implementation is intentionally simple and very inefficient:
// it computes the equality constrained solutions for all possible active
// constraint puermutations and picks the minimum among the valid solutions that
// satisfy all constraints.
SolveQuadraticResult SolveBoxQPBruteForce(
    const eigenmath::MatrixXd& cost_matrix,
    const eigenmath::VectorXd& cost_vector,
    const eigenmath::VectorXd& lower_bound,
    const eigenmath::VectorXd& upper_bound);
}  // namespace testing
}  // namespace collision_checking
#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_TEST_UTILS_H_
