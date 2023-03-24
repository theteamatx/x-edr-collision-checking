#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_LOGGING_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_LOGGING_H_

#include "experimental/users/buschmann/collision_checking/inlining.h"
#include "third_party/absl/base/attributes.h"
#include "third_party/absl/strings/str_format.h"

// This file contains some simple log and panic macros and functions.
// You'll most likely want to replace these with those with appropriate calls
// for whichever system you are using.

// Panics if condition == false.
#define CC_CHECK(condition, ...)                      \
  do {                                                \
    if (ABSL_PREDICT_FALSE(!(condition))) {           \
      ::collision_checking::Panic(                    \
          {                                           \
              .file = __FILE__,                       \
              .line = __LINE__,                       \
              .function = __PRETTY_FUNCTION__,        \
              .message = "Check failed: " #condition, \
          },                                          \
          ##__VA_ARGS__);                             \
    }                                                 \
  } while (0)

// Panics if `lhs` != `rhs`.
#define CC_CHECK_EQ(lhs, ...) \
  CC_CHECK_COMPARE(lhs, ==, __VA_ARGS__)
// Panics if `lhs` == `rhs`.
#define CC_CHECK_NE(lhs, ...) \
  CC_CHECK_COMPARE(lhs, !=, __VA_ARGS__)
// Panics if `lhs` <= `rhs`.
#define CC_CHECK_LE(lhs, ...) \
  CC_CHECK_COMPARE(lhs, <=, __VA_ARGS__)
// Panics if `lhs` < `rhs`.
#define CC_CHECK_LT(lhs, ...) \
  CC_CHECK_COMPARE(lhs, <, __VA_ARGS__)
// Panics if `lhs` >= `rhs`.
#define CC_CHECK_GE(lhs, ...) \
  CC_CHECK_COMPARE(lhs, >=, __VA_ARGS__)
// Panics if `lhs` > `rhs`.
#define CC_CHECK_GT(lhs, ...) \
  CC_CHECK_COMPARE(lhs, >, __VA_ARGS__)

// Implements CC_CHECK_<op>()
#define CC_CHECK_COMPARE(lhs, op, rhs, ...)               \
  do {                                                           \
    const bool condition = (lhs)op(rhs);                         \
    if (ABSL_PREDICT_FALSE(!(condition))) {                      \
      ::collision_checking::Panic(                               \
          {                                                      \
              .file = __FILE__,                                  \
              .line = __LINE__,                                  \
              .function = __PRETTY_FUNCTION__,                   \
              .message = "Check failed: " #lhs " " #op " " #rhs, \
          },                                                     \
          ##__VA_ARGS__);                                        \
    }                                                            \
  } while (0)

// Unconditionally panics.
#define CC_PANIC(...)               \
  ::collision_checking::Panic(             \
      {                                    \
          .file = __FILE__,                \
          .line = __LINE__,                \
          .function = __PRETTY_FUNCTION__, \
      },                                   \
      ##__VA_ARGS__)

namespace collision_checking {

// Parameters for panic.
struct PanicParameters {
  const char* file = nullptr;
  int line = 0;
  const char* function = nullptr;
  const char* message = nullptr;
};

// A trivial panic function that (optionally) prints an error message and then
// calls std::terminate().
// This is /not/ suitable or intended for production systems as is.
// This ignores any possible issues due to signals, threading and doesn't print
// a strack trace, among other things.
template <typename... Args>
CC_INLINE ABSL_ATTRIBUTE_NORETURN void Panic(
    const PanicParameters& params, const absl::FormatSpec<Args...>& format,
    const Args&... args) {
  absl::FPrintF(stderr, "PANIC at %s:%d", params.file, params.line);
  if (params.message != nullptr) {
    absl::FPrintF(stderr, ": %s\n", params.message);
  }
  constexpr std::size_t kMaxSize = 1024;
  char extra_info[kMaxSize];
  absl::SNPrintF(extra_info, kMaxSize, format, args...);
  absl::FPrintF(stderr, format, args...);
  absl::FPrintF(stderr, "\n");
  fflush(stderr);
  abort();
}
CC_INLINE ABSL_ATTRIBUTE_NORETURN void Panic(
    const PanicParameters& params) {
  absl::FPrintF(stderr, "PANIC at %s:%d", params.file, params.line);
  if (params.message != nullptr) {
    absl::FPrintF(stderr, ": %s", params.message);
  }
  absl::FPrintF(stderr, "\n");
  fflush(stderr);
  abort();
}
#ifdef __CUDACC__
// Simplified CUDA version that doesn't provide formatted printing.
__device__ inline void Panic(const PanicParameters& params) {
  __assertfail(params.message, params.file, params.line, params.function,
               sizeof(char));
}
__device__ inline void Panic(const PanicParameters& params,
                             const absl::FormatSpec<Args...>& format,
                             const Args&... args) {
  Panic(params);
}
#endif

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_LOGGING_H_
