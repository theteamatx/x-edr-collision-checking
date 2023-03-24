#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_NORMALIZE_AND_MAYBE_LOG_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_NORMALIZE_AND_MAYBE_LOG_H_

#include "collision_checking/eigenmath.h"
#include "collision_checking/inlining.h"
#include "third_party/absl/log/absl_log.h"

namespace collision_checking {

// Only intended for internal use in collision checking code.
// Not a public interface.
namespace internal {

// Return the normalized vector and LOGD if it is almost zero.
template <typename Derived>
CC_INLINE Vector3<typename Derived::Scalar> NormalizeAndMaybeLog(
    const Eigen::EigenBase<Derived>& input) {
  using Scalar = typename Derived::Scalar;
  const Vector3<Scalar> unnormalized(input);
  const typename Derived::Scalar norm = unnormalized.norm();
  if (norm < Eigen::NumTraits<Scalar>::dummy_precision()) {
#ifndef __CUDACC__
    ABSL_LOG(WARNING) << "Normal computation failed.";
#endif  // __CUDACC__
    return Vector3<Scalar>::Zero();
  }
  return unnormalized / norm;
}

}  // namespace internal
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_NORMALIZE_AND_MAYBE_LOG_H_
