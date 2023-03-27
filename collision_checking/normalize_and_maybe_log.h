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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_NORMALIZE_AND_MAYBE_LOG_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_NORMALIZE_AND_MAYBE_LOG_H_

#include "collision_checking/eigenmath.h"
#include "collision_checking/inlining.h"
#include "absl/log/absl_log.h"

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
