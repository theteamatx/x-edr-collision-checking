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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_EIGENMATH_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_EIGENMATH_H_
#include "collision_checking/inlining.h"
#include "eigenmath/line_search.h"
#include "eigenmath/pose3.h"
#include "eigenmath/types.h"
#include "eigenmath/vector_utils.h"

namespace collision_checking {

constexpr inline auto kEigenDefaultOptions = ::eigenmath::kDefaultOptions;
constexpr inline auto kMaxEigenVectorCapacity =
    ::eigenmath::kMaxEigenVectorCapacity;

// Vector/Matrix types used in these libraries.
template <typename Scalar>
using Vector2 = ::eigenmath::Vector2<Scalar>;
template <typename Scalar>
using Vector3 = ::eigenmath::Vector3<Scalar>;
template <typename Scalar>
using Vector4 = ::eigenmath::Vector4<Scalar>;
template <typename Scalar>
using VectorN = ::eigenmath::VectorN<Scalar>;
template <typename Scalar>
using VectorX = ::eigenmath::VectorX<Scalar>;
template <typename Scalar>
using Matrix3 = ::eigenmath::Matrix3<Scalar>;
template <typename Scalar>
using Pose3 = ::eigenmath::Pose3<Scalar>;
template <typename Scalar>
using Quaternion = ::eigenmath::Quaternion<Scalar>;
template <typename Scalar>
using SO3 = ::eigenmath::SO3<Scalar>;

using Vector2d = Vector2<double>;
using Vector3d = Vector3<double>;
using Vector4d = Vector4<double>;
using VectorNd = VectorN<double>;
using VectorXd = VectorX<double>;
using Pose3d = Pose3<double>;
using Quaterniond = Quaternion<double>;
using SO3d = SO3<double>;
using Matrix3d = Matrix3<double>;

//
template <typename Functor, typename Scalar>
CC_INLINE std::pair<Scalar, Scalar> GoldenSectionSearchMinimize(
    Scalar left, Scalar right, Functor f, Scalar x_tolerance) {
  return ::eigenmath::GoldenSectionSearchMinimize(left, right, f, x_tolerance);
}

template <typename T>
inline constexpr T Saturate(const T& val, const T& min_max) {
  return std::clamp(val, -min_max, min_max);
}

template <class Scalar>
inline std::array<Vector3<Scalar>, 2> ExtendToOrthonormalBasis(
    const Vector3<Scalar>& u) {
  return ::eigenmath::ExtendToOrthonormalBasis(u);
}

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_EIGENMATH_H_
