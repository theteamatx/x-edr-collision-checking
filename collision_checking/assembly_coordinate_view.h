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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COORDINATE_VIEW_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COORDINATE_VIEW_H_
#include <type_traits>

#include "Eigen/Core"
#include "collision_checking/inlining.h"

namespace collision_checking {

// A utility struct providing maps to Eigen types from a memory area or
// coordinate values used for forward kinematics.
template <typename Scalar>
struct AssemblyCoordinateView {
  // 3 translation + 4 quaternion elements.
  static constexpr int kScalarsPerPose = 7;
  // These typedefs are required so the view works with both const and
  // nonconst Scalar data types.
  using NonConstScalar = std::remove_const_t<Scalar>;
  using NonConstVectorXMap = Eigen::Map<Eigen::VectorX<NonConstScalar>>;
  using ConstVectorXMap = Eigen::Map<const Eigen::VectorX<NonConstScalar>>;
  using MaybeConstVectorXMap =
      typename std::conditional<std::is_const_v<Scalar>, ConstVectorXMap,
                                NonConstVectorXMap>::type;
  using NonConstVector3Map = Eigen::Map<Eigen::Vector3<NonConstScalar>>;
  using ConstVector3Map = Eigen::Map<const Eigen::Vector3<NonConstScalar>>;
  using MaybeConstVector3Map =
      typename std::conditional<std::is_const_v<Scalar>, ConstVector3Map,
                                NonConstVector3Map>::type;
  using NonConstQuaternionMap = Eigen::Map<Eigen::Quaternion<NonConstScalar>>;
  using ConstQuaternionMap =
      Eigen::Map<const Eigen::Quaternion<NonConstScalar>>;
  using MaybeConstQuaternionMap =
      typename std::conditional<std::is_const_v<Scalar>, ConstQuaternionMap,
                                NonConstQuaternionMap>::type;

  AssemblyCoordinateView() = delete;
  AssemblyCoordinateView(const AssemblyCoordinateView&) = default;
  CC_INLINE
  AssemblyCoordinateView(Scalar* data, int joint_count_);

  MaybeConstQuaternionMap odom_quaternion_root;
  MaybeConstVector3Map odom_translation_root;
  MaybeConstVectorXMap joint_positions;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.

template <typename Scalar>
CC_INLINE AssemblyCoordinateView<Scalar>::AssemblyCoordinateView(
    Scalar* data, int joint_count_)
    : odom_quaternion_root(data),
      odom_translation_root(data + 4),
      joint_positions(data + kScalarsPerPose, joint_count_) {}

}  // namespace collision_checking
#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COORDINATE_VIEW_H_
