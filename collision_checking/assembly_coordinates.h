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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COORDINATES_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COORDINATES_H_

#include "collision_checking/assembly_coordinate_view.h"
#include "collision_checking/inlining.h"
#include "collision_checking/eigenmath.h"
#include "absl/types/span.h"

namespace collision_checking {
// A utility class that holds the position state for forward kinematics.
// This is a wrapper around a flat array containing the root pose and joint
// position coordinates, with setters and getters for root pose and joint
// positions.
template <typename Scalar>
class AssemblyCoordinates {
 public:
  static constexpr int kScalarsPerPose =
      AssemblyCoordinateView<Scalar>::kScalarsPerPose;

  // Default constructor: initializes the state for zero joints.
  CC_INLINE
  AssemblyCoordinates();
  // Construct a state for an assembly `joint_count` 1-dof joints.
  CC_INLINE
  explicit AssemblyCoordinates(int joint_count);
  // Directly construct the state from a given pose and joint positions.
  CC_INLINE
  AssemblyCoordinates(const Pose3<Scalar>& odom_pose_root,
                      Eigen::Ref<const Eigen::VectorX<Scalar>> joint_positions);
  // Resize to `joint_count` 1-dof joints.
  CC_INLINE
  void Resize(int joint_count);
  // Returns the number of joints.
  CC_INLINE
  int JointCount() const;

  // Returns a flattened array of all coordinate values.
  // Use this as an argument to ComputPoses.
  // Coordinate layout: [odom_pose_robot[0].quat(w,x,y,z),
  // odom_pose_robot[0].trans,joint_positions[0];..]
  CC_INLINE
  absl::Span<Scalar> Span();

  CC_INLINE
  absl::Span<const Scalar> ConstSpan() const;

  // Returns a view, useful for setting individual elements.
  CC_INLINE
  AssemblyCoordinateView<Scalar> View();

  // Returns a const view, useful for setting individual elements.
  CC_INLINE
  AssemblyCoordinateView<const Scalar> ConstView() const;

 private:
  // Position data: [odom_pose_root (quaternion;translation);joint_positions]
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1, kEigenDefaultOptions,
                kMaxEigenVectorCapacity +
                    AssemblyCoordinateView<Scalar>::kScalarsPerPose,
                1>
      coordinates_;
  int joint_count_ = 0;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.
template <typename Scalar>
CC_INLINE AssemblyCoordinates<Scalar>::AssemblyCoordinates()
    : AssemblyCoordinates(0) {}

template <typename Scalar>
CC_INLINE AssemblyCoordinates<Scalar>::AssemblyCoordinates(
    int joint_count) {
  Resize(joint_count);
}

template <typename Scalar>
CC_INLINE AssemblyCoordinates<Scalar>::AssemblyCoordinates(
    const Pose3<Scalar>& odom_pose_root,
    const Eigen::Ref<const Eigen::VectorX<Scalar>> joint_positions) {
  joint_count_ = joint_positions.size();
  coordinates_.resize(joint_count_ + kScalarsPerPose);
  auto view = View();
  view.odom_quaternion_root = odom_pose_root.quaternion();
  view.odom_translation_root = odom_pose_root.translation();
  view.joint_positions = joint_positions;
}

template <typename Scalar>
CC_INLINE void AssemblyCoordinates<Scalar>::Resize(int joint_count) {
  joint_count_ = joint_count;
  coordinates_.resize(joint_count + kScalarsPerPose);
  coordinates_.setZero();
  View().odom_quaternion_root.setIdentity();
}

template <typename Scalar>
CC_INLINE int AssemblyCoordinates<Scalar>::JointCount() const {
  return joint_count_;
}

template <typename Scalar>
CC_INLINE AssemblyCoordinateView<Scalar>
AssemblyCoordinates<Scalar>::View() {
  return AssemblyCoordinateView<Scalar>(coordinates_.data(), joint_count_);
}

template <typename Scalar>
CC_INLINE AssemblyCoordinateView<const Scalar>
AssemblyCoordinates<Scalar>::ConstView() const {
  return AssemblyCoordinateView<const Scalar>(coordinates_.data(),
                                              joint_count_);
}

template <typename Scalar>
CC_INLINE absl::Span<Scalar> AssemblyCoordinates<Scalar>::Span() {
  return absl::MakeSpan(coordinates_.data(), coordinates_.size());
}

template <typename Scalar>
CC_INLINE absl::Span<const Scalar>
AssemblyCoordinates<Scalar>::ConstSpan() const {
  return absl::MakeConstSpan(coordinates_.data(), coordinates_.size());
}

}  // namespace collision_checking
#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COORDINATES_H_
