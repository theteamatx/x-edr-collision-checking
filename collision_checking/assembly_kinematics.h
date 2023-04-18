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

#ifndef COLLISION_CHECKING_ASSEMBLY_KINEMATICS_H_
#define COLLISION_CHECKING_ASSEMBLY_KINEMATICS_H_

// A (forward) kinematics class for collision checking Assemblies.

#include <memory>
#include <string>
#include <type_traits>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/substitute.h"
#include "absl/types/span.h"
#include "collision_checking/assembly/assembly.h"
#include "collision_checking/assembly_coordinate_view.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/inlining.h"
#include "collision_checking/logging.h"
#include "collision_checking/object_id.h"
#include "collision_checking/vector.h"

namespace collision_checking {

// Geometry information required to compute forward kinematics of one link in
// an assembly.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
struct LinkKinematics : public ParametrizedNewDelete<AllocatorTraits> {
  // Index of the parent link in the assembly.
  // -1 means no parent.
  int assembly_parent_link_index = -1;
  // Dof index for parent joint in the joint vector, that is, not the
  // dof index the assembly joint has.
  int parent_joint_dof_index = 0;
  // Parent joint type.
  Joint::Type parent_joint_type = Joint::Type::FIXED;
  // Translation from parent's frame this link's frame to this link's parent
  // joint frame.
  Vector3<Scalar> parent_translation_joint;
  // Rotation from parent's frame this link's to this link's parent joint
  // frame.
  Matrix3<Scalar> parent_rotation_joint;
  // The parent joint's axis (rotation or translation).
  Vector3<Scalar> parent_joint_axis;
  // True if this is an empty leaf node (with no collision geometry).
  bool is_empty_leaf = false;
};

// Geometry information required to compute forward kinematics for an assembly.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
struct AssemblyKinematics : public ParametrizedNewDelete<AllocatorTraits> {
  // Returns an AssemblyLinks object for `assembly` with the given
  // `joint_order`, or a status if an error occurred.
  static absl::StatusOr<AssemblyKinematics> CreateForAssembly(
      const Assembly& assembly, absl::Span<const std::string> joint_order);

  // The link information for the assembly.
  Vector<LinkKinematics<Scalar>, AllocatorTraits> links;

  // Number of joints in the assembly.
  int joint_count = 0;
};

template <typename Scalar>
struct AssemblyPosesView {
  using NonConstScalar = std::remove_const_t<Scalar>;
  using ConstScalar = std::add_const_t<Scalar>;
  using NonConstVector3 = Vector3<NonConstScalar>;
  using NonConstMatrix3 = Matrix3<NonConstScalar>;
  using ConstVector3 = const Vector3<NonConstScalar>;
  using ConstMatrix3 = const Matrix3<NonConstScalar>;
  using MaybeConstVector3 =
      typename std::conditional<std::is_const_v<Scalar>, ConstVector3,
                                NonConstVector3>::type;
  using MaybeConstMatrix3 =
      typename std::conditional<std::is_const_v<Scalar>, ConstMatrix3,
                                NonConstMatrix3>::type;

  CC_INLINE
  std::size_t size() const { return odom_translation_link.size(); }

  // Returns a view on a subspan of poses starting at `pos` and of length `len`.
  CC_INLINE AssemblyPosesView<Scalar> SubView(int pos, int len) {
    CC_CHECK_GE(pos, 0);
    CC_CHECK_LE(len, odom_translation_link.size());
    CC_CHECK_LE(len, odom_rotation_link.size());
    // Can't use subspan in __device__ function.
    if constexpr (std::is_const_v<Scalar>) {
      return AssemblyPosesView<Scalar>{
          .odom_translation_link =
              absl::MakeConstSpan(&odom_translation_link[pos], len),
          .odom_rotation_link =
              absl::MakeConstSpan(&odom_rotation_link[pos], len)};
    } else {
      return AssemblyPosesView<Scalar>{
          .odom_translation_link =
              absl::MakeSpan(&odom_translation_link[pos], len),
          .odom_rotation_link = absl::MakeSpan(&odom_rotation_link[pos], len)};
    }
  }

  absl::Span<MaybeConstVector3> odom_translation_link;
  absl::Span<MaybeConstMatrix3> odom_rotation_link;
};

// Forward kinematics result for an assembly.
// Note: this intentionally doesn't use Pose3/Eigen::Quaternion, as
// the transforms need to be applied many times, which is more efficient with
// the vector + matrix representation.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
struct AssemblyPoses : public ParametrizedNewDelete<AllocatorTraits> {
  AssemblyPoses() = default;
  explicit AssemblyPoses(int pose_count) { Resize(pose_count); }
  // Resizes translation and rotation matrix containers for `pose_count`.
  void Resize(int pose_count) {
    odom_translation_link.resize(pose_count);
    odom_rotation_link.resize(pose_count);
  }

  // Returns the number of poses, which is equal to the number of links in the
  // assembly (but can differ from the number of collision objects in
  // AssemblyCollisionChecker).
  CC_INLINE int GetPoseCount() const { return odom_translation_link.size(); }

  CC_INLINE AssemblyPosesView<Scalar> View() {
    return AssemblyPosesView<Scalar>{
        .odom_translation_link = absl::MakeSpan(&odom_translation_link[0],
                                                odom_translation_link.size()),
        .odom_rotation_link =
            absl::MakeSpan(&odom_rotation_link[0], odom_rotation_link.size())};
  }

  CC_INLINE AssemblyPosesView<const Scalar> ConstView() const {
    return AssemblyPosesView<Scalar>{
        .odom_translation_link = absl::MakeConstSpan(
            &odom_translation_link[0], odom_translation_link.size()),
        .odom_rotation_link = absl::MakeConstSpan(&odom_rotation_link[0],
                                                  odom_rotation_link.size())};
  }

  Vector<Vector3<Scalar>, AllocatorTraits> odom_translation_link;
  Vector<Matrix3<Scalar>, AllocatorTraits> odom_rotation_link;
};

// Computes the `poses` for the given `links` data.
// The layout for `coordinates` matches `AssemblyState::coordinates()`.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
CC_INLINE void ComputePoses(
    const AssemblyKinematics<Scalar, AllocatorTraits>& assembly_kinematics,
    AssemblyCoordinateView<const Scalar> coordinates,
    AssemblyPosesView<Scalar> poses);

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.
template <typename Scalar, typename AllocatorTraits>
absl::StatusOr<AssemblyKinematics<Scalar, AllocatorTraits>>
AssemblyKinematics<Scalar, AllocatorTraits>::CreateForAssembly(
    const Assembly& assembly, absl::Span<const std::string> joint_order) {
  if (assembly.GetLinkCount() > kMaxObjectCount) {
    return absl::OutOfRangeError("Too many links, increase kMaxLinkObjects.");
  }
  AssemblyKinematics result;
  result.links.resize(assembly.GetLinkCount());
  result.joint_count = assembly.GetDofCount();

  Vector<int, AllocatorTraits> assembly_index_to_joint_index;
  assembly_index_to_joint_index.resize(joint_order.size());

  for (size_t idx = 0; idx < joint_order.size(); idx++) {
    const auto& joint_name = joint_order[idx];
    const Joint* joint = assembly.FindJoint(joint_name);
    if (joint == nullptr) {
      return absl::InvalidArgumentError(absl::Substitute(
          "Joint $0 not in assembly but in joint_order vector.", joint_name));
    }
    // This can only happen if a fixed joint was specified in joint_order.
    if (joint->GetDofCount() != 1) {
      return absl::InvalidArgumentError(
          absl::Substitute("Joint `$0`'s number of DOFs != 1 ($1).", joint_name,
                           joint->GetDofCount()));
    }
    // If we get here, the index should be valid.
    CC_CHECK_GE(joint->GetDofIndex(), 0, "joint(%s)->GetDofIndex() (%d.) < 0.",
                joint->GetName(), joint->GetDofIndex());
    CC_CHECK_LT(joint->GetDofIndex(), assembly_index_to_joint_index.size(),
                "joint(%s)->GetDofIndex()= %d, but dof count= %zu.",
                joint->GetName(), joint->GetDofIndex(),
                assembly_index_to_joint_index.size());
    assembly_index_to_joint_index[joint->GetDofIndex()] = idx;
  }

  auto& links = result.links;
  // Assembly link 0 always is the root.
  CC_CHECK_EQ(assembly.GetLink(0).GetParentJoint(), nullptr,
              "Assembly link 0 is not a the root link.");
  links[0].parent_rotation_joint = Matrix3<Scalar>::Identity();
  links[0].parent_translation_joint.setZero();
  links[0].parent_joint_axis.setZero();
  for (size_t idx = 1; idx < assembly.GetLinkCount(); idx++) {
    const Link& link = assembly.GetLink(idx);
    // Non-root links must have a parent.
    CC_CHECK_NE(link.GetParentJoint(), nullptr);
    const Joint& parent_joint = *link.GetParentJoint();
    // Check link ordering precondition for ComputePoses.
    CC_CHECK_LT(parent_joint.GetParentLink().GetIndex(), idx);
    links[idx].assembly_parent_link_index =
        parent_joint.GetParentLink().GetIndex();
    const int parent_dof_index = parent_joint.GetDofIndex();
    if (parent_dof_index == -1) {
      links[idx].parent_joint_dof_index = -1;
    } else {
      links[idx].parent_joint_dof_index =
          assembly_index_to_joint_index[parent_dof_index];
    }
    links[idx].parent_joint_type = parent_joint.GetParameters().type;
    const Pose3<Scalar> parent_pose_joint =
        parent_joint.GetParameters().parent_pose_joint.cast<Scalar>();
    links[idx].parent_rotation_joint = parent_pose_joint.rotationMatrix();
    links[idx].parent_translation_joint = parent_pose_joint.translation();
    links[idx].parent_joint_axis =
        parent_joint.GetParameters().axis.cast<Scalar>();

    // If a link has no collision geometry and no child links, flag that its
    // pose isn't required.
    if (link.GetCollisionGeometries().empty() &&
        link.GetChildJoints().empty()) {
      links[idx].is_empty_leaf = true;
    }
  }
  return result;
}

template <typename Scalar, typename AllocatorTraits>
CC_INLINE void ComputePoses(
    const AssemblyKinematics<Scalar, AllocatorTraits>& assembly_kinematics,
    AssemblyCoordinateView<const Scalar> coordinates,
    AssemblyPosesView<Scalar> poses) {
  const auto& links = assembly_kinematics.links;
  CC_CHECK_EQ(links.size(), poses.odom_rotation_link.size());
  CC_CHECK_EQ(links.size(), poses.odom_translation_link.size());
  CC_CHECK_EQ(coordinates.joint_positions.size(),
              assembly_kinematics.joint_count);

  if (links.size() == 0) {
    return;
  }

  poses.odom_rotation_link[0] =
      coordinates.odom_quaternion_root.toRotationMatrix();
  poses.odom_translation_link[0] = coordinates.odom_translation_root;

  // NOLINT(custom-no-auto-with-eigen) auto is safe here.
  auto& joint_positions = coordinates.joint_positions;

  // TODO: Apply parent_pose_joint to collision shapes at
  // construction and omit it here.
  // construction and avoid one transform here.
  for (int k = 1; k < links.size(); ++k) {
    const auto& link = links[k];
    if (link.is_empty_leaf) {
      continue;
    }
    switch (link.parent_joint_type) {
      case Joint::CONTINUOUS:
      case Joint::REVOLUTE: {
        const Scalar& joint_position_value =
            joint_positions[link.parent_joint_dof_index];
        const Matrix3<Scalar> rotation =
            Eigen::AngleAxis<Scalar>(joint_position_value,
                                     link.parent_joint_axis)
                .matrix();
        poses.odom_rotation_link[k] =
            poses.odom_rotation_link[link.assembly_parent_link_index] *
            link.parent_rotation_joint * rotation;
        poses.odom_translation_link[k] =
            poses.odom_translation_link[link.assembly_parent_link_index] +
            poses.odom_rotation_link[link.assembly_parent_link_index] *
                link.parent_translation_joint;
      } break;
      case Joint::PRISMATIC: {
        const Scalar& joint_position_value =
            joint_positions[link.parent_joint_dof_index];
        const Vector3<Scalar> translation =
            joint_position_value * link.parent_joint_axis;
        poses.odom_rotation_link[k] =
            poses.odom_rotation_link[link.assembly_parent_link_index] *
            link.parent_rotation_joint;
        poses.odom_translation_link[k] =
            poses.odom_translation_link[link.assembly_parent_link_index] +
            poses.odom_rotation_link[link.assembly_parent_link_index] *
                (link.parent_translation_joint +
                 link.parent_rotation_joint * translation);
      } break;
      case Joint::FIXED:
      default:
        poses.odom_rotation_link[k] =
            poses.odom_rotation_link[link.assembly_parent_link_index] *
            link.parent_rotation_joint;
        poses.odom_translation_link[k] =
            poses.odom_translation_link[link.assembly_parent_link_index] +
            poses.odom_rotation_link[link.assembly_parent_link_index] *
                link.parent_translation_joint;
    }
  }
}

}  // namespace collision_checking
#endif  // COLLISION_CHECKING_ASSEMBLY_KINEMATICS_H_
