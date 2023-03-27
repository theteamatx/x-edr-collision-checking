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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_JOINT_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_JOINT_H_

#include <limits>
#include <string>

#include "collision_checking/eigenmath.h"
#include "absl/strings/string_view.h"

namespace collision_checking {

class Assembly;
class Link;

// The Joint class represents an articulated connection with a single
// scalar degree of freedom that connects two rigid geometric objects, called
// Links. The Joint has a single inboard (closer to base) Link, called its
// parent Link, and a single outboard (father from base), called its child Link.
// Every Joint requires both parent and child Links.
// Joints are specified (created) relative to the reference frame of their
// parent Links.
// The Joint has two coordinate frames, which are identical when the joint's q
// value is zero. These frames are called "inboard" and "outboard". The joint
// is specified (created) in its zero position by specifying the transform
// of the inboard frame with respect to the parent joint's.
//
// Joint URDF/XML Specification:
// PARAMETERS
// name (required)
//  Specifies a unique name of the joint
// type (required)
//  Specifies the type of joint, where type can be one of the following:
//  "revolute" - a hinge joint that rotates along the axis and has a limited
//    range specified by the upper and lower limits.
//  "continuous" - a continuous hinge joint that rotates around the axis and
//    has no upper and lower limits
//  "prismatic" - a sliding joint that slides along the axis, and has a limited
//    range specified by the upper and lower limits.
//  "fixed" - technically a zero-dof joint that does nothing - we temporarily
//    model this as a revolute joint with limits that force it to have a q
//    of zero.
//  "floating" - CURRENTLY UNSUPPORTED
//  "planar" - CURRENTLY UNSUPPORTED
// ELEMENTS
// <parent> (required)
//  Use field "link" to specify the name of the parent link.
// <child> (required)
//  Use field "link" to specify the name of the child link.
// <origin> (optional: defaults to identity if not specified)
//  (See config_node_utils.hpp for how origin is specified)
//  This is the transform from the joint's inboard frame to the parent joint's
//  (or root link's) outboard frame. The joint is located at the origin of the
//  inboard frame, and may operate around or along any axis.
// <axis> (optional: defaults to (1,0,0))
//  The joint axis specified in the joint frame. This is the axis of rotation
//  for revolute joints, the axis of translation for prismatic joints.
//  The axis is specified in the joint inboard frame of reference.
//  xyz (required)
//    Represents the \f$x,y,z\f$ components of a vector. The vector should be
//    normalized.
// <dynamics> (optional)
//  An element specifying physical properties of the joint. These values are
//  used to specify modeling properties of the joint, particularly useful for
//  simulation.
//  damping (optional, defaults to 0)
//    The physical damping value of the joint (\f$\frac{N \cdot s}{m}\f$ for
//    prismatic joints, \f$\frac{N \cdot m \cdot s}{rad}\f$ for revolute
// joints).
//  friction (optional, defaults to 0)
//    The physical static friction value of the joint (\f$N\f$ for prismatic
//    joints, \f$N \cdot m\f$ for revolute joints).
// <limit> (required only for revolute and prismatic joint)
//  An element can contain the following attributes:
//  lower (optional, defaults to 0)
//    An attribute specifying the lower joint limit (radians for revolute
//    joints, meters for prismatic joints). Omit if joint is continuous.
//  upper (optional, defaults to 0)
//    An attribute specifying the upper joint limit (radians for revolute
//    joints, meters for prismatic joints). Omit if joint is continuous.
//  effort (required)
//    An attribute for enforcing the maximum joint effort
//    (\f$|\f$applied effort\f$| < |\f$effort\f$|\f$).
//  velocity (required)
//    An attribute for enforcing the maximum joint velocity.
//  acceleration (optional, defaults to 0, not in official URDF specification)
//    An attribute for enforcing the maximum joint acceleration.
//  jerk (optional, defaults to 0, not in official URDF specification)
//    An attribute for enforcing the maximum joint jerk.
// <calibration> (optional)
//  The reference positions of the joint, usable for calibrating the absolute
//    position of the joint.
//  rising (optional, defaults to 0)
//    When the joint moves in a positive direction, this reference position
//    is to trigger a rising edge.
//  falling (optional, defaults to 0)
//    When the joint moves in a positive direction, this reference position
//    is to trigger a falling edge.
//  home (optional, defaults to 0, not in official URDF specification)
//    The joint's home position for the purposes of bringup.
class Joint {
 private:
  // Noncopyable.
  Joint(const Joint&) = delete;
  Joint& operator=(const Joint&) = delete;

  // Only the Assembly can create a Joint.
  friend class Assembly;

  // We need the constructor to be public, so it can be called by make_unique.
  // However, we want to restrict construction to friend classes, so we create a
  // key that can only be constructed by friends.
  struct ConstructionKey {};

 public:
  enum Type : uint8_t {
    REVOLUTE,
    CONTINUOUS,
    PRISMATIC,
    FIXED
    // Floating & Planar joint types currently unsupported.
  };

  // Struct with the parameters which can be changed after construction.
  struct MutableParameters {
    struct Limits {
      double lower = -std::numeric_limits<double>::infinity();
      double upper = std::numeric_limits<double>::infinity();
      double effort = std::numeric_limits<double>::infinity();
      double velocity = std::numeric_limits<double>::infinity();
      double acceleration = std::numeric_limits<double>::infinity();
      double jerk = std::numeric_limits<double>::infinity();
    } limits;

    struct Dynamics {
      double friction = 0.0;
      double damping = 0.0;
    } dynamics;

    struct Calibration {
      double rising = 0.0;
      double falling = 0.0;
      double home = 0.0;
    } calibration;
  };

  // Struct with the parameters which can *not* be changed after construction.
  struct ImmutableParameters {
    Type type = REVOLUTE;

    // The parent_pose_joint transform represents the transformation
    // of the joint's inboard frame to the parent link's reference frame.
    Pose3d parent_pose_joint = Pose3d{};

    // The axis is specified in the joint's inboard coordinate frame.
    // However, for planar, continuous, and revolute joints, the axis
    // is the same in both the inboard and outboard frames.
    Vector3d axis = Vector3d{1, 0, 0};
  };

  // Struct with all available parameters.
  struct Parameters : public MutableParameters, public ImmutableParameters {};

  // Explicit constructor.
  // key: The construction key to restrict callability of this constructor.
  // assembly: The Assembly which created this Joint.
  // name: The unique name of this Joint.
  // params: The configuration parameters for this Joint.
  // parent_link: The parent Link of this Joint.
  // child_link: The child Link of this Joint.
  Joint(ConstructionKey key, Assembly* assembly, absl::string_view name,
        const Parameters& params, Link* parent_link, Link* child_link);

  const std::string& GetName() const { return name_; }

  // Allow const access to all parameters.
  const Parameters& GetParameters() const { return parameters_; }

  // Allow non-const access only to the mutable parameters.
  MutableParameters& GetMutableParameters() { return parameters_; }

  Link& GetParentLink() { return *parent_link_; }

  const Link& GetParentLink() const { return *parent_link_; }

  Link& GetChildLink() { return *child_link_; }

  const Link& GetChildLink() const { return *child_link_; }

  int32_t GetIndex() const { return index_; }

  size_t GetDofCount() const;

  int32_t GetDofIndex(int32_t dof = 0) const { return dof_index_ + dof; }

 private:
  Assembly* assembly_;
  std::string name_;
  Parameters parameters_;
  Link* parent_link_;
  Link* child_link_;

  int32_t index_;
  int32_t dof_index_;
};

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_JOINT_H_
