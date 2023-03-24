#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_LINK_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_LINK_H_

#include <optional>
#include <string>
#include <vector>

#include "collision_checking/eigenmath.h"
#include "googlex/proxy/iterators/transform_iterator.h"
#include "third_party/absl/strings/string_view.h"

namespace collision_checking {

class Assembly;
class Geometry;
class Joint;

// The Link class represents a Rigid Body with inertial properties. Visual and
// Collision Geometry exists as part of Links.
// Physics calculations require a coordinate frame which is centered
// at the link's centroid, and oriented such that the link's inertial tensor
// is a diagonal matrix.  The RigidTransform link_pose_body establishes
// the transform from the body-aligned inertial reference frame and the
// link's reference frame.
//
// Link URDF/XML Specification:
//
// PARAMETERS
//  name (required)
//    The name of the link itself.
//
// ELEMENTS
//  <inertial> (optional)
//    The inertial properties of the link.
//    <origin> (optional: defaults to identity if not specified)
//      This is the pose of the inertial reference frame, relative to the link
//      reference frame. The origin of the inertial reference frame needs to be
//      at the center of mass. The axes of the inertial reference frame do not
//      need to be aligned with the principal axes of the inertia.
//      See config_node_utils.h for how frames are read.
//    <mass>
//      The mass of the link.
//      value : the scalar value for mass.
//    <inertia>
//      The 3x3 rotational inertia matrix, represented in the inertia frame.
//      Because the rotational inertia matrix is symmetric, only 6 above-
//      diagonal elements of this matrix are specified here, using the
//      attributes below:
//      ixx, ixy, ixz, iyy, iyz, izz : the scalar values for the inertia matrix.
//  <visual> (optional)
//    The visual properties of the link. This element specifies the shape of
//    the object (box, cylinder, etc.) for visualization purposes. Note:
//    multiple instances of <visual> tags can exist for the same link. The
//    union of the geometry they define forms the visual representation of the
//    link.
//    name : (optional)
//      Specifies a name for a part of a link's geometry. This is useful to be
//      able to refer to specific bits of the geometry of a link.
//    <origin> (optional: defaults to identity if not specified)
//      See config_node_utils.h for how frames are read.
//    <geometry> (required)
//      See geometry.h for how geometry is read.
//  <collision> (optional)
//    The collision properties of a link. Note that this can be different from
//    the visual properties of a link, for example, simpler collision models
//    are often used to reduce computation time. Note: multiple instances of
//    <collision> tags can exist for the same link. The union of the geometry
//    they define forms the collision representation of the link.
//    name : (optional)
//      Specifies a name for a part of a link's geometry. This is useful to be
//      able to refer to specific bits of the geometry of a link.
//    <origin> (optional: defaults to identity if not specified)
//      See config_node_utils.h for how frames are read.
//    <geometry> (required)
//      See geometry.h for how geometry is read.
class Link {
 private:
  // Noncopyable.
  Link(const Link&) = delete;
  Link& operator=(const Link&) = delete;

  // Only the Assembly can create a Link.
  friend class Assembly;

  // We need the constructor to be public, so it can be called by make_unique.
  // However, we want to restrict construction to friend classes, so we create a
  // key that can only be constructed by friends.
  struct ConstructionKey {};

 public:
  struct Parameters {
    Pose3d link_pose_inertial = Pose3d();
    double mass = 0.0;
    Matrix3d inertia = Matrix3d::Zero();
  };

  struct Contact {
    bool friction_anchor;
    std::optional<double> stiffness;
    std::optional<double> damping;
    std::optional<double> lateral_friction;
    std::optional<double> rolling_friction;
    std::optional<double> spinning_friction;
  };

  // Explicit constructor.
  // key: A construction key to restrict access to this function.
  // assembly: The assembly which created this Link.
  // name: The unique name of this Link.
  // params: The configuration parameters for this Link.
  Link(ConstructionKey key, Assembly* assembly, absl::string_view name,
       const Parameters& params);

  const std::string& GetName() const { return name_; }

  // PARAMETERS.

  // Returns a reference to the parameters for this Link.
  Parameters& GetParameters() { return parameters_; }

  // Returns a const reference to the parameters for this Link.
  const Parameters& GetParameters() const { return parameters_; }

  // CONTACT.

  // Returns a reference to the contact for this Link.
  std::optional<Contact>& GetContact() { return contact_; }

  // Returns a const reference to the contact for this Link.
  const std::optional<Contact>& GetContact() const { return contact_; }


  // PARENT JOINTS.

  // Returns a pointer to the parent joint, or nullptr if there is none.
  Joint* GetParentJoint() { return parent_joint_; }

  // Returns a const pointer to the parent joint, or nullptr if there is none.
  const Joint* GetParentJoint() const { return parent_joint_; }

  // CHILD JOINTS.

  // Find a child Joint by name.
  // name: - the name of the child Joint to find.
  // Returns a pointer to the found child Joint, nullptr if none found.
  Joint* FindChildJoint(absl::string_view name);

  // Find a const child Joint by name.
  // name: - the name of the child Joint to find.
  // Returns a pointer to the found child Joint, nullptr if none found.
  const Joint* FindChildJoint(absl::string_view name) const;

  // Returns an iterable range of child Joint iterators.
  auto GetChildJoints() { return ::blue::RangeWithDereference(child_joints_); }

  // Returns an iterable range of const child Joint iterators.
  auto GetChildJoints() const {
    return ::blue::RangeWithDereference(child_joints_);
  }

  // VISUAL GEOMETRY.

  // Find a visual Geometry by name.
  // name: - the name of the visual Geometry to find.
  // Returns a pointer to the found visual Geometry, nullptr if none found.
  Geometry* FindVisualGeometry(absl::string_view name);

  // Find a const visual Geometry by name.
  // name: - the name of the visual Geometry to find.
  // Returns a pointer to the found visual Geometry, nullptr if none found.
  const Geometry* FindVisualGeometry(absl::string_view name) const;

  // Returns an iterable range of visual Geometry iterators.
  auto GetVisualGeometries() {
    return ::blue::RangeWithDereference(visual_geometries_);
  }

  // Returns an iterable range of const visual Geometry iterators.
  auto GetVisualGeometries() const {
    return ::blue::RangeWithDereference(visual_geometries_);
  }

  // COLLISION GEOMETRY.

  // Find a collision Geometry by name.
  // name: - the name of the collision Geometry to find.
  // Returns a pointer to the found collision Geometry, nullptr if none found.
  Geometry* FindCollisionGeometry(absl::string_view name);

  // Find a const collision Geometry by name.
  // name: - the name of the collision Geometry to find.
  // Returns a pointer to the found collision Geometry, nullptr if none found.
  const Geometry* FindCollisionGeometry(absl::string_view name) const;

  // Returns an iterable range of collision Geometry iterators.
  auto GetCollisionGeometries() {
    return ::blue::RangeWithDereference(collision_geometries_);
  }

  // Returns an iterable range of const collision Geometry iterators.
  auto GetCollisionGeometries() const {
    return ::blue::RangeWithDereference(collision_geometries_);
  }

  // INDEX.

  int32_t GetIndex() const { return index_; }

 private:
  Assembly* assembly_;
  std::string name_;
  Parameters parameters_;
  std::optional<Contact> contact_;
  Joint* parent_joint_ = nullptr;
  std::vector<Joint*> child_joints_;

  std::vector<Geometry*> visual_geometries_;
  std::vector<Geometry*> collision_geometries_;

  int32_t index_;
};

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_LINK_H_
