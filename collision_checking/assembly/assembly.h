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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_ASSEMBLY_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_ASSEMBLY_H_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "collision_checking/assembly/geometry.h"
#include "collision_checking/assembly/joint.h"
#include "collision_checking/assembly/link.h"
#include "collision_checking/geometry_shapes/shape_base.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/container/flat_hash_map.h"
#include "genit/transform_iterator.h"

namespace collision_checking {

// The Assembly class represents a kinematic skeleton.
// It is a directed, acylic graph (DAG) of Links connected together by Joints.
// The Assembly matches the organization and XML syntax of URDF.
// The Assembly provides creation functions for Joints and Links, which allows
// it to manage the memory for any Joints and Links that are a part of it. It
// is the base structure on top of which forward and inverse kinematics are
// defined.
class Assembly {
 public:
  // Create an assembly with no links or joints.
  // This can be used as a basis for manually constructing an assembly.
  // name: the name of the assembly, optional, defaults to "assembly".
  explicit Assembly(absl::string_view name = "assembly");

  // Returns the name of this Assembly.
  const std::string& GetName() const { return name_; }

  // Create a link explicitly.
  // name: the name of the new link.
  // params: the configuration of the new link.
  // Returns a reference to the newly created Link.
  Link& CreateLink(absl::string_view name, const Link::Parameters& params);

  // Find a Link by name.
  // name: the name of the Link to find.
  // Returns a pointer to the found Link, nullptr if none found.
  Link* FindLink(absl::string_view name);

  // Find a const link by name.
  // name: the name of the link to find.
  // Returns a pointer to the found Link, nullptr if none found.
  const Link* FindLink(absl::string_view name) const;

  // Get a link by its index in the assembly.
  // index: the index of the link.
  // Returns a reference to the link.
  Link& GetLink(std::size_t index);

  // Get a const link by its index in the assembly.
  // index: the index of the link.
  // Returns a const reference to the link.
  const Link& GetLink(std::size_t index) const;

  // Returns an iterable range of Link iterators.
  auto GetLinks() { return genit::RangeWithDereference(links_); }

  // Returns an iterable range of const Link iterators.
  auto GetLinks() const { return genit::RangeWithDereference(links_); }

  // Get the number of links in the assembly.
  // Returns the number of links.
  std::size_t GetLinkCount() const { return links_.size(); }

  // Returns a reference to the root link.
  Link& GetRootLink() {
    CC_CHECK_NE(root_link_, nullptr);
    return *root_link_;
  }

  // Returns a const reference to the root link.
  const Link& GetRootLink() const {
    CC_CHECK_NE(root_link_, nullptr);
    return *root_link_;
  }

  // Create a joint explicitly.
  // name: the name of the new joint.
  // params: the configuration of the new joint.
  // parent_link_name: the name of the parent link of this joint.
  // child_link_name: the name of the child link of this joint.
  // Returns a reference to the newly created Joint.
  Joint& CreateJoint(absl::string_view name, const Joint::Parameters& params,
                     absl::string_view parent_link_name,
                     absl::string_view child_link_name);

  // Find a Joint by name.
  // name: the name of the Joint to find.
  // Returns a pointer to the found Joint, nullptr if none found.
  Joint* FindJoint(absl::string_view name);

  // Find a const joint by name.
  // name: the name of the joint to find.
  // Returns a pointer to the found Joint, nullptr if none found.
  const Joint* FindJoint(absl::string_view name) const;

  // Get a joint by its index in the assembly.
  // index: the index of the joint.
  // Returns a reference to the joint.
  const Joint& GetJoint(std::size_t index) const { return *joints_[index]; }

  // Returns an iterable range of Joint iterators.
  auto GetJoints() { return genit::RangeWithDereference(joints_); }

  // Returns an iterable range of const Joint iterators.
  auto GetJoints() const { return genit::RangeWithDereference(joints_); }

  // Get the number of joints in the assembly.
  // Returns the number of joints.
  std::size_t GetJointCount() const { return joints_.size(); }

  // Create a Geometry explicitly.
  // name: the name of the new geometry.
  //  The name may be modified internally to make it unique.
  // type: the type of geometry (Type::COLLISION or Type::VISUAL).
  // shape: the shape of the new geometry.
  // link: the parent link of this geometry.
  // Returns a reference to the newly created Geometry.
  Geometry& CreateGeometry(absl::string_view name, Geometry::Type type,
                           const geometry_shapes::ShapeBase& shape, Link* link);

  // Find a Geometry by name.
  // name: the name of the Geometry to find.
  // Returns a pointer to the found Geometry, nullptr if none found.
  Geometry* FindGeometry(absl::string_view name);

  // Find a const geometry by name.
  // name: the name of the geometry to find.
  // Returns a pointer to the found Geometry, nullptr if none found.
  const Geometry* FindGeometry(absl::string_view name) const;

  // Returns an iterable range of Geometry iterators.
  auto GetGeometries() { return genit::RangeWithDereference(geometries_); }

  // Returns an iterable range of const Geometry iterators
  auto GetGeometries() const {
    return genit::RangeWithDereference(geometries_);
  }

  // VISUAL GEOMETRIES.

  // A predicate for a filter iterator over just the visual geometries.
  struct IsVisualGeometry {
    bool operator()(const Geometry& geometry) const {
      return geometry.GetType() == Geometry::Type::VISUAL;
    }
  };

  // Returns an iterable range of Visual Geometry iterators.
  auto GetVisualGeometries() {
    return FilterRange(GetGeometries(), IsVisualGeometry());
  }

  // Returns an iterable range of const Visual Geometry iterators.
  auto GetVisualGeometries() const {
    return FilterRange(GetGeometries(), IsVisualGeometry());
  }

  // Create one composite shape with all visual geometries.
  // The assembly must be fixed, i.e., GetDofCount() == 0.
  absl::StatusOr<std::unique_ptr<geometry_shapes::ShapeBase>>
  MakeFixedVisualGeometry() const;

  // COLLISION GEOMETRIES.

  // A predicate for a filter iterator over just the collision geometries.
  struct IsCollisionGeometry {
    bool operator()(const Geometry& geometry) const {
      return geometry.GetType() == Geometry::Type::COLLISION;
    }
  };

  // Returns an iterable range of Collision Geometry iterators.
  auto GetCollisionGeometries() {
    return FilterRange(GetGeometries(), IsCollisionGeometry());
  }

  // Returns an iterable range of const Collision Geometry iterators.
  auto GetCollisionGeometries() const {
    return FilterRange(GetGeometries(), IsCollisionGeometry());
  }

  // Create one composite shape with all collision geometries.
  // The assembly must be fixed, i.e., GetDofCount() == 0.
  absl::StatusOr<std::unique_ptr<geometry_shapes::ShapeBase>>
  MakeFixedCollisionGeometry() const;

  // FINALIZATION.

  // Finalizes an assembly after all components have been created.
  // Creates proper indices for all of the links and joints, by topologically
  // sorting starting from each of the root links (links with no parent joints)
  // The links are sorted first and indexed, and then the joints are sorted
  // using the link indices to order them.
  // The link indices and the joint indices will generally not be identical,
  // and because we allow for merging trees as well as branching trees,
  // there's no explicit, unique association of a particular joint to a
  // particular link.
  absl::Status Finalize();

  // Returns whether or not the assembly has been finalized.
  bool IsFinalized() const { return finalized_; }

  // QUERIES.

  // Branching is where a link has multiple child joints. This is common
  // in robotic assemblies.
  // Returns true if there is branching, false otherwise.
  bool HasBranching() const;

  // A Serial Chain is one in which there is only one root link, and the
  // assembly has no branching and no merging.  This means that
  // every link has less than 2 parent joints, and less than 2 child joints.
  // Returns true if assembly is a serial chain, false otherwise.
  bool IsSerialChain() const;

  // Returns the number of degrees of freedom in the whole assembly.
  size_t GetDofCount() const;

  // Return a copy of this assembly.
  // If auto_finalize == true, the returned assembly will be finalized if this
  // one is. Otherwise, it will not be finalized.
  std::unique_ptr<Assembly> Clone(bool auto_finalize = true) const;

 private:
  friend class Link;
  friend class Joint;
  friend class Geometry;
  friend class Material;

  absl::Status TopologicallySortLinks();
  void VisitLink(Link* link, std::deque<Link*>* sorted_links);

  absl::StatusOr<std::unique_ptr<geometry_shapes::ShapeBase>> MakeFixedGeometry(
      bool visual_shapes) const;


  std::string name_;

  std::vector<std::unique_ptr<Link>> links_;
  absl::flat_hash_map<std::string, Link*> link_map_;

  Link* root_link_ = nullptr;

  std::vector<std::unique_ptr<Joint>> joints_;
  absl::flat_hash_map<std::string, Joint*> joint_map_;

  std::vector<std::unique_ptr<Geometry>> geometries_;
  absl::flat_hash_map<std::string, Geometry*> geometry_map_;

  bool finalized_ = false;
};

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_ASSEMBLY_H_
