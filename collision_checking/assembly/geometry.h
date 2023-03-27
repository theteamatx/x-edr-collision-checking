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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_GEOMETRY_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_GEOMETRY_H_

#include <memory>
#include <string>

#include "collision_checking/geometry_shapes/shape_base.h"
#include "absl/strings/string_view.h"

namespace collision_checking {

class Assembly;
class Link;
class Material;

// The Geometry class represents a drawable geometric object for display or
// for simulation. This could be a primitive box, cylinder or sphere, or
// alternatively it could be an explicit mesh.
//
// Geometry URDF/XML Specification:
// <visual> or <collision>
//   <origin> (optional)
//     See config_node_utils.hpp for details on how transforms are read
//     This represents the transform from the geometry's local coordinate
//     space to the link coordinate frame that it is defined under.
//
//   <geometry> (required)
//     The shape of the object. This can be one of the following:
//     <box>
//       size attribute contains the three side lengths of the box.
//       The origin of the box is in its center.
//     <cylinder>
//       Specify the radius and length. The origin of the cylinder is in its
//       center. The axis of rotation is aligned with the x axis.
//     <sphere>
//       Specify the radius. The origin of the sphere is in its center.
//     <mesh>
//       A mesh element specified by a filename, which will be a unique key into
//       a mesh geometry database supplied by the application.
//
//   <material> (optional)
//     name a name.
//     <color>  base color, use "rgba" parameter.
//     <texture>  texture, use "filename" parameter.
class Geometry {
 private:
  // Noncopyable.
  Geometry(const Geometry&) = delete;
  Geometry& operator=(const Geometry&) = delete;

  // Only the Assembly can create a Geometry.
  friend class Assembly;

  // We need the constructor to be public, so it can be called by make_unique.
  // However, we want to restrict construction to friend classes, so we create a
  // key that can only be constructed by friends.
  struct ConstructionKey {};

 public:
  // The type of the Geometry (VISUAL or COLLISION).
  enum class Type {
    VISUAL,    // for use in rendering.
    COLLISION  // for use in physics.
  };

  // Explicit constructor.
  // key: The construction key to restrict callability of this constructor.
  // assembly: The Assembly which created this Geometry.
  // name: The unique name of this Geometry.
  // type: The geometry type (VISUAL or COLLISION).
  // shape: The geometry shape, will be copied internally.
  // link: The parent Link of this Geometry.
  Geometry(ConstructionKey key, Assembly* assembly, absl::string_view name,
           Type type, const geometry_shapes::ShapeBase& shape, Link* link);

  // Returns the name of the Geometry.
  const std::string& GetName() const { return name_; }

  // Returns the geometry type (Type::COLLISION or Type::VISUAL).
  Type GetType() const { return type_; }

  // Returns the ShapeBase used to create the Geometry.
  const geometry_shapes::ShapeBase& GetShape() const { return *shape_; }
  geometry_shapes::ShapeBase& GetShape() { return *shape_; }

  // Returns the Link that is the parent of this Geometry.
  Link& GetLink() { return *link_; }

  // Returns the const Link that is the parent of this const Geometry.
  const Link& GetLink() const { return *link_; }

  // Assign a material to this geometry, or none if nullptr is used.
  // material: a pointer to the Material, possibly nullptr, to be
  //   assigned to this Geometry.
  void SetMaterial(Material* material) { material_ = material; }

  // Returns a pointer to the Material, possibly nullptr, assigned to this
  // Geometry.
  Material* GetMaterial() { return material_; }

  // Returns a const pointer to the Material, possibly nullptr, assigned to this
  // Geometry.
  const Material* GetMaterial() const { return material_; }

 private:
  Assembly* assembly_ = nullptr;
  std::string name_;
  Type type_;
  std::unique_ptr<geometry_shapes::ShapeBase> shape_;
  Link* link_ = nullptr;
  Material* material_ = nullptr;
};

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_GEOMETRY_H_
