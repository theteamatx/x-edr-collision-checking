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

#ifndef COLLISION_CHECKING_GEOMETRY_SHAPES_SHAPE_BASE_H_
#define COLLISION_CHECKING_GEOMETRY_SHAPES_SHAPE_BASE_H_

#include <memory>

#include "collision_checking/eigenmath.h"
#include "collision_checking/logging.h"

namespace collision_checking {
namespace geometry_shapes {

// Supported shapes
enum class ShapeType {
  CAPSULE,          // A capsule, with length and radius.
  SPHERE,           // A sphere with a radius.
  SPHERES,          // A set of spheres.
  BOX,              // A box, defined by its 3 dimensions.
  COMPOSITE_SHAPE,  // A composite of any number of shapes of any type.
};

// A base class for geometry shapes.
class ShapeBase {
 public:
  virtual ~ShapeBase() = default;

  // Returns the shape type.
  ShapeType GetType() const { return type_; }

  // Gets a specific shape type.
  // Downcasts the shape to the specific type provided as a template parameter.
  // T: The shape type to which to downcast.
  // Returns a reference to this object, downcasted to the specific type.
  // Throws if the current shape is not of the provided type.
  template <typename T>
  T const& Get() const;

  const Pose3d& GetLocalTransform() const { return local_transform_; }

  void SetLocalTransform(const Pose3d& local_transform) {
    local_transform_ = local_transform;
  }

  virtual std::unique_ptr<ShapeBase> Clone() const = 0;

 protected:
  explicit ShapeBase(ShapeType type) : type_(type) {}

  ShapeType type_;
  Pose3d local_transform_;
};

// Returns a string representation of `shape_type`.
inline const char* ToString(ShapeType shape_type) {
  switch (shape_type) {
    case ShapeType::CAPSULE:
      return "ShapeType::CAPSULE";
    case ShapeType::SPHERE:
      return "ShapeType::SPHERE";
    case ShapeType::SPHERES:
      return "ShapeType::SPHERES";
    case ShapeType::BOX:
      return "ShapeType::BOX";
    case ShapeType::COMPOSITE_SHAPE:
      return "ShapeType::COMPOSITE_SHAPE";
  }
  return "Invalid ShapeType value";
}

template <typename T>
T const& ShapeBase::Get() const {
  CC_CHECK_EQ(T::kType, GetType(),
              "Requesting incompatible Geometry type: %s != %s",
              ToString(T::kType), ToString(GetType()));
  return static_cast<T const&>(*this);
}

}  // namespace geometry_shapes
}  // namespace collision_checking

#endif  // COLLISION_CHECKING_GEOMETRY_SHAPES_SHAPE_BASE_H_
