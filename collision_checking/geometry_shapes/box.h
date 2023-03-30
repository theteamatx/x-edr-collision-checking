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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_BOX_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_BOX_H_

#include <memory>

#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry_shapes/shape_base.h"
#include "collision_checking/logging.h"

namespace collision_checking {
namespace geometry_shapes {

// A Box shape.
class Box : public ShapeBase {
 public:
  static constexpr const ShapeType kType = ShapeType::BOX;

  // Constructs a box shape.
  // size: the full size of the box (lengths on x, y, z axis).
  explicit Box(const Vector3d& size) : ShapeBase(kType), size_(size) {
    CC_CHECK_GT(size.minCoeff(), 0, "Box dimensions must be positive");
  }
  ~Box() override = default;

  std::unique_ptr<ShapeBase> Clone() const override {
    std::unique_ptr<ShapeBase> result(new Box(size_));
    result->SetLocalTransform(local_transform_);
    return result;
  }

  // Returns the full size of the box (lengths on x, y, z axis).
  const Vector3d& GetSize() const { return size_; }

 private:
  Vector3d size_;
};

}  // namespace geometry_shapes
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_BOX_H_
