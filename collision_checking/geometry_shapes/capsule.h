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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_CAPSULE_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_CAPSULE_H_

#include <memory>

#include "collision_checking/geometry_shapes/shape_base.h"
#include "collision_checking/logging.h"

namespace collision_checking {
namespace geometry_shapes {

// A Capsule shape.
class Capsule : public ShapeBase {
 public:
  static constexpr const ShapeType kType = ShapeType::CAPSULE;

  // Constructs a capsule shape.
  // A capsule is represented by its length and radius.
  // length: The length of the capsule.
  // radius: The radius of the capsule.
  Capsule(double length, double radius)
      : ShapeBase(kType), length_(length), radius_(radius) {
    CC_CHECK_GT(length, 0, "Capsule length must be positive");
    CC_CHECK_GT(radius, 0, "Capsule radius must be positive");
  }
  ~Capsule() override = default;

  std::unique_ptr<ShapeBase> Clone() const override {
    std::unique_ptr<ShapeBase> result(new Capsule(length_, radius_));
    result->SetLocalTransform(local_transform_);
    return result;
  }

  // Returns the length of the capsule.
  double GetLength() const { return length_; }

  // Returns the radius of the capsule.
  double GetRadius() const { return radius_; }

 private:
  double length_;
  double radius_;
};

}  // namespace geometry_shapes
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_CAPSULE_H_
