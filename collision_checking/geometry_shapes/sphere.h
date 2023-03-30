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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_SPHERE_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_SPHERE_H_

#include <memory>

#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry_shapes/shape_base.h"
#include "collision_checking/logging.h"

namespace collision_checking {
namespace geometry_shapes {

// A Sphere shape
class Sphere : public ShapeBase {
 public:
  static constexpr const ShapeType kType = ShapeType::SPHERE;

  // Constructs a sphere shape.
  // A sphere is represented by it's radius.
  // radius: The radius of the sphere.
  explicit Sphere(double radius) : ShapeBase(kType), radius_(radius) {
    CC_CHECK_GT(radius, 0, "Sphere radius must be positive");
  }
  ~Sphere() override = default;

  std::unique_ptr<ShapeBase> Clone() const override {
    std::unique_ptr<ShapeBase> result(new Sphere(radius_));
    result->SetLocalTransform(local_transform_);
    return result;
  }

  // Returns the radius of the sphere.
  double GetRadius() const { return radius_; }

 private:
  double radius_;
};

}  // namespace geometry_shapes
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_SPHERE_H_
