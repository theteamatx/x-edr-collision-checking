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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_SPHERES_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_SPHERES_H_

#include <memory>
#include <vector>

#include "collision_checking/eigenmath.h"
#include "collision_checking/logging.h"
#include "collision_checking/geometry_shapes/shape_base.h"
#include "collision_checking/geometry_shapes/sphere.h"

namespace collision_checking {
namespace geometry_shapes {

// A shape composed of a set of spheres.
class Spheres : public ShapeBase {
 public:
  static constexpr const ShapeType kType = ShapeType::SPHERES;

  // Constructs a shape from a set of spheres.
  // spheres: A vector of spheres.
  explicit Spheres(const std::vector<Sphere>& spheres)
      : ShapeBase(kType), spheres_(spheres) {}
  Spheres(const std::vector<Vector3d>& centers,
          const std::vector<double>& radii)
      : ShapeBase(kType) {
    CC_CHECK_EQ(centers.size(), radii.size(),
               "Number of centers (%d) and radii (%d) do not match!",
               static_cast<int>(centers.size()),
               static_cast<int>(radii.size()));
    for (int i = 0; i < centers.size(); ++i) {
      spheres_.emplace_back(radii[i]).SetLocalTransform(Pose3d(centers[i]));
    }
  }
  ~Spheres() override = default;

  std::unique_ptr<ShapeBase> Clone() const override {
    std::unique_ptr<ShapeBase> result(new Spheres(spheres_));
    result->SetLocalTransform(local_transform_);
    return result;
  }

  const std::vector<Sphere> &GetSpheres() const { return spheres_; }

 private:
  std::vector<Sphere> spheres_;
};

}  // namespace geometry_shapes
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_SPHERES_H_
