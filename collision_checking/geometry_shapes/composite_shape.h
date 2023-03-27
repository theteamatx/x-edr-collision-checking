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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_COMPOSITE_SHAPE_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_COMPOSITE_SHAPE_H_

#include <memory>
#include <utility>
#include <vector>

#include "collision_checking/geometry_shapes/shape_base.h"

namespace collision_checking {
namespace geometry_shapes {

// A shape composed of a set of shapes.
class CompositeShape : public ShapeBase {
 public:
  static constexpr const ShapeType kType = ShapeType::COMPOSITE_SHAPE;

  // Constructs a shape from a set of shapes.
  // shapes: A vector of shapes.
  explicit CompositeShape(std::vector<std::unique_ptr<ShapeBase>>&& shapes)
      : ShapeBase(kType), shapes_(std::move(shapes)) {}
  CompositeShape(const CompositeShape& rhs) : ShapeBase(kType), shapes_() {
    shapes_.reserve(rhs.shapes_.size());
    for (const auto& shape : rhs.shapes_) {
      shapes_.emplace_back(shape->Clone());
    }
  }
  CompositeShape& operator=(const CompositeShape& rhs) {
    shapes_.clear();
    shapes_.reserve(rhs.shapes_.size());
    for (const auto& shape : rhs.shapes_) {
      shapes_.emplace_back(shape->Clone());
    }
    return *this;
  }
  CompositeShape(CompositeShape&& rhs) = default;
  CompositeShape& operator=(CompositeShape&& rhs) = default;
  ~CompositeShape() override = default;

  std::unique_ptr<ShapeBase> Clone() const override {
    std::vector<std::unique_ptr<ShapeBase>> shapes_copy;
    shapes_copy.reserve(shapes_.size());
    for (const auto& shape : shapes_) {
      shapes_copy.emplace_back(shape->Clone());
    }
    auto result = std::make_unique<CompositeShape>(std::move(shapes_copy));
    result->SetLocalTransform(local_transform_);
    return result;
  }

  const std::vector<std::unique_ptr<ShapeBase>>& GetShapes() const {
    return shapes_;
  }

 private:
  std::vector<std::unique_ptr<ShapeBase>> shapes_;
};

}  // namespace geometry_shapes
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_COMPOSITE_SHAPE_H_
