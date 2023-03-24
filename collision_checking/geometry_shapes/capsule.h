#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_CAPSULE_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_CAPSULE_H_

#include <memory>

#include "experimental/users/buschmann/collision_checking/geometry_shapes/shape_base.h"

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
    BLUE_CHECK(length > 0, "Capsule length must be positive");
    BLUE_CHECK(radius > 0, "Capsule radius must be positive");
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
