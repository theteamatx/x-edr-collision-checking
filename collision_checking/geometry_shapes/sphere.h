#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_SPHERE_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_SPHERE_H_

#include <memory>

#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry_shapes/shape_base.h"

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
    BLUE_CHECK(radius > 0, "Sphere radius must be positive");
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
