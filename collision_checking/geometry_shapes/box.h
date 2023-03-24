#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_BOX_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_GEOMETRY_SHAPES_BOX_H_

#include <memory>

#include "experimental/users/buschmann/collision_checking/eigenmath.h"
#include "experimental/users/buschmann/collision_checking/geometry_shapes/shape_base.h"

namespace collision_checking {
namespace geometry_shapes {

// A Box shape.
class Box : public ShapeBase {
 public:
  static constexpr const ShapeType kType = ShapeType::BOX;

  // Constructs a box shape.
  // size: the full size of the box (lengths on x, y, z axis).
  explicit Box(const Vector3d& size) : ShapeBase(kType), size_(size) {
    BLUE_CHECK(size.minCoeff() > 0, "Box dimensions must be positive");
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
