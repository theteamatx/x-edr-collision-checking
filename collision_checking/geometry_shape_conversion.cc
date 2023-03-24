#include "collision_checking/geometry_shape_conversion.h"

#include "collision_checking/composite_object.h"
#include "collision_checking/geometry_shapes/composite_shape.h"
#include "collision_checking/geometry_shapes/shape_base.h"
#include "collision_checking/geometry_shapes/spheres.h"
#include "third_party/absl/status/status.h"
#include "third_party/absl/strings/str_cat.h"

namespace collision_checking {

absl::Status AddToSphereCount(const geometry_shapes::ShapeBase& shape,
                              std::size_t& sphere_count) {
  switch (shape.GetType()) {
    case geometry_shapes::ShapeType::SPHERE:
      ++sphere_count;
      break;
    case geometry_shapes::ShapeType::SPHERES:
      sphere_count += shape.Get<geometry_shapes::Spheres>().GetSpheres().size();
      break;
    case geometry_shapes::ShapeType::COMPOSITE_SHAPE:
      for (auto& sub_shape :
           shape.Get<geometry_shapes::CompositeShape>().GetShapes()) {
        if (const auto status = AddToSphereCount(*sub_shape, sphere_count);
            !status.ok()) {
          return status;
        }
      }
      break;
    default: {
      return absl::UnimplementedError(
          absl::StrCat("Unsupported geometry type '",
                       geometry_shapes::ToString(shape.GetType()), "'."));
    }
  }
  return absl::OkStatus();
}

absl::Status AddGeometryShapesToPrimitivesCount(
    const geometry_shapes::ShapeBase& shape,
    PrimitivesCount& primitives_count) {
  switch (shape.GetType()) {
    case geometry_shapes::ShapeType::SPHERE:
      primitives_count.num_spheres++;
      break;
    case geometry_shapes::ShapeType::SPHERES:
      primitives_count.num_spheres +=
          shape.Get<geometry_shapes::Spheres>().GetSpheres().size();
      break;
    case geometry_shapes::ShapeType::COMPOSITE_SHAPE:
      for (auto& sub_shape :
           shape.Get<geometry_shapes::CompositeShape>().GetShapes()) {
        if (const auto status = AddGeometryShapesToPrimitivesCount(
                *sub_shape, primitives_count);
            !status.ok()) {
          return status;
        }
      }
      break;
    case geometry_shapes::ShapeType::CAPSULE:
      primitives_count.num_capsules++;
      break;
    case geometry_shapes::ShapeType::BOX:
      primitives_count.num_boxes++;
      break;
    default:
      return absl::UnimplementedError(
          absl::StrCat("Unsupported geometry type '",
                       geometry_shapes::ToString(shape.GetType()), "'."));
  }
  return absl::OkStatus();
}
}  // namespace collision_checking
