#include "collision_checking/assembly/geometry.h"

#include "third_party/absl/strings/string_view.h"

namespace collision_checking {

Geometry::Geometry(ConstructionKey, Assembly* assembly, absl::string_view name,
                   Type type, const geometry_shapes::ShapeBase& shape,
                   Link* link)
    : assembly_(assembly),
      name_(name),
      type_(type),
      shape_(shape.Clone()),
      link_(link),
      material_(nullptr) {}

}  // namespace collision_checking
