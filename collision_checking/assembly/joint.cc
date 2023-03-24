#include "collision_checking/assembly/joint.h"

#include <algorithm>

#include "collision_checking/assembly/assembly.h"
#include "collision_checking/assembly/link.h"

namespace collision_checking {

Joint::Joint(ConstructionKey, Assembly* assembly, absl::string_view name,
             const Parameters& params, Link* parent_link, Link* child_link)
    : assembly_(assembly),
      name_(name),
      parameters_(params),
      parent_link_(parent_link),
      child_link_(child_link),
      index_(-1),
      dof_index_(-1) {
  // Normalize quaternions. This should almost always be a no-op.
  parameters_.parent_pose_joint.setQuaternion(
      parameters_.parent_pose_joint.quaternion().normalized());
  if (params.type != FIXED) {
    // Check if axis vector is valid.
    BLUE_CHECK(params.axis.lpNorm<Eigen::Infinity>() > 0,
               "Joint '%s' is not fixed, but has zero axis.", name);
    // Normalize axis vector. This should almost always be a no-op.
    parameters_.axis.normalize();
  }
}

size_t Joint::GetDofCount() const {
  switch (parameters_.type) {
    case Joint::Type::REVOLUTE:
    case Joint::Type::CONTINUOUS:
    case Joint::Type::PRISMATIC:
      return 1;
    case Joint::Type::FIXED:
    default:
      return 0;
  }
}

}  // namespace collision_checking
