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

#include "collision_checking/assembly/link.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_format.h"
#include "collision_checking/assembly/assembly.h"
#include "collision_checking/assembly/geometry.h"
#include "collision_checking/assembly/joint.h"
#include "collision_checking/logging.h"

namespace collision_checking {
namespace {

void CheckValidInertiaMatrix(absl::string_view link_name,
                             const Matrix3d& inertia) {
  const double kEpsilon = 10.0 * std::numeric_limits<double>::epsilon();

  auto inertia_as_string = [&]() {
    return absl::StrFormat(
        "[%.20e %.20e %.20e;\n"
        "%.20e %.20e %.20e;\n"
        "%.20e %.20e %.20e]\n",
        inertia(0, 0), inertia(0, 1), inertia(0, 2), inertia(1, 0),
        inertia(1, 1), inertia(1, 2), inertia(2, 0), inertia(2, 1),
        inertia(2, 2));
  };

  for (int i = 0; i < 3; i++) {
    // Diagonal must be positive (or zero within epsilon).
    CC_CHECK_GE(inertia(i, i), -kEpsilon,
                "Invalid inertia tensor, I(%d,%d) = %e < -%e\n  I = \n%s", i, i,
                inertia(i, i), kEpsilon, inertia_as_string().c_str());

    // Tensor must be symmetric within epsilon.
    int b1 = i % 2 + 1;  // 1, 2, 1
    int b2 = i & 0x2;    // 0, 0, 2
    int b3 = 2 - i;      // 2, 1, 0
    CC_CHECK_LE(std::abs(inertia(b1, b2) - inertia(b2, b1)), kEpsilon,
                "%s has invalid inertia tensor, |I(%d,%d) - I(%d,%d)| = %e > "
                "%e\n  I = \n%s",
                link_name, b1, b2, b2, b1,
                std::abs(inertia(b1, b2) - inertia(b2, b1)), kEpsilon,
                inertia_as_string().c_str());

    // Tensor diagonals must obey triangular inequality.
    CC_CHECK_GE(
        inertia(b1, b1) + inertia(b2, b2) - inertia(b3, b3), -kEpsilon,
        "%s has invalid inertia tensor, I(%d,%d) + I(%d,%d) - I(%d,%d) = %e < "
        "-%e\n  I = \n%s",
        link_name, b1, b1, b2, b2, b3, b3,
        inertia(b1, b1) + inertia(b2, b2) - inertia(b3, b3), kEpsilon,
        inertia_as_string().c_str());
  }

  // Check positive semi-definitiveness within epsilon by checking 2x2 and 3x3
  // determinants (1x1 already checked as diagonal).
  // Account for round-off-error in determinant calculation.
  const double k2x2Epsilon = 64.0 * kEpsilon;
  const double k3x3Epsilon = 128.0 * kEpsilon;
  const double det2x2 = inertia.block<2, 2>(0, 0).determinant();
  CC_CHECK_GE(det2x2, -k2x2Epsilon,
              "Invalid inertia tensor, |I(0:1,0:1)| = %e < -%e\n  I = \n%s",
              det2x2, k2x2Epsilon, inertia_as_string().c_str());
  CC_CHECK_GE(inertia.determinant(), -k3x3Epsilon,
              "Invalid inertia tensor, |I| = %e < -%e\n  I = \n%s",
              inertia.determinant(), k3x3Epsilon, inertia_as_string().c_str());
}
}  // namespace

Link::Link(ConstructionKey, Assembly* assembly, absl::string_view name,
           const Parameters& params)
    : assembly_(assembly), name_(name), parameters_(params), index_(-1) {
  // Normalize quaternions. This should almost always be a no-op.
  parameters_.link_pose_inertial.setQuaternion(
      parameters_.link_pose_inertial.quaternion().normalized());
  CheckValidInertiaMatrix(name, params.inertia);
}

Joint* Link::FindChildJoint(absl::string_view name) {
  auto found = std::find_if(
      child_joints_.begin(), child_joints_.end(),
      [&name](const Joint* joint) -> bool { return joint->GetName() == name; });
  if (found != child_joints_.end()) {
    return *found;
  } else {
    return nullptr;
  }
}

const Joint* Link::FindChildJoint(absl::string_view name) const {
  auto found = std::find_if(
      child_joints_.begin(), child_joints_.end(),
      [&name](const Joint* joint) -> bool { return joint->GetName() == name; });
  if (found != child_joints_.end()) {
    return *found;
  } else {
    return nullptr;
  }
}

Geometry* Link::FindVisualGeometry(absl::string_view name) {
  auto found =
      std::find_if(visual_geometries_.begin(), visual_geometries_.end(),
                   [&name](const Geometry* geometry) -> bool {
                     return geometry->GetName() == name;
                   });
  if (found != visual_geometries_.end()) {
    return *found;
  } else {
    return nullptr;
  }
}

const Geometry* Link::FindVisualGeometry(absl::string_view name) const {
  auto found =
      std::find_if(visual_geometries_.begin(), visual_geometries_.end(),
                   [&name](const Geometry* geometry) -> bool {
                     return geometry->GetName() == name;
                   });
  if (found != visual_geometries_.end()) {
    return *found;
  } else {
    return nullptr;
  }
}

Geometry* Link::FindCollisionGeometry(absl::string_view name) {
  auto found =
      std::find_if(collision_geometries_.begin(), collision_geometries_.end(),
                   [&name](const Geometry* geometry) -> bool {
                     return geometry->GetName() == name;
                   });
  if (found != collision_geometries_.end()) {
    return *found;
  } else {
    return nullptr;
  }
}

const Geometry* Link::FindCollisionGeometry(absl::string_view name) const {
  auto found =
      std::find_if(collision_geometries_.begin(), collision_geometries_.end(),
                   [&name](const Geometry* geometry) -> bool {
                     return geometry->GetName() == name;
                   });
  if (found != collision_geometries_.end()) {
    return *found;
  } else {
    return nullptr;
  }
}

}  // namespace collision_checking
