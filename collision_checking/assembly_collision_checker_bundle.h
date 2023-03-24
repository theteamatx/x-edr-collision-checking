#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COLLISION_CHECKER_BUNDLE_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COLLISION_CHECKER_BUNDLE_H_

#include "experimental/users/buschmann/collision_checking/assembly_collision_checker.h"
#include "experimental/users/buschmann/collisions_config.proto.h"
#include "experimental/users/buschmann/collision_checking/assembly/assembly.h"
#include "experimental/users/buschmann/collision_checking/assembly/kinematics_assembly.proto.h"
#include "experimental/users/buschmann/collision_checking/assembly/parts_config.proto.h"
#include "experimental/users/buschmann/collision_checking/assembly/proto_utils.h"
#include "googlex/proxy/kinematics/joint_mapping.h"
#include "googlex/proxy/module_system/robot_config/robot_config.proto.h"
#include "third_party/absl/status/status.h"
#include "third_party/absl/status/statusor.h"
#include "util/task/status_macros.h"

namespace blue {
namespace collision_checking {

// A bundle of Assembly, AssemblyCollisionChecker, Scratch and Result objects.
template <typename Scalar = float,
          typename AllocatorTraits = DefaultAllocatorTraits>
struct AssemblyCollisionCheckerBundle {
  AssemblyCollisionChecker<Scalar, DefaultAllocatorTraits> checker;
  CollisionResult<Scalar, DefaultAllocatorTraits> result;
  typename AssemblyCollisionChecker<Scalar, DefaultAllocatorTraits>::Scratch
      scratch;
  VoxelMapObject<Scalar> obstacles;
  kinematics::JointMapping joint_mapping;
  kinematics::Assembly assembly;
  AssemblyId assembly_id;
};

// Returns an AssemblyCollisionCheckerBundle with configuration from
// robot_config, or a status on error.
template <typename Scalar = float,
          typename AllocatorTraits = DefaultAllocatorTraits>
absl::StatusOr<AssemblyCollisionCheckerBundle<Scalar, DefaultAllocatorTraits>>
CreateAssemblyCollisionCheckerBundle(
    const common::RobotConfig& robot_config,
    const ModelOptions<Scalar>& checker_options,
    const QueryOptions& query_options) {
  if (!robot_config.HasExtension(hilo::kinematics_assembly)) {
    return absl::InvalidArgumentError("Missing assembly config.");
  }
  if (!robot_config.HasExtension(hilo::parts_config)) {
    return absl::InvalidArgumentError("Missing parts config.");
  }
  if (!robot_config.HasExtension(hilo::collisions_config)) {
    return absl::InvalidArgumentError("Missing collisions config.");
  }

  AssemblyCollisionCheckerBundle<Scalar, DefaultAllocatorTraits> bundle;

  ASSIGN_OR_RETURN(bundle.assembly,
                   kinematics::FromProto(
                       robot_config.GetExtension(hilo::kinematics_assembly)));
  ASSIGN_OR_RETURN(
      bundle.joint_mapping,
      kinematics::JointMapping::Create(
          bundle.assembly, robot_config.GetExtension(hilo::parts_config)));

  ASSIGN_OR_RETURN(
      bundle.assembly_id,
      bundle.checker.AddAssembly(
          bundle.assembly, bundle.joint_mapping.JointNames(), checker_options));

  for (const auto& pair :
       robot_config.GetExtension(hilo::collisions_config).disabled_pairs()) {
    RETURN_IF_ERROR(
        bundle.checker.DisableCollisionPair(pair.link_a(), pair.link_b())
            .ToAbslStatus());
  }

  bundle.result = bundle.checker.CreateCollisionResult(query_options);
  bundle.scratch = bundle.checker.CreateScratch();

  return bundle;
}

}  // namespace collision_checking
}  // namespace blue

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COLLISION_CHECKER_BUNDLE_H_
