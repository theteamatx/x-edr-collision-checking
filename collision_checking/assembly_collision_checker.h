#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COLLISION_CHECKER_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COLLISION_CHECKER_H_

// Convenience functions and classes for collision checking with data from
// a Assembly class.

#include <string>
#include <vector>

#include "collision_checking/assembly/assembly.h"
#include "collision_checking/assembly/assembly_connectivity.h"
#include "collision_checking/assembly_coordinates.h"
#include "collision_checking/assembly_id.h"
#include "collision_checking/assembly_kinematics.h"
#include "collision_checking/collision_checking_scratch.h"
#include "collision_checking/compute_collisions.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/geometry_shape_conversion.h"
#include "collision_checking/id_assigner.h"
#include "collision_checking/inlining.h"
#include "collision_checking/model_interface.h"
#include "collision_checking/model_options.h"
#include "collision_checking/object_id.h"
#include "collision_checking/options.h"
#include "collision_checking/proto_utils.h"
#include "collision_checking/status.h"
#include "collision_checking/vector.h"
#include "collision_checking/voxel_map_object_name.h"
#include "third_party/absl/algorithm/container.h"
#include "third_party/absl/container/btree_map.h"
#include "third_party/absl/functional/bind_front.h"
#include "third_party/absl/status/status.h"
#include "third_party/absl/status/statusor.h"
#include "third_party/absl/strings/str_cat.h"
#include "third_party/absl/strings/str_format.h"
#include "third_party/absl/strings/string_view.h"
#include "third_party/absl/strings/substitute.h"

namespace collision_checking {

// A class for performing collision checking on a robot model loaded from a
// Assembly class.
// Uses virtual inheritance from ModelInterface so that can also serve as a
// base to BatchCheckerInterface, for which this template can also be a useful
// base class providing a partial implementation.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
class AssemblyCollisionChecker : virtual public ModelInterface<Scalar>,
                                 public ParametrizedNewDelete<AllocatorTraits> {
 public:
  using AssemblyKinematics = AssemblyKinematics<Scalar, AllocatorTraits>;
  using AssemblyPoses = AssemblyPoses<Scalar, AllocatorTraits>;
  using CollisionResult = CollisionResult<Scalar, AllocatorTraits>;
  using Options = typename ModelInterface<Scalar>::Options;
  using Scratch = Scratch<Scalar, AllocatorTraits>;
  using VoxelMapObject = VoxelMapObject<Scalar>;

  // Returns an appropriately sized Scratch structure.
  Scratch CreateScratch() const;

  // Resizes `scratch` to match sizes of *this, if necessary.
  void ResizeScratch(Scratch& scratch) const;

  // Returns an appropriately sized Result structure for queries according to
  // `options`. If normals & contact points should be computed, the default
  // argument must be overridden.
  CollisionResult CreateCollisionResult(
      QueryOptions options = QueryOptions()) const;

  // Resizes `result` to match sizes of *this, if necessary.
  void ResizeCollisionResult(CollisionResult& result) const;

  // TODO(b/208409848): Add support for multiple assemblies.
  absl::StatusOr<AssemblyId> AddAssembly(
      const Assembly& assembly, absl::Span<const std::string> joint_order,
      const Options& options) override;

  absl::Status RemoveAssembly(AssemblyId id) override;

  absl::StatusOr<ObjectId> AddStaticObject(
      absl::string_view name, const Pose3<Scalar>& world_pose_object,
      absl::Span<const geometry_shapes::ShapeBase*const> shapes,
      Scalar padding) override;

  absl::StatusOr<ObjectId> AddRelativeObject(
      absl::string_view name, ObjectId reference_object_id,
      const Pose3<Scalar>& reference_pose_object,
      absl::Span<const geometry_shapes::ShapeBase*const> shapes,
      Scalar padding) override;

  absl::Status RemoveRelativeObject(ObjectId id) override;

  absl::Status SetRelativeObjectPose(
      ObjectId id, const Pose3<Scalar>& reference_pose_object) override;

  void SetAABBPadding(Scalar padding) override { aabb_padding_ = padding; }

  Scalar GetAABBPadding() const override { return aabb_padding_; }

  bool IsValidAssembly(AssemblyId id) const override;

  bool IsValidObject(ObjectId id) const override;

  absl::string_view GetObjectName(ObjectId id) const override;

  absl::string_view GetObjectNameFromIdSet(ObjectIdSet set) const override;

  ObjectType GetObjectType(absl::string_view name) const override;
  ObjectType GetObjectType(ObjectId id) const override;
  ObjectType GetObjectType(ObjectIdSet set) const override;
  ObjectId GetObjectId(absl::string_view name) const override;
  int GetObjectIndexForName(absl::string_view name) const override;
  int GetObjectIndexForId(ObjectId id) const override;
  absl::string_view GetAssemblyName(AssemblyId id) const override;
  AssemblyId GetAssemblyId(absl::string_view name) const override;
  std::vector<std::string> GetObjectNamesFromSet(
      ObjectIdSet set) const override;
  Status GetObjectSetFromObjectNames(absl::Span<const std::string> names,
                                     ObjectIdSet& set) const override;

  Status MaskConnectedObjects() override;
  Status DisableCollisionPair(absl::string_view object_a,
                              absl::string_view object_b) override;
  Status AddToObjectInclusionSet(ObjectId id, ObjectIdSet set_to_add) override;
  Status AddToObjectInclusionSet(ObjectId id, ObjectId id_to_add) override {
    return AddToObjectInclusionSet(id, ObjectIdSet(id_to_add));
  }
  Status RemoveFromObjectInclusionSet(ObjectId id,
                                      ObjectIdSet set_to_remove) override;
  Status RemoveFromObjectInclusionSet(ObjectId id,
                                      ObjectId id_to_remove) override {
    return RemoveFromObjectInclusionSet(id, ObjectIdSet(id_to_remove));
  }
  Status GetObjectFlags(const ObjectId& object_id,
                        ObjectFlags& flags) const override;
  std::vector<ObjectFlags> GetAllObjectFlags() const override;
  Status SetObjectFlags(const ObjectFlags& flag) override;
  Status SetObjectFlags(absl::Span<const ObjectFlags> flags) override;
  std::string GetFilterDebugString() const override;
  void RemoveAllAttachments() override;
  void RemoveAllStaticObjects() override;

  // Perform collision check according to the given `coordinates` and
  // obstacles. The `scratch` and `result` structures have to be
  // appropriately sized. Can be used from real-time.
  Status ComputeCollisions(const VoxelMapObject& obstacles,
                           AssemblyCoordinateView<const Scalar> coordinates,
                           const QueryOptions& options, Scratch& scratch,
                           CollisionResult& result) const;
  // Overload for the function above.
  Status ComputeCollisions(const VoxelMapObject& obstacles,
                           const Pose3<Scalar>& odom_pose_root,
                           const VectorN<Scalar>& joint_positions,
                           const QueryOptions& options, Scratch& scratch,
                           CollisionResult& result) const {
    return ComputeCollisions(
        obstacles,
        AssemblyCoordinates<Scalar>(odom_pose_root, joint_positions)
            .ConstView(),
        options, scratch, result);
  }

  // Compute forward kinematics, using the cached assembly geometry.
  // See
  // google3/collision_checking/assembly_kinematics.h
  // For details on the layout for coordinates.
  CC_INLINE
  void ComputePoses(AssemblyCoordinateView<const Scalar> coordinates,
                    AssemblyPosesView<Scalar> poses) const;
  // Overload for the function above.
  CC_INLINE
  void ComputePoses(const Pose3<Scalar>& odom_pose_root,
                    const VectorN<Scalar>& joint_positions,
                    AssemblyPosesView<Scalar> poses) const {
    return ComputePoses(
        AssemblyCoordinates<Scalar>(odom_pose_root, joint_positions)
            .ConstView(),
        poses);
  }

  void TransformObjects(Scratch& scratch) const;

  const CollisionObjects<Scalar, AllocatorTraits>&
  GetObjectsAtZeroConfiguration() const {
    return objects_zero_;
  }

  // Returns a CollisionStateProto for the given `scratch` and `result` objects,
  // or a Status if an error occurred.
  // Uses the internal object id to name maps to populate object names in the
  // proto.
  absl::StatusOr<CollisionStateProto> GetCollisionStateProto(
      const VoxelMapObject& obstacles, const Scratch& scratch,
      const CollisionResult& result) const;

 protected:
  // This function is called by all implementations of ModelInterface
  // functions that modify model data.
  // Override this in a derived class if any dependent data needs to be updated
  // as a consequence.
  virtual void FinalizeModelChange() {}

 private:
  using CompositeObject = CompositeObject<Scalar, AllocatorTraits>;

  // Bookkeeping information for an assembly in the collision model.
  struct AssemblyInfo {
    AssemblyId id = AssemblyIdAssigner::kInvalidId;
    std::string name;
    AssemblyKinematics assembly_kinematics;
    std::size_t assembly_dof_count;
    std::size_t assembly_link_count;
    int result_object_offset = 0;
    AssemblyConnectivity link_connectivity;
  };

  // Supported object kinematics types.
  enum class ObjectKinematicsType { kInvalid, kAssembly, kRelative };

  // Bookkeeping information for an Object in the collision model.
  struct ObjectInfo {
    AssemblyId assembly_id;
    std::string name;
    ObjectId object_id = kInvalidObjectId;
    ObjectKinematicsType kinematics_type = ObjectKinematicsType::kInvalid;
    ObjectId reference_object_id = kInvalidObjectId;
    CompositeObject object;
    int object_index = 0;
    int assembly_link_index = 0;
    Pose3<Scalar> reference_pose_object;
    PrimitivesCount primitive_counts;
  };

  // Information for the pose of a non-assembly collision object relative to
  // it's reference position.
  struct RelativeObjectPoseInfo {
    int reference_object_result_index = -1;
    Pose3<Scalar> reference_pose_object;
  };

  // Position and size indices, for use in spans/ranges.
  struct PosAndSize {
    int pos = 0;
    int size = 0;
  };

  struct ResultIndexInfo {
    PosAndSize assembly_objects;
    PosAndSize relative_objects;
  };

  void FinalizeGeometryChange();
  ObjectType ToObjectType(const ObjectInfo& info) const {
    if (info.kinematics_type == ObjectKinematicsType::kRelative) {
      if (info.reference_object_id == kInvalidObjectId) {
        return ObjectType::kStatic;
      }
      return ObjectType::kAttachment;
    }
    return ObjectType::kAssembly;
  }

  // Collision geometry at the zero joint angle configuration.
  Scalar aabb_padding_ = Scalar{0};

  // Use a btree_map so the iteration order (and hence runtimes) are
  // deterministic.
  absl::btree_map<ObjectId, ObjectInfo> object_info_;

  // Useable in RT/on device.
  ResultIndexInfo object_index_info_;
  Vector<RelativeObjectPoseInfo, AllocatorTraits> relative_object_pose_info_;
  CollisionObjects<Scalar, AllocatorTraits> objects_zero_;

  // TODO(b/208409848) Support multiple assemblies.
  AssemblyInfo assembly_info_;
  ObjectIdAssigner object_ids_assigner_;
  AssemblyIdAssigner assembly_id_assigner_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.
template <typename Scalar, typename AllocatorTraits>
typename AssemblyCollisionChecker<Scalar, AllocatorTraits>::Scratch
AssemblyCollisionChecker<Scalar, AllocatorTraits>::CreateScratch() const {
  Scratch scratch(object_info_.size());
  // Assign objects in zero configuration, which will both resize
  // scratch.objects_ and assign constant parameters (such as radii).
  scratch.transformed_objects = objects_zero_;
  return scratch;
}

template <typename Scalar, typename AllocatorTraits>
void AssemblyCollisionChecker<Scalar, AllocatorTraits>::ResizeScratch(
    Scratch& scratch) const {
  scratch.Resize(object_info_.size());
  // Assign objects in zero configuration, which will both resize
  // scratch.objects_ and assign constant parameters (such as radii).
  scratch.transformed_objects = objects_zero_;
}

template <typename Scalar, typename AllocatorTraits>
CollisionResult<Scalar, AllocatorTraits>
AssemblyCollisionChecker<Scalar, AllocatorTraits>::CreateCollisionResult(
    QueryOptions options) const {
  return CollisionResult(objects_zero_.objects.size(), options);
}

template <typename Scalar, typename AllocatorTraits>
void AssemblyCollisionChecker<Scalar, AllocatorTraits>::ResizeCollisionResult(
    CollisionResult& result) const {
  result.Allocate(objects_zero_.objects.size());
}

template <typename Scalar, typename AllocatorTraits>
absl::StatusOr<AssemblyId>
AssemblyCollisionChecker<Scalar, AllocatorTraits>::AddAssembly(
    const Assembly& assembly, absl::Span<const std::string> joint_order,
    const Options& options) {
  // TODO(b/208409848) Handle multiple assemblies.
  if (assembly_info_.id != AssemblyIdAssigner::kInvalidId) {
    return absl::UnimplementedError(
        "Multiple assembly support not implemented.");
  }
  if (assembly_id_assigner_.NumFreeIds() == 0) {
    return absl::FailedPreconditionError("No more free AssemblyIds.");
  }
  if (joint_order.size() != assembly.GetDofCount()) {
    return absl::InvalidArgumentError(absl::Substitute(
        "assembly ($0) and joint_order ($1) have different size",
        assembly.GetDofCount(), joint_order.size()));
  }

  if (assembly.GetLinkCount() > object_ids_assigner_.NumFreeIds()) {
    return absl::OutOfRangeError("Too many links, increase kMaxLinkObjects.");
  }

  aabb_padding_ = options.GetAABBPadding();

  AssemblyInfo assembly_info{.id = assembly_id_assigner_.GetFreeId(),
                             .name = assembly.GetName(),
                             .assembly_dof_count = assembly.GetDofCount(),
                             .assembly_link_count = assembly.GetLinkCount(),
                             .link_connectivity = AssemblyConnectivity(
                                 assembly, /*max_movable_joints=*/1)};
  // Should be prevented by check above.
  CC_CHECK_NE(assembly_info.id, AssemblyIdAssigner::kInvalidId);

  if (const auto status_or =
          AssemblyKinematics::CreateForAssembly(assembly, joint_order);
      status_or.ok()) {
    assembly_info.assembly_kinematics = std::move(*status_or);
  } else {
    return status_or.status();
  }

  // Extract geometry into resized buffers.
  for (int link_index = 0; link_index < assembly.GetLinkCount(); ++link_index) {
    const auto& link = assembly.GetLink(link_index);
    // Save mappings between id, index and name. These are only used for
    // utility functions.
    ObjectInfo info{.assembly_id = assembly_info.id,
                    .name = link.GetName(),
                    .object_id = object_ids_assigner_.GetFreeId(),
                    .kinematics_type = ObjectKinematicsType::kAssembly,
                    .object_index = -1,  // Assigned in FinalizeGeometryChange()
                    .assembly_link_index = link_index};
    // This should be prevented by the check of free ids above.
    CC_CHECK_NE(info.object_id, ObjectIdAssigner::kInvalidId);
    info.object.flags.id_set = ObjectIdSet(info.object_id);

    for (const auto& geo : link.GetCollisionGeometries()) {
      if (const auto status = AddGeometryShapesToPrimitivesCount(
              geo.GetShape(), info.primitive_counts);
          !status.ok()) {
        return status;
      }
    }

    // If a link has no collision geometry, disable it.
    if (link.GetCollisionGeometries().empty()) {
      // We currently don't have cases with empty links that are not leaves
      // but have only empty child links uptree.
      // If we ever get those, this should be generalized to include that case.
      if (link.GetChildJoints().empty()) {
        info.object.flags.inclusion_set.Clear();
      }
    } else {
      info.object.flags.inclusion_set = ~info.object.flags.id_set;
    }

    PrimitivesCount primitives_indices;
    info.object.ResizeBuffers(info.primitive_counts);
    // TODO(b/148435774): Apply parent_pose_joint to collision shapes.
    for (const auto& geo : link.GetCollisionGeometries()) {
      if (const auto status = AddGeometryShapesToCompositeObject(
              Pose3d::Identity(), geo.GetShape(), options.GetAssemblyPadding(),
              info.object, primitives_indices);
          !status.ok()) {
        return status;
      }
    }
    object_info_[info.object_id] = info;
  }

  assembly_info_ = assembly_info;

  FinalizeGeometryChange();

  switch (options.GetMaskingPolicy()) {
    case CollisionMaskingPolicy::kNothing:
      break;
    case CollisionMaskingPolicy::kUnknown:
      [[fallthrough]];
    case CollisionMaskingPolicy::kConnectedLinks: {
      const Status status = MaskConnectedObjects();
      if (!status.ok()) {
        RemoveAssembly(assembly_info.id).IgnoreError();
        return status.ToAbslStatus();
      }
    } break;
  }

  FinalizeModelChange();

  return assembly_info.id;
}

template <typename Scalar, typename AllocatorTraits>
absl::Status AssemblyCollisionChecker<Scalar, AllocatorTraits>::RemoveAssembly(
    AssemblyId id) {
  if (id != assembly_info_.id) {
    return absl::FailedPreconditionError(
        absl::StrCat("Assembly id (", id.value(), ") not present."));
  }
  assembly_id_assigner_.ReleaseId(id);
  for (auto it = object_info_.begin(); it != object_info_.end();) {
    if (it->second.assembly_id == id) {
      object_ids_assigner_.ReleaseId(it->second.object_id);
      it = object_info_.erase(it);
    } else {
      ++it;
    }
  }

  assembly_info_.id = AssemblyIdAssigner::kInvalidId;
  assembly_info_.name = assembly_info_.assembly_kinematics.joint_count = 0;
  assembly_info_.assembly_kinematics.links.resize(0);
  assembly_info_.assembly_dof_count = 0;
  assembly_info_.assembly_link_count = 0;

  FinalizeGeometryChange();
  FinalizeModelChange();
  return absl::OkStatus();
}

template <typename Scalar, typename AllocatorTraits>
absl::StatusOr<ObjectId>
AssemblyCollisionChecker<Scalar, AllocatorTraits>::AddStaticObject(
    absl::string_view name, const Pose3<Scalar>& world_pose_object,
    absl::Span<const geometry_shapes::ShapeBase*const> shapes, Scalar padding) {
  return AddRelativeObject(name, kInvalidObjectId, world_pose_object, shapes,
                           padding);
}

template <typename Scalar, typename AllocatorTraits>
absl::StatusOr<ObjectId>
AssemblyCollisionChecker<Scalar, AllocatorTraits>::AddRelativeObject(
    absl::string_view name, const ObjectId reference_object_id,
    const Pose3<Scalar>& reference_pose_object,
    absl::Span<const geometry_shapes::ShapeBase*const> shapes,
    const Scalar padding) {
  if (object_ids_assigner_.NumFreeIds() == 0) {
    return absl::FailedPreconditionError("No more free object ids.");
  }

  // Populate ObjectInfo for the new static (non-assembly) object.
  ObjectInfo info{.assembly_id = AssemblyIdAssigner::kInvalidId,
                  .name = std::string(name),
                  .object_id = object_ids_assigner_.GetFreeId(),
                  .kinematics_type = ObjectKinematicsType::kRelative,
                  .reference_object_id = reference_object_id,
                  .object_index = -1,  // assigned in FinalizeGeometryChange
                  .assembly_link_index = -1,
                  .reference_pose_object = reference_pose_object};

  // This should be prevented by the check for free ids above.
  CC_CHECK_NE(info.object_id, ObjectIdAssigner::kInvalidId);
  info.object.flags.id_set = ObjectIdSet(info.object_id);
  info.object.flags.inclusion_set = ~info.object.flags.id_set;
  // Disable collisions between reference object and all objects reachable
  // by fixed joints + one movable joint.
  if (reference_object_id != ObjectIdAssigner::kInvalidId) {
    auto reference_info_it = object_info_.find(reference_object_id);
    if (reference_info_it == object_info_.end()) {
      object_ids_assigner_.ReleaseId(info.object_id);
      return absl::FailedPreconditionError(
          absl::StrCat("reference_object_id (", reference_object_id.value(),
                       ") is not valid."));
    }
    if (reference_info_it->second.assembly_id ==
        AssemblyIdAssigner::kInvalidId) {
      object_ids_assigner_.ReleaseId(info.object_id);
      return absl::FailedPreconditionError(
          "reference_object_id must refer to an assembly object.");
    }
    ObjectInfo& reference_info = reference_info_it->second;
    ObjectIdSet reachable_set;
    absl::Span<const int> connected_link_indices =
        assembly_info_.link_connectivity.GetConnectedLinkIndices(
            reference_info.assembly_link_index);
    for (int connected_index : connected_link_indices) {
      auto it =
          absl::c_find_if(object_info_, [connected_index](const auto& it) {
            return it.second.assembly_link_index == connected_index;
          });
      CC_CHECK_NE(
          it, object_info_.end(),
          "Assembly index from connectivity (%d) not found in object_ids_.",
          connected_index);
      reachable_set.Insert(it->second.object_id);
    }
    // Remove all 'reachable' objects from collision checks with the
    // relative object/attachment.
    info.object.flags.inclusion_set.Remove(reachable_set);

    // Remove the relative object from all 'reachable' objects.
    for (int connected_index : connected_link_indices) {
      auto it =
          absl::c_find_if(object_info_, [connected_index](const auto& it) {
            return it.second.assembly_link_index == connected_index;
          });
      CC_CHECK_NE(
          it, object_info_.end(),
          "Assembly index from connectivity (%d) not found in object_ids_.",
          connected_index);
      it->second.object.flags.inclusion_set.Remove(info.object_id);
    }
  } else {
    // Remove all static objects and the VoxelMap from collision with each
    // other.
    ObjectIdSet static_objects_set =
        ObjectIdSet(info.object_id).Insert(kVoxelMapIdSet);
    for (const auto& [_, obj_info] : object_info_) {
      if (obj_info.kinematics_type == ObjectKinematicsType::kRelative &&
          obj_info.reference_object_id == kInvalidObjectId) {
        static_objects_set.Insert(obj_info.object_id);
      }
    }
    info.object.flags.inclusion_set.Remove(static_objects_set);
    for (auto& [_, obj_info] : object_info_) {
      if (obj_info.kinematics_type == ObjectKinematicsType::kRelative &&
          obj_info.reference_object_id == kInvalidObjectId) {
        obj_info.object.flags.inclusion_set.Remove(static_objects_set);
      }
    }
  }

  for (const auto& shape : shapes) {
    if (auto status =
            AddGeometryShapesToPrimitivesCount(*shape, info.primitive_counts);
        !status.ok()) {
      object_ids_assigner_.ReleaseId(info.object_id);
      return status;
    }
  }
  info.object.ResizeBuffers(info.primitive_counts);
  PrimitivesCount primitives_indices;
  for (const auto& shape : shapes) {
    if (auto status = AddGeometryShapesToCompositeObject(
            Pose3d::Identity(), *shape, padding, info.object,
            primitives_indices);
        !status.ok()) {
      object_ids_assigner_.ReleaseId(info.object_id);
      return status;
    }
  }

  SetEmptySize(&info.object.aabb);
  UpdateAlignedBox(aabb_padding_, info.object);

  // Insert the object & finalize the model update.
  object_info_[info.object_id] = info;
  FinalizeGeometryChange();
  FinalizeModelChange();

  return info.object_id;
}

template <typename Scalar, typename AllocatorTraits>
absl::Status
AssemblyCollisionChecker<Scalar, AllocatorTraits>::RemoveRelativeObject(
    ObjectId id) {
  const auto it = object_info_.find(id);
  if (it == object_info_.end()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ObjectId id (", id.value(), ") not present."));
  }

  object_ids_assigner_.ReleaseId(id);
  object_info_.erase(it);
  FinalizeGeometryChange();
  FinalizeModelChange();

  return absl::OkStatus();
}

template <typename Scalar, typename AllocatorTraits>
absl::Status
AssemblyCollisionChecker<Scalar, AllocatorTraits>::SetRelativeObjectPose(
    const ObjectId id, const Pose3<Scalar>& reference_pose_object) {
  const auto it = object_info_.find(id);
  if (it == object_info_.end()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ObjectId id (", id.value(), ") not present."));
  }
  if (it->second.assembly_id != AssemblyIdAssigner::kInvalidId) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Object with id (", id.value(), ")  is not a relative object."));
  }

  // Also update the value in the flat pose vector used in ComputeCollisions.
  // The pose is also maintained in ObjectInfo to simplify preserving the pose
  // across geometry changes.
  ObjectInfo& info = it->second;
  info.reference_pose_object = reference_pose_object;
  SetEmptySize(&info.object.aabb);
  UpdateAlignedBox(aabb_padding_, info.object);

  const int relative_pose_index =
      info.object_index - object_index_info_.relative_objects.pos;
  CC_CHECK(relative_pose_index >= 0 &&
               relative_pose_index < relative_object_pose_info_.size(),
           "relative_pose_index= %d, relative_object_pose_info_= %zu.",
           relative_pose_index, relative_object_pose_info_.size());

  relative_object_pose_info_[relative_pose_index].reference_pose_object =
      reference_pose_object;
  FinalizeModelChange();

  return absl::OkStatus();
}

template <typename Scalar, typename AllocatorTraits>
void AssemblyCollisionChecker<Scalar,
                              AllocatorTraits>::FinalizeGeometryChange() {
  objects_zero_.objects.resize(object_info_.size());

  // Reset indices and sizes to zero.
  object_index_info_ = {};

  for (auto& [id, info] : object_info_) {
    switch (info.kinematics_type) {
      case ObjectKinematicsType::kAssembly:
        ++object_index_info_.assembly_objects.size;
        break;
      case ObjectKinematicsType::kRelative:
        ++object_index_info_.relative_objects.size;
        break;
      default:
        CC_PANIC("Uninitialized kinematics_type.");
    }
  }

  object_index_info_.relative_objects.pos =
      object_index_info_.assembly_objects.size;

  relative_object_pose_info_.resize(object_index_info_.relative_objects.size);

  // Assign indices in the fixed order AssemblyObjects, RelativeObjects.
  int object_index = 0;
  for (auto& [id, info] : object_info_) {
    if (info.kinematics_type != ObjectKinematicsType::kAssembly) {
      continue;
    }
    info.object_index = object_index;
    objects_zero_.objects[object_index].ResizeBuffers(info.primitive_counts);
    objects_zero_.objects[object_index] = info.object;
    object_index++;
  }
  for (auto& [id, info] : object_info_) {
    if (info.kinematics_type != ObjectKinematicsType::kRelative) {
      continue;
    }
    info.object_index = object_index;
    objects_zero_.objects[object_index].ResizeBuffers(info.primitive_counts);
    objects_zero_.objects[object_index] = info.object;

    RelativeObjectPoseInfo& relative_pose_info =
        relative_object_pose_info_[object_index -
                                   object_index_info_.relative_objects.pos];
    relative_pose_info.reference_pose_object = info.reference_pose_object;
    relative_pose_info.reference_object_result_index = -1;

    if (info.reference_object_id != kInvalidObjectId) {
      auto it = object_info_.find(info.reference_object_id);
      CC_CHECK_NE(it, object_info_.end());
      relative_pose_info.reference_object_result_index =
          it->second.object_index;
    }

    object_index++;
  }
}

template <typename Scalar, typename AllocatorTraits>
bool AssemblyCollisionChecker<Scalar, AllocatorTraits>::IsValidAssembly(
    AssemblyId id) const {
  return assembly_info_.id == id;
}

template <typename Scalar, typename AllocatorTraits>
bool AssemblyCollisionChecker<Scalar, AllocatorTraits>::IsValidObject(
    ObjectId id) const {
  return object_info_.find(id) != object_info_.end();
}

template <typename Scalar, typename AllocatorTraits>
absl::string_view
AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetObjectName(
    ObjectId id) const {
  const auto it = object_info_.find(id);
  if (it == object_info_.end()) {
    return absl::string_view();
  }

  return it->second.name;
}
template <typename Scalar, typename AllocatorTraits>
absl::string_view
AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetObjectNameFromIdSet(
    ObjectIdSet set) const {
  // Empty set: return an empty string.
  if (set.Count() != 1) {
    return absl::string_view();
  }
  const auto it = absl::c_find_if(object_info_, [&set](const auto& kv) {
    return ObjectIdSet(kv.first) == set;
  });
  if (it == object_info_.end()) {
    if (kVoxelMapIdSet.Overlaps(set)) {
      return kVoxelMapObjectName;
    }
    return absl::string_view();
  }
  return it->second.name;
}
template <typename Scalar, typename AllocatorTraits>
ObjectType AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetObjectType(
    absl::string_view name) const {
  const auto it = absl::c_find_if(
      object_info_, [name](const auto& kv) { return kv.second.name == name; });
  if (it == object_info_.end()) {
    return ObjectType::kInvalid;
  }
  return ToObjectType(it->second);
}

template <typename Scalar, typename AllocatorTraits>
ObjectType AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetObjectType(
    ObjectId id) const {
  const auto it = object_info_.find(id);

  if (it == object_info_.end()) {
    return ObjectType::kInvalid;
  }
  return ToObjectType(it->second);
}

template <typename Scalar, typename AllocatorTraits>
ObjectType AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetObjectType(
    ObjectIdSet set) const {
  const auto it = absl::c_find_if(object_info_, [&set](const auto& kv) {
    return ObjectIdSet(kv.first) == set;
  });
  if (it == object_info_.end()) {
    return ObjectType::kInvalid;
  }
  return ToObjectType(it->second);
}

template <typename Scalar, typename AllocatorTraits>
ObjectId AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetObjectId(
    absl::string_view name) const {
  const auto it = absl::c_find_if(
      object_info_, [name](const auto& kv) { return kv.second.name == name; });
  if (it == object_info_.end()) {
    return ObjectIdAssigner::kInvalidId;
  }
  return it->first;
}

template <typename Scalar, typename AllocatorTraits>
int AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetObjectIndexForName(
    absl::string_view name) const {
  const auto it = absl::c_find_if(
      object_info_, [name](const auto& kv) { return kv.second.name == name; });
  if (it == object_info_.end()) {
    return -1;
  }
  return it->second.object_index;
}

template <typename Scalar, typename AllocatorTraits>
int AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetObjectIndexForId(
    ObjectId id) const {
  const auto it = object_info_.find(id);
  if (it == object_info_.end()) {
    return -1;
  }
  return it->second.object_index;
}

template <typename Scalar, typename AllocatorTraits>
absl::string_view
AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetAssemblyName(
    AssemblyId id) const {
  if (id != assembly_info_.id) {
    return absl::string_view();
  }
  return assembly_info_.name;
}

template <typename Scalar, typename AllocatorTraits>
AssemblyId AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetAssemblyId(
    absl::string_view name) const {
  if (name != assembly_info_.name) {
    return AssemblyIdAssigner::kInvalidId;
  }
  return assembly_info_.id;
}

template <typename Scalar, typename AllocatorTraits>
std::string AssemblyCollisionChecker<
    Scalar, AllocatorTraits>::GetFilterDebugString() const {
  std::string debug_string;
  for (auto& [id, info] : object_info_) {
    absl::StrAppend(
        &debug_string,
        absl::StrFormat("name:      %s\n"
                        "id:        %s\n"
                        "inclusion: %s\n",
                        info.name, ObjectIdSet(id).DebugString(),
                        info.object.flags.inclusion_set.DebugString()));
    absl::StrAppend(&debug_string, "\n");
  }
  return debug_string;
}

template <typename Scalar, typename AllocatorTraits>
void AssemblyCollisionChecker<Scalar, AllocatorTraits>::RemoveAllAttachments() {
  for (auto it = object_info_.begin(); it != object_info_.end();) {
    const ObjectInfo& info = it->second;
    if (info.kinematics_type != ObjectKinematicsType::kRelative) {
      ++it;
      continue;
    }
    if (info.reference_object_id == kInvalidObjectId) {
      // Not an attached object.
      ++it;
      continue;
    }

    object_ids_assigner_.ReleaseId(it->first);
    it = object_info_.erase(it);
  }
  FinalizeGeometryChange();
  FinalizeModelChange();
}

template <typename Scalar, typename AllocatorTraits>
void AssemblyCollisionChecker<Scalar,
                              AllocatorTraits>::RemoveAllStaticObjects() {
  for (auto it = object_info_.begin(); it != object_info_.end();) {
    const ObjectInfo& info = it->second;
    if (info.kinematics_type != ObjectKinematicsType::kRelative) {
      ++it;
      continue;
    }
    if (info.reference_object_id != kInvalidObjectId) {
      // Not a static object.
      ++it;
      continue;
    }

    object_ids_assigner_.ReleaseId(it->first);
    it = object_info_.erase(it);
  }
  FinalizeGeometryChange();
  FinalizeModelChange();
}

template <typename Scalar, typename AllocatorTraits>
Status AssemblyCollisionChecker<Scalar, AllocatorTraits>::ComputeCollisions(
    const VoxelMapObject& obstacles,
    AssemblyCoordinateView<const Scalar> coordinates,
    const QueryOptions& options, Scratch& scratch,
    CollisionResult& result) const {
  if (coordinates.joint_positions.size() != assembly_info_.assembly_dof_count) {
    return Status(absl::StatusCode::kInvalidArgument,
                  Status::kInputJointVectorSizeWrong);
  }

  if (scratch.transformed_objects.objects.size() !=
          objects_zero_.objects.size() ||
      scratch.odom_translation_link.size() != objects_zero_.objects.size() ||
      scratch.odom_rotation_link.size() != objects_zero_.objects.size()) {
    return Status(absl::StatusCode::kInvalidArgument,
                  Status::kScratchSizeWrong);
  }
  for (int i = 0; i < scratch.transformed_objects.objects.size(); ++i) {
    if (scratch.transformed_objects.objects[i].spheres.size() !=
        objects_zero_.objects[i].spheres.size()) {
      return Status(absl::StatusCode::kInvalidArgument,
                    Status::kScratchSizeWrong);
    }
    if (scratch.transformed_objects.objects[i].capsules.size() !=
        objects_zero_.objects[i].capsules.size()) {
      return Status(absl::StatusCode::kInvalidArgument,
                    Status::kScratchSizeWrong);
    }
    if (scratch.transformed_objects.objects[i].boxes.size() !=
        objects_zero_.objects[i].boxes.size()) {
      return Status(absl::StatusCode::kInvalidArgument,
                    Status::kScratchSizeWrong);
    }
  }

  if (result.GetObjectCount() != objects_zero_.objects.size()) {
    return Status(absl::StatusCode::kInvalidArgument, Status::kResultSizeWrong);
  }

  ComputePoses(coordinates, scratch.View());

  TransformObjects(scratch);

  return ::collision_checking::ComputeCollisions(
      obstacles, scratch.transformed_objects, options, result);
}

template <typename Scalar, typename AllocatorTraits>
CC_INLINE void
AssemblyCollisionChecker<Scalar, AllocatorTraits>::ComputePoses(
    AssemblyCoordinateView<const Scalar> coordinates,
    AssemblyPosesView<Scalar> poses) const {
  // Compute or copy poses for assembly links and relative objects.
  if (const auto& [pos, size] = object_index_info_.assembly_objects; size > 0) {
    collision_checking::ComputePoses(assembly_info_.assembly_kinematics,
                                     coordinates, poses.SubView(pos, size));
  }

  for (int i = 0; i < relative_object_pose_info_.size(); ++i) {
    const int pose_index = object_index_info_.relative_objects.pos + i;
    Vector3<Scalar>& odom_translation_object =
        poses.odom_translation_link[pose_index];
    Matrix3<Scalar>& odom_rotation_object =
        poses.odom_rotation_link[pose_index];
    const Pose3<Scalar>& reference_pose_object =
        relative_object_pose_info_[i].reference_pose_object;
    const int reference_object_index =
        relative_object_pose_info_[i].reference_object_result_index;

    if (reference_object_index == -1) {
      odom_translation_object = reference_pose_object.translation();
      odom_rotation_object = reference_pose_object.rotationMatrix();
    } else {
      const Vector3<Scalar>& odom_translation_reference =
          poses.odom_translation_link[reference_object_index];
      const Matrix3<Scalar>& odom_rotation_reference =
          poses.odom_rotation_link[reference_object_index];
      odom_translation_object =
          odom_translation_reference +
          odom_rotation_reference * reference_pose_object.translation();
      odom_rotation_object =
          odom_rotation_reference * reference_pose_object.rotationMatrix();
    }
  }
}

template <typename Scalar, typename AllocatorTraits>
void AssemblyCollisionChecker<Scalar, AllocatorTraits>::TransformObjects(
    Scratch& scratch) const {
  CC_CHECK_EQ(scratch.GetPoseCount(), objects_zero_.objects.size());
  CC_CHECK_EQ(scratch.GetObjectCount(), objects_zero_.objects.size());
  auto& transformed_objects = scratch.transformed_objects.objects;

  // Compute object rotated and translated object geometry.
  const int num_objects = scratch.GetObjectCount();
  for (int result_idx = 0; result_idx < num_objects; ++result_idx) {
    const auto& object_zero = objects_zero_.objects[result_idx];
    auto& object = transformed_objects[result_idx];
    // Skip objects not used in any querries.
    if (object_zero.flags.inclusion_set.Empty()) {
      continue;
    }
    object.flags = object_zero.flags;
    object.AssignTransformedShape(scratch.odom_translation_link[result_idx],
                                  scratch.odom_rotation_link[result_idx],
                                  object_zero);
    SetEmptySize(&object.aabb);
  }

  UpdateAlignedBoxes(aabb_padding_, scratch.transformed_objects);
}

template <typename Scalar, typename AllocatorTraits>
Status
AssemblyCollisionChecker<Scalar, AllocatorTraits>::MaskConnectedObjects() {
  for (auto [_, object] : object_info_) {
    ObjectIdSet reachable_set;
    for (int connected_index :
         assembly_info_.link_connectivity.GetConnectedLinkIndices(
             object.assembly_link_index)) {
      auto it = absl::c_find_if(
          object_info_, [connected_index](const auto& search_it) {
            return search_it.second.assembly_link_index == connected_index;
          });
      CC_CHECK_NE(
          it, object_info_.end(),
          "Assembly index from connectivity (%d) not found in object_ids_.",
          connected_index);
      reachable_set.Insert(it->second.object_id);
    }
    const auto status =
        RemoveFromObjectInclusionSet(object.object_id, reachable_set);
    if (!status.ok()) {
      return status;
    }
  }

  return OkStatus();
}

template <typename Scalar, typename AllocatorTraits>
Status AssemblyCollisionChecker<Scalar, AllocatorTraits>::DisableCollisionPair(
    absl::string_view object_a, absl::string_view object_b) {
  const auto object_a_it = absl::c_find_if(
      object_info_,
      [object_a](const auto& kv) { return kv.second.name == object_a; });
  if (object_a_it == object_info_.end()) {
    return Status(absl::StatusCode::kInvalidArgument, Status::kUnknownObject);
  }
  const auto object_b_it = absl::c_find_if(
      object_info_,
      [object_b](const auto& kv) { return kv.second.name == object_b; });
  if (object_b_it == object_info_.end()) {
    return Status(absl::StatusCode::kInvalidArgument, Status::kUnknownObject);
  }

  objects_zero_.objects[object_a_it->second.object_index]
      .flags.inclusion_set.Remove(object_b_it->first);
  objects_zero_.objects[object_b_it->second.object_index]
      .flags.inclusion_set.Remove(object_a_it->first);

  object_a_it->second.object.flags.inclusion_set.Remove(object_b_it->first);
  object_b_it->second.object.flags.inclusion_set.Remove(object_a_it->first);

  FinalizeModelChange();

  return OkStatus();
}

template <typename Scalar, typename AllocatorTraits>
Status
AssemblyCollisionChecker<Scalar, AllocatorTraits>::AddToObjectInclusionSet(
    ObjectId id, ObjectIdSet set_to_add) {
  const auto id_it = object_info_.find(id);
  if (id_it == object_info_.end()) {
    return Status(absl::StatusCode::kInvalidArgument, Status::kUnknownObject);
  }
  // Add `set_to_add` to `id`'s inclusion set.
  objects_zero_.objects[id_it->second.object_index].flags.inclusion_set.Insert(
      set_to_add);
  id_it->second.object.flags.inclusion_set.Insert(set_to_add);

  // Add `id` to all ids in `set_to_add`.
  for (ObjectId other_id = ObjectId(0); other_id <= kMaxValidObjectId;
       ++other_id) {
    if ((other_id == id) || !set_to_add.Contains(other_id)) {
      continue;
    }
    if (auto other_it = object_info_.find(other_id);
        other_it != object_info_.end()) {
      objects_zero_.objects[other_it->second.object_index]
          .flags.inclusion_set.Insert(id);
      other_it->second.object.flags.inclusion_set.Insert(id);
    }
  }
  FinalizeModelChange();
  return OkStatus();
}

template <typename Scalar, typename AllocatorTraits>
Status
AssemblyCollisionChecker<Scalar, AllocatorTraits>::RemoveFromObjectInclusionSet(
    ObjectId id, ObjectIdSet set_to_remove) {
  const auto id_it = object_info_.find(id);
  if (id_it == object_info_.end()) {
    return Status(absl::StatusCode::kInvalidArgument, Status::kUnknownObject);
  }
  // Remove `set_to_remove` from `id`'s inclusion set.
  objects_zero_.objects[id_it->second.object_index].flags.inclusion_set.Remove(
      set_to_remove);
  id_it->second.object.flags.inclusion_set.Remove(set_to_remove);

  // For all ids in `set_to_remove`, remove `id` from the inclusion set.
  for (ObjectId other_id = ObjectId(0); other_id <= kMaxValidObjectId;
       ++other_id) {
    if ((other_id == id) || !set_to_remove.Contains(other_id)) {
      continue;
    }
    if (auto other_it = object_info_.find(other_id);
        other_it != object_info_.end()) {
      objects_zero_.objects[other_it->second.object_index]
          .flags.inclusion_set.Remove(id);
      other_it->second.object.flags.inclusion_set.Remove(id);
    }
  }
  FinalizeModelChange();
  return OkStatus();
}

template <typename Scalar, typename AllocatorTraits>
Status AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetObjectFlags(
    const ObjectId& object_id, ObjectFlags& flag) const {
  if (const auto it = object_info_.find(object_id); it != object_info_.end()) {
    flag = it->second.object.flags;
    return OkStatus();
  }
  return Status(absl::StatusCode::kInvalidArgument, Status::kUnknownObject);
}

template <typename Scalar, typename AllocatorTraits>
std::vector<ObjectFlags>
AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetAllObjectFlags() const {
  std::vector<ObjectFlags> flags;
  flags.reserve(object_info_.size());
  for (const auto& [id, info] : object_info_) {
    flags.push_back(info.object.flags);
  }
  return flags;
}

template <typename Scalar, typename AllocatorTraits>
Status AssemblyCollisionChecker<Scalar, AllocatorTraits>::SetObjectFlags(
    const ObjectFlags& flags) {
  ObjectId id;
  if (!GetObjectIdFromSet(flags.id_set, id)) {
    return Status(absl::StatusCode::kInvalidArgument,
                  Status::kInvalidObjectIdSet);
  }
  auto it = object_info_.find(id);
  if (it == object_info_.end()) {
    return Status(absl::StatusCode::kInvalidArgument, Status::kUnknownObject);
  }
  it->second.object.flags = flags;
  objects_zero_.objects[it->second.object_index].flags.inclusion_set =
      flags.inclusion_set;
  FinalizeModelChange();
  return OkStatus();
}
template <typename Scalar, typename AllocatorTraits>
Status AssemblyCollisionChecker<Scalar, AllocatorTraits>::SetObjectFlags(
    absl::Span<const ObjectFlags> flags) {
  for (const auto& flag : flags) {
    const auto status = SetObjectFlags(flag);
    if (!status.ok()) return status;
  }
  return OkStatus();
}

template <typename Scalar, typename AllocatorTraits>
std::vector<std::string>
AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetObjectNamesFromSet(
    ObjectIdSet set) const {
  std::vector<std::string> names;
  ObjectIdSet remaining_ids = set;
  // There aren't a lot of objects, so just loop over all of them.
  for (const auto& [id, info] : object_info_) {
    if (set.Contains(id)) {
      remaining_ids.Remove(id);
      names.push_back(info.name);
    }
  }
  // == consider mapping voxel map sub ids.
  if (set.Overlaps(kVoxelMapIdSet)) {
    names.push_back(std::string(kVoxelMapObjectName));
    remaining_ids.Remove(kVoxelMapIdSet);
  }
  if (!remaining_ids.Empty()) {
    names.push_back(absl::StrFormat("_ObjectIdSet_%s_is_unknown",
                                    remaining_ids.DebugString()));
  }
  return names;
}

template <typename Scalar, typename AllocatorTraits>
Status
AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetObjectSetFromObjectNames(
    absl::Span<const std::string> names, ObjectIdSet& set) const {
  set.Clear();
  for (const auto& name : names) {
    const auto it = absl::c_find_if(object_info_, [&name](const auto& kv) {
      return kv.second.name == name;
    });
    if (it == object_info_.end()) {
      return Status(absl::StatusCode::kInvalidArgument,
                    Status::kInvalidObjectName);
    }
    set.Insert(it->first);
  }
  return OkStatus();
}

template <typename Scalar, typename AllocatorTraits>
absl::StatusOr<CollisionStateProto>
AssemblyCollisionChecker<Scalar, AllocatorTraits>::GetCollisionStateProto(
    const VoxelMapObject& obstacles, const Scratch& scratch,
    const CollisionResult& result) const {
  return MakeCollisionStateProto(
      absl::bind_front(
          &AssemblyCollisionChecker<Scalar,
                                    AllocatorTraits>::GetObjectNameFromIdSet,
          this),
      obstacles, scratch.transformed_objects, result);
}

}  // namespace collision_checking
#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_COLLISION_CHECKER_H_
