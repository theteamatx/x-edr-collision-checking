#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_MODEL_INTERFACE_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_MODEL_INTERFACE_H_
#include <string>
#include <vector>

#include "experimental/users/buschmann/collision_checking/assembly/assembly.h"
#include "experimental/users/buschmann/collision_checking/assembly_id.h"
#include "experimental/users/buschmann/collision_checking/model_options.h"
#include "experimental/users/buschmann/collision_checking/object_id.h"
#include "experimental/users/buschmann/collision_checking/status.h"
#include "third_party/absl/status/statusor.h"

namespace collision_checking {

// Types of collision objects.
enum class ObjectType {
  // An invalid type.
  kInvalid,
  // The object is part of an assembly.
  kAssembly,
  // The object is defined relative to an assembly object.
  kAttachment,
  // The object is static, with a fixed pose.
  kStatic
};

// A template for pure virtual base classes that defines the interface for
// building and modifying collision models, as well as querying information
// about them.
// Templated on the scalar type to allow generation of single and double
// precision versions. The virtual function interface is used by
// BatchCheckerInterface to provide dynamic dispatch to GPU or CPU
// implementations of single precision batch collision queries.
template <typename Scalar>
class ModelInterface {
 public:
  using Options = ModelOptions<Scalar>;

  virtual ~ModelInterface() = default;
  // Returns a default Options class.
  static Options DefaultOptions() { return Options(); }

  // Adds `assembly` to the collision model with `options`.
  // The `joint_order` specifies the mapping between joint position vector
  // elements passed to members of this class and the names of the joints as
  // specified in assembly.
  // Returns the id for the assembly, or a Status if an error occurred.
  // Currently, only one assembly can be added.
  // TODO(b/208409848): Add support for multiple assemblies.
  virtual absl::StatusOr<AssemblyId> AddAssembly(
      const Assembly& assembly, absl::Span<const std::string> joint_order,
      const Options& options) = 0;

  // Remove assembly with `id` from the collision model.
  virtual absl::Status RemoveAssembly(AssemblyId id) = 0;

  // Adds a static object composed of `shapes` at pose `world_pose_object` with
  // `padding`.
  // This is equivalent to `AddRelativeObject` with `reference_object_id ==
  // ObjectFlags::kInvalid`.
  // Returns the object's id, or a status on error.
  virtual absl::StatusOr<ObjectId> AddStaticObject(
      absl::string_view name, const Pose3<Scalar>& world_pose_object,
      absl::Span<const geometry_shapes::ShapeBase* const> shapes,
      Scalar padding) = 0;

  // Adds an object composed of `shapes` with `padding`, whose pose is
  // `reference_pose_object` relative to `reference_object_id`.
  // Use `reference_object_id == ObjectFlags::kInvalid` for a static pose that
  // is not relative to any other object. In that case, `reference_pose_object`
  // specifies the object's pose relative to the common "world" frame.
  // If `reference_object_id != ObjectFlags::kInvalid`, it must refer to an
  // assembly object.
  // If the reference object is an assembly object, collisions between the two
  // are disabled.
  // If the object is static, collisions between it and the voxel map and other
  // static objects are disabled.
  // Returns the object's id, or a status on error.
  virtual absl::StatusOr<ObjectId> AddRelativeObject(
      absl::string_view name, ObjectId reference_object_id,
      const Pose3<Scalar>& reference_pose_object,
      absl::Span<const geometry_shapes::ShapeBase* const> shapes,
      Scalar padding) = 0;

  // Removes the static object with `id`. Returns a non-ok status on error.
  virtual absl::Status RemoveRelativeObject(ObjectId id) = 0;

  // Set the pose of a static object to `odom_pose_object`.
  // This replaces the value set in AddStaticObject.
  // Returns a non-ok status on error.
  virtual absl::Status SetRelativeObjectPose(
      ObjectId id, const Pose3<Scalar>& reference_pose_object) = 0;

  // Sets the padding for AABB updates.
  virtual void SetAABBPadding(Scalar padding) = 0;

  // Returns the padding for AABB updates.
  virtual Scalar GetAABBPadding() const = 0;

  // Returns true if an assembly with `id` is part of the collision model.
  virtual bool IsValidAssembly(AssemblyId id) const = 0;

  // Returns true if an object with `id` is part of the collision model.
  virtual bool IsValidObject(ObjectId id) const = 0;

  // Returns the name of the object with the given `id`.
  // If the id is invalid, an empty string is returned.
  virtual absl::string_view GetObjectName(ObjectId id) const = 0;

  // Returns the name of the object with the given id `set`.
  // If the set is invalid, an empty string is returned.
  virtual absl::string_view GetObjectNameFromIdSet(ObjectIdSet set) const = 0;

  // Returns the ObjectType, or kInvalid if it isn't known.
  virtual ObjectType GetObjectType(absl::string_view name) const = 0;
  // Returns the ObjectType, or kInvalid if it isn't known.
  virtual ObjectType GetObjectType(ObjectId id) const = 0;
  // Returns the ObjectType, or kInvalid if it isn't known.
  virtual ObjectType GetObjectType(ObjectIdSet set) const = 0;

  // Returns the ObjectId for the given name, or kInvalidObjectId
  // if name is invalid.
  virtual ObjectId GetObjectId(absl::string_view name) const = 0;

  // Returns the index for the object with `name` as used in CollisionResult, or
  // -1 if name is invalid.
  virtual int GetObjectIndexForName(absl::string_view name) const = 0;

  // Returns the index for the object with `id` as used in CollisionResult, or
  // -1 if the index is invalid.
  virtual int GetObjectIndexForId(ObjectId id) const = 0;

  // Returns the name of the assembly with the given `id`.
  // If the id is invalid, an empty string is returned.
  virtual absl::string_view GetAssemblyName(AssemblyId id) const = 0;

  // Returns the AssemblyId for the given name, or
  // AssemblyIdAssigner::kInvalidId if name is invalid.
  virtual AssemblyId GetAssemblyId(absl::string_view name) const = 0;

  // Returns a vector of object names corresponding to `set`.
  virtual std::vector<std::string> GetObjectNamesFromSet(
      ObjectIdSet set) const = 0;

  // Populates `set` corresponding to a list of object `names`.
  // Returns a non-ok Status if an error occurred.
  // The return value will contain kInvalidObjectId iff any of the names
  // is not valid.
  virtual Status GetObjectSetFromObjectNames(
      absl::Span<const std::string> names, ObjectIdSet& set) const = 0;

  // Mask collisions between connected links.
  virtual Status MaskConnectedObjects() = 0;

  // Disable collisions between the two specified objects.
  virtual Status DisableCollisionPair(absl::string_view object_a,
                                      absl::string_view object_b) = 0;

  // Add `set_to_add` to the inclusion mask of the object with `id`,
  // and add `id` to all inclusion sets of objects in `set_to_add`.
  virtual Status AddToObjectInclusionSet(ObjectId id,
                                         ObjectIdSet set_to_add) = 0;
  virtual Status AddToObjectInclusionSet(ObjectId id, ObjectId id_to_add) = 0;
  // Remove `set_to_remove` from the inclusion mask of the object with `id`,
  // as well as `id` from all elements of `set_to_remove`.
  virtual Status RemoveFromObjectInclusionSet(ObjectId id,
                                              ObjectIdSet set_to_remove) = 0;
  virtual Status RemoveFromObjectInclusionSet(ObjectId id,
                                              ObjectId id_to_remove) = 0;

  // Sets `object_flags` to the ObjectFlags for `object_id`.
  // Returns a non-ok status if an error occurred.
  // Use this to save and later restore object filter settings (inclusion
  // sets).
  virtual Status GetObjectFlags(const ObjectId& object_id,
                                ObjectFlags& flags) const = 0;
  // Returns a vector of all object flags.
  virtual std::vector<ObjectFlags> GetAllObjectFlags() const = 0;

  // Sets filters / inclusion sets according to `flags`.
  // Returns a non-ok status if an error occurred (flags.id_set is unknown).
  virtual Status SetObjectFlags(const ObjectFlags& flag) = 0;

  // Calls SetObjectFlags on all `flags`.
  virtual Status SetObjectFlags(absl::Span<const ObjectFlags> flags) = 0;

  // Returns a human-readable description of the current filtering
  // configuration. Not real-time safe.
  virtual std::string GetFilterDebugString() const = 0;

  // Removes all relative / non-assembly objects that have a reference object.
  virtual void RemoveAllAttachments() = 0;

  // Removes all relative / non-assembly objects that don't have a reference
  // object.
  virtual void RemoveAllStaticObjects() = 0;
};
}  // namespace collision_checking
#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_MODEL_INTERFACE_H_
