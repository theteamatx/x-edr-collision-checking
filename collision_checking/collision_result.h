#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_COLLISION_RESULT_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_COLLISION_RESULT_H_

#include <limits>

#include "experimental/users/buschmann/collision_checking/eigenmath.h"
#include "experimental/users/buschmann/collision_checking/logging.h"
#include "experimental/users/buschmann/collision_checking/options.h"
#include "experimental/users/buschmann/collision_checking/vector.h"

// Basic classes and functions for performing collision checking between
// a set of composite geometric shapes and a static environment.

namespace collision_checking {

// CollisionResult holds the result data for a call to ComputeCollisions.
// Scalar must be a floating point type.
// Construct this with a matching 'count' argument or call SetObjectCount() to
// match the number of CompositeObjects for a collision query.
template <typename Scalar, typename AllocatorTraits = DefaultAllocatorTraits>
class CollisionResult : public ParametrizedNewDelete<AllocatorTraits> {
 public:
  // Collection of data related to a pair of contact points.
  // Vectors are expressed in the same bases as the object lists passed into
  // ComputeCollisions, this is commonly the root frame of a collision world.
  struct ContactPointInfo {
    // Contact point on `this` object.
    Vector3<Scalar> contact_point_this;
    // Contact point corresponding to `contact_point_this` on the other body.
    Vector3<Scalar> contact_point_other;
    // Contact normal pointing away from `this` body toward the `other` body.
    Vector3<Scalar> normal_this_other;
    // Id set of the contact partner.
    ObjectIdSet other_id_set;
  };

  // A description of which part of the obstacle/voxel map has the minimum
  // distance.
  struct MinimumObstacleInfo {
    // Center of the geometry element (currently: the sphere) with the minimum
    // distance.
    Vector3<Scalar> center;
    // True if there is a minimum distance to an obstacle (i.e., `center` is
    // valid).
    bool has_minimum_distance = false;
    // The id of the
    int index = -1;
  };

  CollisionResult() = default;
  explicit CollisionResult(int count, const QueryOptions& options)
      : query_options_(options) {
    static_assert(std::is_floating_point_v<Scalar>,
                  "Scalar must be a floating point type.");
    Allocate(count);
  }

  // Allocates data for `count` obstacles and queries according to `options`
  // set in the constructor.
  void Allocate(int count) {
    object_count_ = count;
    object_hits_.resize(count);
    object_id_sets_.resize(count);
    min_distances_.resize(count);
    if (query_options_.GetType() == QueryOptions::kComputeContactPoints) {
      contact_point_info_.resize(count);
    } else {
      contact_point_info_.resize(0);
    }
    Reset();
  }
  // Returns true if memory for a collision check with `options` has been
  // allocated.
  CC_INLINE
  bool MemoryIsAllocatedFor(const QueryOptions& options) const {
    if (options.GetType() == QueryOptions::kComputeContactPoints) {
      return query_options_.GetType() == QueryOptions::kComputeContactPoints;
    }
    return true;
  }

  // Returns the number of obstacles.
  CC_INLINE
  int GetObjectCount() const { return object_count_; }

  CC_INLINE void Reset() {
    min_distance_ = std::numeric_limits<Scalar>::infinity();
    has_collisions_ = false;
    for (auto& hits : object_hits_) {
      hits.Clear();
    }
    for (auto& set : object_id_sets_) {
      set.Clear();
    }
    for (auto& distance : min_distances_) {
      distance = std::numeric_limits<Scalar>::infinity();
    }
    for (auto& info : contact_point_info_) {
      info = {
          Vector3<Scalar>::Constant(std::numeric_limits<Scalar>::infinity()),
          Vector3<Scalar>::Constant(std::numeric_limits<Scalar>::infinity()),
          Vector3<Scalar>::Zero(), ObjectIdSet::kEmpty()};
    }

    min_obstacle_info_.index = -1;
    min_obstacle_info_.has_minimum_distance = false;
  }

  // True if any link collides.
  CC_INLINE bool GetHasCollisions() const { return has_collisions_; }

  // Sets has_collision to true.
  CC_INLINE void SetHasCollisions(bool value) {
    has_collisions_ = value;
  }

  // Returns the bitmask of colliding objects for object_index.
  // The object at object_index corresponds to the one in MovingObjects
  // used to compute the result.
  // Returns kInvalidObjectId if the value has not been computed.
  CC_INLINE ObjectIdSet GetObjectHits(int object_index) const {
    return object_hits_[object_index];
  }
  CC_INLINE ObjectIdSet& GetObjectHits(int object_index) {
    return object_hits_[object_index];
  }

  // Returns an ObjectIdSet containing only the object's id.
  CC_INLINE ObjectIdSet GetObjectIdSet(int object_index) const {
    return object_id_sets_[object_index];
  }
  CC_INLINE ObjectIdSet& GetObjectIdSet(int object_index) {
    return object_id_sets_[object_index];
  }

  // Returns the minimum distance of all computed distances.
  CC_INLINE Scalar GetMinimumDistance() const { return min_distance_; }
  // Sets the minimum distance.
  CC_INLINE void SetMinimumDistance(Scalar distance) {
    min_distance_ = distance;
  }

  // Returns the minimum distance for object_index.
  // The object at object_index corresponds to the one in MovingObjects
  // used to compute the result.
  CC_INLINE Scalar GetMinimumDistance(int object_index) const {
    return min_distances_[object_index];
  }
  // Returns the minimum distance for object_index.
  CC_INLINE Scalar& GetMinimumDistance(int object_index) {
    return min_distances_[object_index];
  }

  // Returns the ContactPointInfo struct for the given object_index.
  // The object at object_index corresponds to the one in MovingObjects
  // used to compute the result.
  CC_INLINE const ContactPointInfo& GetContactPointInfo(
      int object_index) const {
    CC_CHECK_LT(object_index, contact_point_info_.size());
    return contact_point_info_[object_index];
  }

  // Returns the ContactPointInfo struct for the given object_index.
  // The object at object_index corresponds to the one in MovingObjects
  // used to compute the result.
  CC_INLINE ContactPointInfo& GetContactPointInfo(int object_index) {
    CC_CHECK_LT(object_index, contact_point_info_.size());
    return contact_point_info_[object_index];
  }

  // Returns information on which obstacle geometry has the minimum distance
  // to a robot geometry.
  // If the minimum collision distance is not with the environment,
  // GetMinimumObstacleInfo().has_minimum_distance == false.
  // Only set if minimum distances are computed.
  CC_INLINE const MinimumObstacleInfo& GetMinimumObstacleInfo() const {
    return min_obstacle_info_;
  }
  CC_INLINE void SetMinimumObstacleInfo(const Vector3<Scalar>& center,
                                        const int index) {
    min_obstacle_info_.index = index;
    min_obstacle_info_.center = center;
    min_obstacle_info_.has_minimum_distance = true;
  }

 private:
  QueryOptions query_options_;
  std::size_t object_count_ = 0;
  bool has_collisions_ = false;
  Vector<ObjectIdSet, AllocatorTraits> object_hits_;
  Vector<ObjectIdSet, AllocatorTraits> object_id_sets_;
  Vector<Scalar, AllocatorTraits> min_distances_;
  Scalar min_distance_ = std::numeric_limits<Scalar>::infinity();
  Vector<ContactPointInfo, AllocatorTraits> contact_point_info_;
  MinimumObstacleInfo min_obstacle_info_;
};

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_COLLISION_RESULT_H_
