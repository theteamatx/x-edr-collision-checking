#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_OPTIONS_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_OPTIONS_H_

#include <string>

#include "collision_checking/inlining.h"
#include "collision_checking/object_id.h"
#include "third_party/absl/strings/string_view.h"

namespace collision_checking {

// Options for specifying a collision query.
class QueryOptions {
 public:
  enum Type {
    // Invalid value.
    kInvalid = 0,
    // Only determine if a collision occurs or not.
    // Do not compute all collisions.
    kIsCollisionFree = 1,
    // Compute the minimum distance for each movable object.
    // Distances for objects outside each other's bounding boxes are not
    // computed.
    kComputeObjectDistances = 2,
    // Compute everything enabled in kComputeObjectDistances, plus the contact
    // points for the minimum distance.
    kComputeContactPoints = 3
  };

  CC_INLINE const Type& GetType() const { return type_; }
  QueryOptions& SetType(Type type) {
    type_ = type;
    return *this;
  }

  // Sets an additional exclusion mask. Any objects matching the mask are
  // excluded from collision checks.
  QueryOptions& SetExclusionSet(const ObjectIdSet& set) {
    exclusion_set_ = set;
    return *this;
  }

  CC_INLINE const ObjectIdSet& GetExclusionSet() const {
    return exclusion_set_;
  }

  // If true, batch queries may exit on the first negative or colliding sample.
  // Only relevant for batch queries.
  // Implementations may ignore this flag and compute results for all samples.
  QueryOptions& SetBatchEarlyExitOk(const bool ok) {
    batch_early_exit_ok_ = ok;
    return *this;
  }

  CC_INLINE const bool GetBatchEarlyExitOk() const {
    return batch_early_exit_ok_;
  }

  // Returns a debug string representation for *this.
  std::string DebugString() const;

 private:
  // The type of collision query to perform.
  // The default is to only check if any collision occurs or not.
  Type type_ = Type::kIsCollisionFree;
  // Bitmask of ids to exclude from the query.
  ObjectIdSet exclusion_set_;
  // If true, batch queries may exit on the first negative or colliding sample.
  bool batch_early_exit_ok_ = false;
};

// Returns a string representation of `type`.
absl::string_view ToString(QueryOptions::Type type);

// Returns a string representation of `options`.
std::string ToString(const QueryOptions& options);

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_OPTIONS_H_
