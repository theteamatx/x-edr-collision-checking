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

#ifndef COLLISION_CHECKING_OPTIONS_H_
#define COLLISION_CHECKING_OPTIONS_H_

#include <string>

#include "absl/strings/string_view.h"
#include "collision_checking/inlining.h"
#include "collision_checking/object_id.h"

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

#endif  // COLLISION_CHECKING_OPTIONS_H_
