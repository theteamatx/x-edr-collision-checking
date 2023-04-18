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

#ifndef COLLISION_CHECKING_OBJECT_ID_H_
#define COLLISION_CHECKING_OBJECT_ID_H_

#include <array>
#include <bitset>
#include <climits>
#include <cstdint>
#include <sstream>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "collision_checking/typed_id_int.h"

namespace collision_checking {

// An identifier for a collision object.
// At most kMaxValidObjectId + 1 distinct objects are supported.
using ObjectId = TypedIdInt<unsigned>;

// A set of ids.
// `SetType` is the integral type used to represent the set of ids of type
// `IdType` as a bitset.
class ObjectIdSet {
 public:
  using SetType = uint64_t;
  using IdType = ObjectId;
  // An invalid Id (one beyond what is representable in IdSet).
  static constexpr IdType kInvalidId = IdType{CHAR_BIT * sizeof(SetType)};
  // The largest valid Id.
  static constexpr IdType kMaxValidId = IdType{CHAR_BIT * sizeof(SetType) - 1};

  // The empty set.
  static constexpr ObjectIdSet kEmpty() { return ObjectIdSet(SetType{0}); }
  // The set containing all valid ObjectIds.
  static constexpr ObjectIdSet kAll() { return ~kEmpty(); }

  // Constructs an empty set.
  constexpr ObjectIdSet() = default;
  // Construct an IdSet corresponding to the bitset `set`.
  constexpr explicit ObjectIdSet(SetType set) : set_{set} {}
  // Constructs an IdSet containing only `id`.
  constexpr explicit ObjectIdSet(const IdType& id) {
    set_ = SetType{1} << id.value();
  }

  // Returns true if *this contains `id`.
  constexpr bool Contains(const IdType& id) const {
    return (set_ & (SetType{1} << id.value())) != 0x0;
  }
  // Returns true if *this and `other` have at least one ObjectId in common.
  constexpr bool Overlaps(const ObjectIdSet& other) const {
    return (set_ & other.set_) != 0x0;
  }
  // Returns true if *this is empty.
  constexpr bool Empty() const { return set_ == 0; }
  // Returns the number of ObjectIds in the set.
  constexpr int Count() const { return absl::popcount(set_); }
  // Adds `id` to *this.
  constexpr ObjectIdSet& Insert(const IdType& id) {
    *this |= ObjectIdSet(id);
    return *this;
  }
  // Adds all `ids` to *this.
  template <typename IdContainer>
  constexpr ObjectIdSet& Insert(const IdContainer& ids) {
    for (auto& id : ids) {
      Insert(id);
    }
    return *this;
  }

  // Adds `other` to *this.
  constexpr ObjectIdSet& Insert(const ObjectIdSet& other) {
    set_ |= other.set_;
    return *this;
  }
  // Removes `id` from *this.
  constexpr ObjectIdSet& Remove(const ObjectId& id) {
    Remove(ObjectIdSet(id));
    return *this;
  }
  // Removes `other` from *this.
  constexpr ObjectIdSet& Remove(const ObjectIdSet& other) {
    set_ &= ~other.set_;
    return *this;
  }
  // Removes all ids.
  constexpr ObjectIdSet& Clear() {
    set_ = SetType{0};
    return *this;
  }
  constexpr ObjectIdSet& operator&=(const ObjectIdSet& rhs) {
    set_ &= rhs.set_;
    return *this;
  }
  constexpr ObjectIdSet& operator|=(const ObjectIdSet& rhs) {
    set_ |= rhs.set_;
    return *this;
  }
  constexpr ObjectIdSet& operator^=(const ObjectIdSet& rhs) {
    set_ ^= rhs.set_;
    return *this;
  }
  constexpr ObjectIdSet operator~() const { return ObjectIdSet(~set_); }
  constexpr ObjectIdSet& operator=(const ObjectIdSet& other) = default;
  constexpr ObjectIdSet operator&(const ObjectIdSet& right) const {
    return ObjectIdSet(*this) &= right;
  }
  constexpr ObjectIdSet operator|(const ObjectIdSet& right) const {
    return ObjectIdSet(*this) |= right;
  }
  constexpr ObjectIdSet operator^(const ObjectIdSet& right) const {
    return ObjectIdSet(*this) ^= right;
  }
  constexpr bool operator==(const ObjectIdSet& other) const {
    return set_ == other.set_;
  }
  constexpr bool operator!=(const ObjectIdSet& other) const {
    return !(*this == other);
  }
  // Returns a string representation for debugging.
  std::string DebugString() const {
    std::stringstream ss;
    constexpr std::size_t kBitCount = sizeof(SetType) * CHAR_BIT;
    ss << std::bitset<kBitCount>(set_);
    return ss.str();
  }

  const SetType& AsSetType() const { return set_; }

 private:
  SetType set_{0};
};

// An invalid ObjectId.
inline constexpr ObjectId kInvalidObjectId = ObjectIdSet::kInvalidId;
// The largest valid ObjectId.
inline constexpr ObjectId kMaxValidObjectId = ObjectIdSet::kMaxValidId;

inline constexpr size_t kVoxelMapIdCount = 8;
inline constexpr std::array<ObjectId, kVoxelMapIdCount> kVoxelMapIds = {
    {kMaxValidObjectId - ObjectId{7}, kMaxValidObjectId - ObjectId{6},
     kMaxValidObjectId - ObjectId{5}, kMaxValidObjectId - ObjectId{4},
     kMaxValidObjectId - ObjectId{3}, kMaxValidObjectId - ObjectId{2},
     kMaxValidObjectId - ObjectId{1}, kMaxValidObjectId}};
inline constexpr ObjectId kMinVoxelMapId = kVoxelMapIds.front();
inline constexpr ObjectId kMaxVoxelMapId = kVoxelMapIds.back();
// The default id for voxels. This is used if no semantic information is
// available.
inline constexpr ObjectId kDefaultVoxelMapId = kMinVoxelMapId;

// The largest valid ObjectId vor non-environment objects.
inline constexpr ObjectId kMaxValidObjectIdForAssignment =
    kMinVoxelMapId - ObjectId{1};

// ObjectSet containing only the VoxelMapObject's id
inline constexpr ObjectIdSet kVoxelMapIdSet =
    ObjectIdSet().Insert(kVoxelMapIds);

// The maximum number of collision objects (excluding the VoxelMapObject).
inline constexpr size_t kMaxObjectCount = size_t{kMinVoxelMapId.value()};

// A lightweight structure for collision object identification and collision
// pair filtering.
struct ObjectFlags {
  // An ObjectIdSet containing one ObjectId.
  ObjectIdSet id_set = ObjectIdSet::kEmpty();
  // Set of ObjectIds to include in collision checks with the object identified
  // by `id_set`.
  ObjectIdSet inclusion_set = ObjectIdSet::kEmpty();
};

// Returns true if collisions between the object identified by object_a and
// object_b should be checked, false otherwise.
constexpr bool ShouldCheckPair(const ObjectFlags& object_a,
                               const ObjectFlags& object_b) {
  return object_a.inclusion_set.Overlaps(object_b.id_set) &&
         object_b.inclusion_set.Overlaps(object_a.id_set);
}

// Sets `id` to the single ObjectId in `id_set`.
// Returns true on success and false if an error occurred.
inline bool GetObjectIdFromSet(const ObjectIdSet& id_set, ObjectId& id) {
  if (!absl::has_single_bit(id_set.AsSetType())) {
    return false;
  }
  const auto bw = absl::bit_width(id_set.AsSetType() - 1);
  id = ObjectId(bw);
  return true;
}

// Returns the single ObjectId in ObjectidSet, or a status if an error occurred.
inline absl::StatusOr<ObjectId> GetObjectIdFromSet(const ObjectIdSet& id_set) {
  ObjectId id;
  if (!GetObjectIdFromSet(id_set, id)) {
    return absl::FailedPreconditionError("More than one bit is set.");
  }
  return id;
}

}  // namespace collision_checking
#endif  // COLLISION_CHECKING_OBJECT_ID_H_
