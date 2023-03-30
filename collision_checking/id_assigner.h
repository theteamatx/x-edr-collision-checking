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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ID_ASSIGNER_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ID_ASSIGNER_H_

#include "absl/container/flat_hash_set.h"
#include "collision_checking/assembly_id.h"
#include "collision_checking/logging.h"
#include "collision_checking/object_id.h"

namespace collision_checking {

// Generates and manages unique ids.
// Not thread safe.
template <typename Traits>
class IdAssigner {
 public:
  using IdType = typename Traits::IdType;
  static constexpr IdType kInvalidId = Traits::kInvalidId;
  static constexpr IdType kMinValidId = Traits::kMinValidId;
  static constexpr IdType kMaxValidId = Traits::kMaxValidId;
  static constexpr size_t kNumberOfIds =
      static_cast<size_t>(kMaxValidId - kMinValidId + IdType{1});

  IdAssigner() = default;

  // Resturns a new unique IdType, or kInvalidId if there are no remaining
  // free ids.
  IdType GetFreeId();

  // Releases `id` and makes it available for subsequent calls to GetFreeId().
  void ReleaseId(IdType id);

  size_t NumFreeIds() const;

 private:
  IdType Step(IdType id) const;
  IdType next_id_ = kMinValidId;
  absl::flat_hash_set<IdType> assigned_ids_;
};

// Assigner for ObjectIds.
struct ObjectIdAssignerTraits {
  using IdType = ObjectId;
  static constexpr IdType kInvalidId = kInvalidObjectId;
  static constexpr IdType kMinValidId = IdType{0};
  static constexpr IdType kMaxValidId = kMaxValidObjectIdForAssignment;
};

using ObjectIdAssigner = IdAssigner<ObjectIdAssignerTraits>;

// Assigner for AssemblyIds.
struct AssemblyIdAssignerTraits {
  using IdType = AssemblyId;
  static constexpr IdType kInvalidId = AssemblyId::Max();
  static constexpr IdType kMinValidId = IdType(0);
  static constexpr IdType kMaxValidId = IdType(AssemblyId::Max() - IdType(1));
};

using AssemblyIdAssigner = IdAssigner<AssemblyIdAssignerTraits>;

////////////////////////////////////////////////////////////////////////////////
// Implementation details only below here.
template <typename Traits>
void IdAssigner<Traits>::ReleaseId(IdType id) {
  const size_t erased = assigned_ids_.erase(id);
  CC_CHECK_NE(erased, 0, "Trying to release id that hasn't been assigned.");
}

template <typename Traits>
size_t IdAssigner<Traits>::NumFreeIds() const {
  return kNumberOfIds - assigned_ids_.size();
}

template <typename Traits>
typename IdAssigner<Traits>::IdType IdAssigner<Traits>::Step(
    const IdType id) const {
  if (id == kMaxValidId) {
    return kMinValidId;
  }
  return id + IdType{1};
}

template <typename Traits>
typename IdAssigner<Traits>::IdType IdAssigner<Traits>::GetFreeId() {
  if (NumFreeIds() == 0) {
    return kInvalidId;
  }

  const IdType id = next_id_;
  assigned_ids_.insert(id);

  // Find a free id to use next.
  if (NumFreeIds() == 0) {
    return id;
  }

  for (; assigned_ids_.contains(next_id_); next_id_ = Step(next_id_)) {
  }

  return id;
}

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ID_ASSIGNER_H_
