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

#include "collision_checking/options.h"

#include <string>

#include "collision_checking/logging.h"
#include "absl/strings/str_format.h"

namespace collision_checking {

std::string QueryOptions::DebugString() const { return ToString(*this); }

absl::string_view ToString(const QueryOptions::Type type) {
  switch (type) {
    case QueryOptions::Type::kInvalid:
      return "kInvalid";
    case QueryOptions::Type::kIsCollisionFree:
      return "kIsCollisionFree";
    case QueryOptions::Type::kComputeObjectDistances:
      return "kComputeObjectDistances";
    case QueryOptions::Type::kComputeContactPoints:
      return "kComputeContactPoints";
  }
  CC_PANIC("Invalid enum value '%d'.", static_cast<int>(type));
}

std::string ToString(const QueryOptions& options) {
  return absl::StrFormat("type: %s; exclusion_set: %s; batch_early_exit_ok: %d",
                         ToString(options.GetType()),
                         options.GetExclusionSet().DebugString(),
                         options.GetBatchEarlyExitOk());
}

}  // namespace collision_checking
