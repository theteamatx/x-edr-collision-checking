#include "collision_checking/options.h"

#include <string>

#include "collision_checking/logging.h"
#include "third_party/absl/strings/str_format.h"

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
