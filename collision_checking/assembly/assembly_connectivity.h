#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_ASSEMBLY_CONNECTIVITY_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_ASSEMBLY_CONNECTIVITY_H_

#include <string>
#include <vector>

#include "collision_checking/assembly/assembly.h"
#include "third_party/absl/container/flat_hash_map.h"
#include "third_party/absl/container/flat_hash_set.h"
#include "third_party/absl/strings/string_view.h"
#include "third_party/absl/types/span.h"

namespace collision_checking {

// A class to determine link connectivity in assemblies.
class AssemblyConnectivity {
 public:
  // Default constructor, creates an empty connectivity object.
  AssemblyConnectivity() = default;
  // Creates an AssemblyConnectivity class.
  // `max_movable_joints` is the maximum number of non-fixed joints that are
  // allowed between any two links in a set returned from
  // `GetConnectedLinkIndices`.
  explicit AssemblyConnectivity(const Assembly& assembly,
                                int max_movable_joints = 0);

  // Returns the indices of the link group connected to `link_index` according
  // to the construction parameter `max_movable_joints`. If the link index is
  // invalid, and empty span is returned.
  absl::Span<const int> GetConnectedLinkIndices(int link_index) const;

 private:
  void RecurseOutwardAddingLinks(const Link& link,
                                 absl::flat_hash_set<int>& connected_links,
                                 absl::flat_hash_set<int>& visited,
                                 int& remaining_movable_joints) const;

  absl::flat_hash_map<int, std::vector<int>> connected_links_;
};

// A version of AssemblyConnectivity that supports a string-based interface.
// Prefer AssemblyConnectivity if efficiency is a concern.
class AssemblyConnectivityWithStrings : public AssemblyConnectivity {
 public:
  // Creates an AssemblyConnectivity class.
  // `max_movable_joints` is the maximum number of non-fixed joints that are
  // allowed between any two links in a set returned from
  // `GetConnectedLinkIndices`.
  explicit AssemblyConnectivityWithStrings(const Assembly& assembly,
                                           int max_movable_joints = 0);
  // Returns the names of the link group connected to `link_name` according
  // to the construction parameter `max_movable_joints`. If the link index is
  // invalid, and empty span is returned.
  absl::Span<const std::string> GetConnectedLinkNames(
      absl::string_view link_name) const;

 private:
  absl::flat_hash_map<std::string, std::vector<std::string>>
      connected_link_names_;
};
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_ASSEMBLY_CONNECTIVITY_H_
