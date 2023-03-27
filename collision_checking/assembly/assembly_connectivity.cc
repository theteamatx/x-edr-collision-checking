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

#include "collision_checking/assembly/assembly_connectivity.h"

#include <string>
#include <vector>

#include "collision_checking/assembly/assembly.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"

namespace collision_checking {

AssemblyConnectivity::AssemblyConnectivity(const Assembly& assembly,
                                           const int max_movable_joints) {
  // Assemblies shouldn't be very large, and this function is not called many
  // times, so it should be sufficient to extract the connectivity by naively
  // walking the tree starting at every link.
  for (const Link& link : assembly.GetLinks()) {
    absl::flat_hash_set<int> visited;
    absl::flat_hash_set<int> connected_set;
    int remaining_movable_joints = max_movable_joints;
    RecurseOutwardAddingLinks(link, connected_set, visited,
                              remaining_movable_joints);
    std::vector<int>& connected_link_vector = connected_links_[link.GetIndex()];
    connected_link_vector.reserve(connected_set.size());
    for (int i : connected_set) {
      connected_link_vector.push_back(assembly.GetLink(i).GetIndex());
    }
  }
}

void AssemblyConnectivity::RecurseOutwardAddingLinks(
    const Link& link, absl::flat_hash_set<int>& connected_links,
    absl::flat_hash_set<int>& visited, int& remaining_movable_joints) const {
  if (visited.contains(link.GetIndex())) {
    return;
  }
  connected_links.insert(link.GetIndex());
  visited.insert(link.GetIndex());
  // If the parent link is attached by a fixed joint, add it and recurse farther
  // toward the root.
  const Joint* parent_joint = link.GetParentJoint();
  if (parent_joint != nullptr) {
    const Link& parent_link = parent_joint->GetParentLink();
    if (parent_joint->GetParameters().type == Joint::FIXED) {
      connected_links.insert(parent_link.GetIndex());
      RecurseOutwardAddingLinks(parent_link, connected_links, visited,
                                remaining_movable_joints);
    } else if (remaining_movable_joints > 0) {
      --remaining_movable_joints;
      connected_links.insert(parent_link.GetIndex());
      RecurseOutwardAddingLinks(parent_link, connected_links, visited,
                                remaining_movable_joints);
      ++remaining_movable_joints;
    }
  }

  for (const Joint& child_joint : link.GetChildJoints()) {
    if (child_joint.GetParameters().type == Joint::FIXED) {
      connected_links.insert(child_joint.GetChildLink().GetIndex());
      RecurseOutwardAddingLinks(child_joint.GetChildLink(), connected_links,
                                visited, remaining_movable_joints);
    } else if (remaining_movable_joints > 0) {
      if (remaining_movable_joints > 0) {
        --remaining_movable_joints;
        connected_links.insert(child_joint.GetChildLink().GetIndex());
        RecurseOutwardAddingLinks(child_joint.GetChildLink(), connected_links,
                                  visited, remaining_movable_joints);
        ++remaining_movable_joints;
      }
    }
  }
}

absl::Span<const int> AssemblyConnectivity::GetConnectedLinkIndices(
    const int link_index) const {
  const auto links = connected_links_.find(link_index);
  if (links == connected_links_.end()) {
    return {};
  }
  return links->second;
}

AssemblyConnectivityWithStrings::AssemblyConnectivityWithStrings(
    const Assembly& assembly, const int max_movable_joints)
    : AssemblyConnectivity(assembly, max_movable_joints) {
  connected_link_names_.reserve(assembly.GetLinkCount());
  for (const Link& link : assembly.GetLinks()) {
    std::vector<std::string>& connected_names =
        connected_link_names_[link.GetName()];
    absl::Span<const int> connected_indices =
        GetConnectedLinkIndices(link.GetIndex());
    connected_names.reserve(connected_indices.size());
    for (const int connected_index : connected_indices) {
      connected_names.push_back(assembly.GetLink(connected_index).GetName());
    }
  }
}

absl::Span<const std::string>
AssemblyConnectivityWithStrings::GetConnectedLinkNames(
    absl::string_view link_name) const {
  const auto links = connected_link_names_.find(link_name);
  if (links == connected_link_names_.end()) {
    return {};
  }
  return links->second;
}

}  // namespace collision_checking
