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

#include "collision_checking/assembly/assembly.h"

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "collision_checking/geometry_shapes/composite_shape.h"
#include "collision_checking/logging.h"

namespace collision_checking {

std::unique_ptr<Assembly> Assembly::Clone(bool auto_finalize) const {
  auto twin = std::make_unique<Assembly>(name_);
  for (const auto& link : links_) {
    twin->CreateLink(link->GetName(), link->GetParameters());
  }
  for (const auto& joint : joints_) {
    twin->CreateJoint(joint->GetName(), joint->GetParameters(),
                      joint->GetParentLink().GetName(),
                      joint->GetChildLink().GetName());
  }

  for (const auto& geometry : geometries_) {
    auto twin_link = twin->FindLink(geometry->GetLink().GetName());
    CC_CHECK_NE(twin_link, nullptr);
    twin->CreateGeometry(geometry->GetName(), geometry->GetType(),
                         geometry->GetShape(), twin_link);
  }

  if (auto_finalize && finalized_) {
    const auto status = twin->Finalize();
    CC_CHECK(status.ok(), "Finalize failed: %s", status.ToString().c_str());
  }
  return twin;
}

Assembly::Assembly(absl::string_view name) : name_(name), finalized_(false) {}

// LINKS
Link& Assembly::CreateLink(absl::string_view name,
                           const Link::Parameters& params) {
  // TODO: Return an absl::Status instead of asserting.
  // Can't create links in a finalized assembly.
  CC_CHECK(!finalized_, "The assembly is already finalized.");
  CC_CHECK(!link_map_.contains(name), "Link name '%s' is not unique.", name);

  // Make, store, and return the link.
  auto link =
      std::make_unique<Link>(Link::ConstructionKey{}, this, name, params);
  link_map_[link->GetName()] = link.get();
  links_.push_back(std::move(link));
  return *(links_.back());
}

Link* Assembly::FindLink(absl::string_view name) {
  auto found_iter = link_map_.find(name);
  if (found_iter == link_map_.end()) {
    return nullptr;
  } else {
    return (*found_iter).second;
  }
}

const Link* Assembly::FindLink(absl::string_view name) const {
  auto found_iter = link_map_.find(name);
  if (found_iter == link_map_.end()) {
    return nullptr;
  } else {
    return (*found_iter).second;
  }
}

Link& Assembly::GetLink(std::size_t index) {
  CC_CHECK_LT(index, links_.size());
  CC_CHECK_NE(links_[index].get(), nullptr);
  return *links_[index];
}

const Link& Assembly::GetLink(std::size_t index) const {
  CC_CHECK_LT(index, links_.size());
  CC_CHECK_NE(links_[index].get(), nullptr);
  return *links_[index];
}

// JOINTS

Joint& Assembly::CreateJoint(absl::string_view name,
                             const Joint::Parameters& params,
                             absl::string_view parent_link_name,
                             absl::string_view child_link_name) {
  // TODO: Return an absl::Status instead of asserting.
  // Can't create joints in a finalized assembly.
  CC_CHECK(!finalized_, "The assembly is already finalized.");
  CC_CHECK(!joint_map_.contains(name), "Joint name '%s' is not unique.", name);

  // Get parent link config node, use it to find parent link
  Link* parent_link = FindLink(parent_link_name);
  CC_CHECK_NE(parent_link, nullptr,
              "Parent link '%s' for joint '%s' does not exist.",
              parent_link_name, name);

  // Get child link config node, use it to find child link
  Link* child_link = FindLink(child_link_name);
  CC_CHECK_NE(child_link, nullptr,
              "Child link '%s' for joint '%s' does not exist.", child_link_name,
              name);

  // Make and store the joint.
  auto joint = std::make_unique<Joint>(Joint::ConstructionKey{}, this, name,
                                       params, parent_link, child_link);
  joint_map_[joint->GetName()] = joint.get();
  joints_.push_back(std::move(joint));
  Joint* joint_ptr = joints_.back().get();

  // Wire us into the parent and child links
  parent_link->child_joints_.push_back(joint_ptr);
  CC_CHECK_EQ(child_link->parent_joint_, nullptr,
              "Child link '%s' already has a parent joint.",
              child_link->GetName().c_str());
  child_link->parent_joint_ = joint_ptr;

  return *joint_ptr;
}

Joint* Assembly::FindJoint(absl::string_view name) {
  auto found_iter = joint_map_.find(name);
  if (found_iter == joint_map_.end()) {
    return nullptr;
  } else {
    return (*found_iter).second;
  }
}

const Joint* Assembly::FindJoint(absl::string_view name) const {
  auto found_iter = joint_map_.find(name);
  if (found_iter == joint_map_.end()) {
    return nullptr;
  } else {
    return (*found_iter).second;
  }
}

// GEOMETRIES

Geometry& Assembly::CreateGeometry(absl::string_view name, Geometry::Type type,
                                   const geometry_shapes::ShapeBase& shape,
                                   Link* link) {
  // TODO: Return an absl::Status instead of asserting.
  // Can't create geometries in a finalized assembly.
  CC_CHECK_NE(link, nullptr);
  CC_CHECK(!finalized_, "The assembly is already finalized.");

  // Make and store the geometry.
  auto geometry = std::make_unique<Geometry>(Geometry::ConstructionKey{}, this,
                                             name, type, shape, link);
  CC_CHECK_NE(geometry, nullptr);

  geometry_map_[geometry->GetName()] = geometry.get();
  geometries_.push_back(std::move(geometry));
  Geometry* geometry_ptr = geometries_.back().get();

  // Wire up the link
  auto geom_type = geometry_ptr->GetType();
  switch (geom_type) {
    case Geometry::Type::COLLISION:
      link->collision_geometries_.push_back(geometry_ptr);
      break;
    case Geometry::Type::VISUAL:
      link->visual_geometries_.push_back(geometry_ptr);
      break;
  }

  return *geometry_ptr;
}

Geometry* Assembly::FindGeometry(absl::string_view name) {
  auto found_iter = geometry_map_.find(name);
  if (found_iter == geometry_map_.end()) {
    return nullptr;
  } else {
    return (*found_iter).second;
  }
}

const Geometry* Assembly::FindGeometry(absl::string_view name) const {
  auto found_iter = geometry_map_.find(name);
  if (found_iter == geometry_map_.end()) {
    return nullptr;
  } else {
    return (*found_iter).second;
  }
}

absl::StatusOr<std::unique_ptr<geometry_shapes::ShapeBase>>
Assembly::MakeFixedVisualGeometry() const {
  return MakeFixedGeometry(/*visual_shapes=*/true);
}
absl::StatusOr<std::unique_ptr<geometry_shapes::ShapeBase>>
Assembly::MakeFixedCollisionGeometry() const {
  return MakeFixedGeometry(/*visual_shapes=*/false);
}

absl::StatusOr<std::unique_ptr<geometry_shapes::ShapeBase>>
Assembly::MakeFixedGeometry(bool visual_shapes) const {
  if (GetDofCount() != 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Assembly is not fixed, it has ", GetDofCount(), " dofs."));
  }

  // Depth-first traversal task to be stacked up.
  struct DftTask {
    Pose3d root_pose_parent;
    const Link* link;
  };

  std::vector<DftTask> stack;
  stack.push_back({.root_pose_parent = Pose3d::Identity(), .link = root_link_});

  std::vector<std::unique_ptr<geometry_shapes::ShapeBase>> sub_shapes;

  while (!stack.empty()) {
    DftTask current_task = stack.back();
    stack.pop_back();
    for (const Geometry& geom :
         (visual_shapes ? current_task.link->GetVisualGeometries()
                        : current_task.link->GetCollisionGeometries())) {
      auto moved_shape = geom.GetShape().Clone();
      moved_shape->SetLocalTransform(current_task.root_pose_parent *
                                     moved_shape->GetLocalTransform());
      sub_shapes.emplace_back(std::move(moved_shape));
    }
    for (const Joint& joint : current_task.link->GetChildJoints()) {
      stack.push_back(
          {.root_pose_parent = current_task.root_pose_parent *
                               joint.GetParameters().parent_pose_joint,
           .link = &joint.GetChildLink()});
    }
  }

  return std::make_unique<geometry_shapes::CompositeShape>(
      std::move(sub_shapes));
}

// FINALIZATION

absl::Status Assembly::Finalize() {
  // This sorts the links so that they've all got the correct indices
  if (const auto status = TopologicallySortLinks(); !status.ok()) {
    return status;
  }

  // Sort the joints by the dual index of their parent and then child link
  // indices.
  std::sort(joints_.begin(), joints_.end(),
            [](const std::unique_ptr<Joint>& joint_a,
               const std::unique_ptr<Joint>& joint_b) {
              auto parent_index_a = joint_a->parent_link_->index_;
              auto parent_index_b = joint_b->parent_link_->index_;
              if (parent_index_a < parent_index_b) {
                return true;
              } else if (parent_index_a > parent_index_b) {
                return false;
              } else {
                return joint_a->child_link_->index_ <
                       joint_b->child_link_->index_;
              }
            });

  // Set the indices of the joints based on their sorted joint order.
  // Also set the DOF index, based on the number of DOFs of each joint.
  int32_t index = 0;
  int32_t dof_index = 0;
  for (auto iter = joints_.begin(); iter != joints_.end(); ++iter, ++index) {
    (*iter)->index_ = index;
    auto dof_count = (*iter)->GetDofCount();
    if (dof_count > 0) {
      (*iter)->dof_index_ = dof_index;
      dof_index += dof_count;
    } else {
      (*iter)->dof_index_ = -1;
    }
  }

  // Finalized!
  finalized_ = true;
  return absl::OkStatus();
}

bool Assembly::HasBranching() const {
  for (const auto& link : links_) {
    if (link->child_joints_.size() > 1) {
      return true;
    }
  }
  return false;
}

bool Assembly::IsSerialChain() const {
  for (const auto& link : links_) {
    if (link->child_joints_.size() > 1) {
      return false;
    }
  }
  return true;
}

size_t Assembly::GetDofCount() const {
  size_t count = 0;
  for (const auto& joint : joints_) {
    count += joint->GetDofCount();
  }
  return count;
}

// The topological sort algorithm treats the assembly as a directed, acyclic
// graph (DAG) where the links are nodes and the joints are directed edges.
//
// This algorithm for topological sorting is based on depth-first search. The
// algorithm loops through each node of the graph, in an arbitrary order,
// initiating a depth-first search that terminates when it hits any node that
// has already been visited since the beginning of the topological sort or the
// node has no outgoing edges (i.e. a leaf node):
//
// Each node n gets prepended to the output list L only after considering all
// other nodes which depend on n (all descendants of n in the graph).
// Specifically, when the algorithm adds node n, we are guaranteed that all
// nodes which depend on n are already in the output list L: they were added to
// L either by the recursive call to visit() which ended before the call to
// visit n, or by a call to visit() which started even before the call to
// visit n. Since each edge and node is visited once, the algorithm runs in
// linear time. This depth-first-search-based algorithm is the one described by
// Cormen et al. (2001); it seems to have been first described in print by
// Tarjan (1976).
absl::Status Assembly::TopologicallySortLinks() {
  if (links_.empty()) {
    return absl::InternalError("No links in Assembly");
  }

  // Sort the links lexicographically
  std::sort(links_.begin(), links_.end(),
            [](const std::unique_ptr<Link>& link_a,
               const std::unique_ptr<Link>& link_b) {
              return link_a->GetName() < link_b->GetName();
            });

  // First, set the index on all the links to -1, as a starting point
  // to mark them as un-set. Also find all the root links.
  for (auto& link : links_) {
    link->index_ = -1;
  }

  // Create a temporary deque for storage
  std::deque<Link*> sorted_links;

  // Visit all the links.
  for (auto& link : links_) {
    VisitLink(link.get(), &sorted_links);
  }
  CC_CHECK_EQ(sorted_links.size(), links_.size(),
              "All links should have been visited and sorted.");

  // Use the sorted list to index the links.
  int index = 0;
  for (auto iter = sorted_links.begin(); iter != sorted_links.end();
       ++iter, ++index) {
    (*iter)->index_ = index;
  }

  // Use the link indices to sort the links.
  std::sort(links_.begin(), links_.end(),
            [](const std::unique_ptr<Link>& link_a,
               const std::unique_ptr<Link>& link_b) {
              return link_a->index_ < link_b->index_;
            });

  // Find the root link.
  root_link_ = nullptr;
  for (auto& link : links_) {
    if (link->parent_joint_ == nullptr) {
      CC_CHECK_EQ(root_link_, nullptr,
                  "Assembly is not a tree, parentless links: '%s' and '%s'.",
                  root_link_->GetName().c_str(), link->GetName().c_str());
      root_link_ = link.get();
    }
  }
  CC_CHECK_NE(root_link_, nullptr,
              "There must be at least one root link in an acyclic assembly.");

  // The root links are already sorted by index, because they were inserted
  // in the order of the regular links list, which is sorted by index.

  // Test to make sure that the ordering invariant is preserved
  // The invariant is that all upstream nodes have lower indices, and all
  // downstream nodes have higher indices.
  for (const auto& link : links_) {
    CC_CHECK_GE(link->index_, 0, "All links should be indexed.");
    if (link->parent_joint_ != nullptr) {
      CC_CHECK_GT(link->index_, link->parent_joint_->parent_link_->index_,
                  "Parent links must have higher indices.");
    }
    for (const auto child_joint : link->child_joints_) {
      CC_CHECK_LT(link->index_, child_joint->child_link_->index_,
                  "Child links must have lower indices.");
    }
  }

  return absl::OkStatus();
}

void Assembly::VisitLink(Link* link, std::deque<Link*>* sorted_links) {
  // If link has a temporary mark, assembly is not acyclic
  CC_CHECK_NE(link->index_, -2, "Kinematic cycles are unsupported.");

  // If link is marked, but not temporarily, it has been visited already
  if (link->index_ >= 0) {
    return;
  }

  // Mark link temporarily
  link->index_ = -2;

  // Sort the child joints in place by the names of their child links.
  std::sort(link->child_joints_.begin(), link->child_joints_.end(),
            [](const Joint* joint_a, const Joint* joint_b) {
              return joint_a->child_link_->GetName() <
                     joint_b->child_link_->GetName();
            });

  // Loop over child links of the child joints, visiting them.
  for (auto child_joint : link->child_joints_) {
    VisitLink(child_joint->child_link_, sorted_links);
  }

  // Mark link permanently
  link->index_ = 0;

  // Add link to head of sorted links
  sorted_links->push_front(link);
}

}  // namespace collision_checking
