#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_UTILS_IMPL_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_UTILS_IMPL_H_

#include <memory>
#include <utility>

#include "experimental/users/buschmann/collision_checking/assembly/assembly.h"
#include "experimental/users/buschmann/collision_checking/assembly/assembly.proto.h"
#include "experimental/users/buschmann/collision_checking/geometry_shapes/eigen_proto_utils.h"
#include "experimental/users/buschmann/collision_checking/geometry_shapes/utils_impl.h"
#include "third_party/absl/status/status.h"
#include "third_party/absl/status/statusor.h"
#include "third_party/absl/strings/str_cat.h"

namespace collision_checking {
namespace utils_impl {

namespace details {

template <typename LinkProto>
absl::Status CreateLink(Assembly* assembly, const LinkProto& link_proto) {
  Link::Parameters link_params;
  link_params.mass = 1.0;

  if (link_proto.has_inertial()) {
    const auto& inertial = link_proto.inertial();
    link_params.mass = inertial.mass();

    link_params.link_pose_inertial = PoseFromProto(inertial.link_t_inertial());
    link_params.inertia = EigenMatrixFromProto(inertial.inertia());
  }

  Link& link = assembly->CreateLink(link_proto.name(), link_params);

  // Loop over visual geometries, and give each one a unique name.
  int geometry_count = 0;
  for (const auto& visual_shape : link_proto.visual_shapes()) {
    const auto shape = MakeShapeFromProto(visual_shape);
    if (!shape.ok()) {
      return shape.status();
    }
    assembly->CreateGeometry(
        absl::StrCat(link_proto.name(), "_viz_geometry_", geometry_count++),
        Geometry::Type::VISUAL, **shape, &link);
  }

  // Loop over collision geometries and give each one a unique name.
  geometry_count = 0;
  for (const auto& collision_shape : link_proto.collision_shapes()) {
    const auto shape = MakeShapeFromProto(collision_shape);
    if (!shape.ok()) {
      return shape.status();
    }
    assembly->CreateGeometry(
        absl::StrCat(link_proto.name(), "_collision_geometry_",
                     geometry_count++),
        Geometry::Type::COLLISION, **shape, &link);
  }

  return absl::OkStatus();
}

template <typename JointProto>
absl::Status CreateJoint(Assembly* assembly, const JointProto& joint_proto) {
  Joint::Parameters joint_params;
  switch (joint_proto.type()) {
    case JointProto::REVOLUTE:
      joint_params.type = Joint::Type::REVOLUTE;
      break;
    case JointProto::CONTINUOUS:
      joint_params.type = Joint::Type::CONTINUOUS;
      break;
    case JointProto::PRISMATIC:
      joint_params.type = Joint::Type::PRISMATIC;
      break;
    case JointProto::FIXED:
      joint_params.type = Joint::Type::FIXED;
      break;
    default:
      return absl::OutOfRangeError("Error decoding joint type");
  }

  joint_params.parent_pose_joint = PoseFromProto(joint_proto.parent_t_joint());

  joint_params.axis = EigenVectorFromProto(joint_proto.axis());

  if (joint_params.type == Joint::FIXED) {
    joint_params.limits.lower = 0.0;
    joint_params.limits.upper = 0.0;
    joint_params.limits.effort = 0.0;
    joint_params.limits.velocity = 0.0;
    joint_params.limits.acceleration = 0.0;
    joint_params.limits.jerk = 0.0;
    joint_params.dynamics.friction = 0.0;
    joint_params.dynamics.damping = 0.0;
    joint_params.calibration.rising = 0.0;
    joint_params.calibration.falling = 0.0;
    joint_params.calibration.home = 0.0;
  } else {
    joint_params.limits.lower = joint_proto.limits().lower();
    joint_params.limits.upper = joint_proto.limits().upper();
    joint_params.limits.effort = joint_proto.limits().effort();
    joint_params.limits.velocity = joint_proto.limits().velocity();
    joint_params.limits.acceleration = joint_proto.limits().acceleration();
    joint_params.limits.jerk = joint_proto.limits().jerk();
    joint_params.dynamics.friction = joint_proto.dynamics().friction();
    joint_params.dynamics.damping = joint_proto.dynamics().damping();
    joint_params.calibration.rising = joint_proto.calibration().rising();
    joint_params.calibration.falling = joint_proto.calibration().falling();
    joint_params.calibration.home = joint_proto.calibration().home();
  }

  // Actually create the joint.
  assembly->CreateJoint(joint_proto.name(), joint_params,
                        joint_proto.parent_link_name(),
                        joint_proto.child_link_name());

  return absl::OkStatus();
}

template <typename AssemblyProto>
absl::StatusOr<Assembly> ParseProto(const AssemblyProto& assembly_proto,
                                    bool finalize) {
  Assembly assembly(assembly_proto.name());

  // Read and create all the links.
  // Don't use range-based for, as RepeatedPtrField doesn't work in blueproto
  // version (see b/111822760).
  for (size_t idx = 0; idx < assembly_proto.links_size(); idx++) {
    const auto status =
        details::CreateLink(&assembly, assembly_proto.links(idx));
    if (!status.ok()) {
      return status;
    }
  }

  // Read all the joints.
  // Don't use range-based for, as RepeatedPtrField doesn't work in blueproto
  // version.
  for (size_t idx = 0; idx < assembly_proto.joints_size(); idx++) {
    const auto status =
        details::CreateJoint(&assembly, assembly_proto.joints(idx));
    if (!status.ok()) {
      return status;
    }
  }

  if (finalize) {
    const auto status = assembly.Finalize();
    if (!status.ok()) {
      return status;
    }
  }

  return std::move(assembly);
}

}  // namespace details

// Main function to parse an AssemblyProto into an Assembly object.
// This will return a failure status on proto parsing errors.
// The assembly tree correctness is checked via CHECK() macros and will
// therefore terminate the program, e.g. duplicated joint/link names, etc.
template <typename AssemblyProto>
absl::StatusOr<Assembly> FromProto(const AssemblyProto& assembly_proto,
                                   bool finalize) {
  return details::ParseProto(assembly_proto, finalize);
}

// Forward Declaration
template <typename ShapeInfoProto>
absl::Status ToShapeInfoProto(const Geometry& geometry,
                              ShapeInfoProto* shape_proto);

namespace details {

template <typename JointProto>
void ToProto(const Joint& joint, JointProto* joint_proto) {
  joint_proto->set_name(joint.GetName());

  joint_proto->set_parent_link_name(joint.GetParentLink().GetName());
  joint_proto->set_child_link_name(joint.GetChildLink().GetName());

  // parent_t_joint
  ProtoFromPose(joint.GetParameters().parent_pose_joint,
                joint_proto->mutable_parent_t_joint());

  // axis
  ProtoFromVector(joint.GetParameters().axis, joint_proto->mutable_axis());

  // limits
  if (joint.GetParameters().type == Joint::FIXED) {
    joint_proto->mutable_limits()->set_lower(0.0);
    joint_proto->mutable_limits()->set_upper(0.0);
    joint_proto->mutable_limits()->set_effort(0.0);
    joint_proto->mutable_limits()->set_velocity(0.0);
    joint_proto->mutable_limits()->set_acceleration(0.0);
    joint_proto->mutable_limits()->set_jerk(0.0);
    joint_proto->mutable_dynamics()->set_friction(0.0);
    joint_proto->mutable_dynamics()->set_damping(0.0);
    joint_proto->mutable_calibration()->set_rising(0.0);
    joint_proto->mutable_calibration()->set_falling(0.0);
    joint_proto->mutable_calibration()->set_home(0.0);
  } else {
    joint_proto->mutable_limits()->set_lower(
        joint.GetParameters().limits.lower);
    joint_proto->mutable_limits()->set_upper(
        joint.GetParameters().limits.upper);
    joint_proto->mutable_limits()->set_velocity(
        joint.GetParameters().limits.velocity);
    joint_proto->mutable_limits()->set_acceleration(
        joint.GetParameters().limits.acceleration);
    joint_proto->mutable_limits()->set_jerk(joint.GetParameters().limits.jerk);
    joint_proto->mutable_limits()->set_effort(
        joint.GetParameters().limits.effort);

    // dynamics
    joint_proto->mutable_dynamics()->set_friction(
        joint.GetParameters().dynamics.friction);
    joint_proto->mutable_dynamics()->set_damping(
        joint.GetParameters().dynamics.damping);

    // calibration
    joint_proto->mutable_calibration()->set_rising(
        joint.GetParameters().calibration.rising);
    joint_proto->mutable_calibration()->set_falling(
        joint.GetParameters().calibration.falling);
    joint_proto->mutable_calibration()->set_home(
        joint.GetParameters().calibration.home);
  }

  switch (joint.GetParameters().type) {
    case Joint::REVOLUTE:
      joint_proto->set_type(JointProto::Type::Joint_Type_REVOLUTE);
      break;
    case Joint::CONTINUOUS:
      joint_proto->set_type(JointProto::Type::Joint_Type_CONTINUOUS);
      break;
    case Joint::PRISMATIC:
      joint_proto->set_type(JointProto::Type::Joint_Type_PRISMATIC);
      break;
    case Joint::FIXED:
      joint_proto->set_type(JointProto::Type::Joint_Type_FIXED);
      break;
  }
}

template <typename LinkProto>
absl::Status ToProto(const Link& link, LinkProto* link_proto) {
  link_proto->set_name(link.GetName());

  for (const auto& collision : link.GetCollisionGeometries()) {
    const auto status =
        ToShapeInfoProto(collision, link_proto->add_collision_shapes());
    if (!status.ok()) {
      return status;
    }
  }

  for (const auto& visual : link.GetVisualGeometries()) {
    const auto status =
        ToShapeInfoProto(visual, link_proto->add_visual_shapes());
    if (!status.ok()) {
      return status;
    }
  }

  ProtoFromPose(link.GetParameters().link_pose_inertial,
                link_proto->mutable_inertial()->mutable_link_t_inertial());
  const proto::Matrix3dProto inertia_proto =
      ProtoFromMatrix3d(link.GetParameters().inertia);
  if constexpr (std::is_same_v<proto::Matrix3dProto,
                               typeof(*link_proto->mutable_inertial()
                                           ->mutable_inertia())>) {
    *link_proto->mutable_inertial()->mutable_inertia() = inertia_proto;
  } else {
    link_proto->mutable_inertial()->mutable_inertia()->FromProto(inertia_proto);
  }

  link_proto->mutable_inertial()->set_mass(link.GetParameters().mass);

  return absl::OkStatus();
}
}  // namespace details

template <typename AssemblyProto>
absl::Status ToProto(const Assembly& assembly, AssemblyProto* assembly_proto) {
  assembly_proto->set_name(assembly.GetName());

  for (const auto& joint : assembly.GetJoints()) {
    details::ToProto(joint, assembly_proto->add_joints());
  }

  for (const auto& link : assembly.GetLinks()) {
    const auto status = details::ToProto(link, assembly_proto->add_links());
    if (!status.ok()) {
      return status;
    }
  }

  return absl::OkStatus();
}

template <typename ShapeInfoProto>
absl::StatusOr<std::unique_ptr<geometry_shapes::ShapeBase>> MakeShapeFromProto(
    const ShapeInfoProto& shape_proto) {
  Pose3d link_t_shape = PoseFromProto(shape_proto.link_t_shape());
  auto shape = geometry_shapes::utils_impl::FromProto(shape_proto.primitive());
  if (!shape.ok()) {
    return shape.status();
  }
  (*shape)->SetLocalTransform(link_t_shape);
  return std::move(shape);
}

template <typename ShapeInfoProto>
absl::Status ToShapeInfoProto(const Geometry& geometry,
                              ShapeInfoProto* shape_proto) {
  const auto status = geometry_shapes::utils_impl::ToShapePrimitiveProto(
      geometry.GetShape(), shape_proto->mutable_primitive());
  if (!status.ok()) {
    return status;
  }

  ProtoFromPose(geometry.GetShape().GetLocalTransform(),
                shape_proto->mutable_link_t_shape());

  return absl::OkStatus();
}

}  // namespace utils_impl
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_ASSEMBLY_UTILS_IMPL_H_
