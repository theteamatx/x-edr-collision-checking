#include "experimental/users/buschmann/collision_checking/assembly/assembly.h"

#include <string>

#include "experimental/users/buschmann/collision_checking/assembly/assembly.proto.h"
#include "experimental/users/buschmann/collision_checking/assembly/geometry.h"
#include "experimental/users/buschmann/collision_checking/assembly/joint.h"
#include "experimental/users/buschmann/collision_checking/assembly/link.h"
#include "experimental/users/buschmann/collision_checking/assembly/proto_utils.h"
#include "experimental/users/buschmann/collision_checking/geometry_shapes/box.h"
#include "experimental/users/buschmann/collision_checking/test_utils.h"
#include "third_party/absl/status/status.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"

namespace collision_checking {
namespace {

using ::blue::eigenmath::testing::IsApprox;
using testing::ParseTextProtoOrDie;

static const char kExampleLinkage[] =
    R"pb(name: "TwoLegs"
         joints {
           name: "joint3"
           parent_link_name: "root_link"
           child_link_name: "link3"
           limits {
             lower: -1.7976931348623157e+308
             upper: 1.7976931348623157e+308
             velocity: 1.7976931348623157e+308
             acceleration: 1.7976931348623157e+308
             jerk: 1.7976931348623157e+308
             effort: 1.7976931348623157e+308
           }
           dynamics {}
           calibration {}
           type: REVOLUTE
           parent_t_joint { ty: -0.5 rw: 1 }
           axis { vec: 1 vec: 0 vec: 0 }
         }
         joints {
           name: "joint1"
           parent_link_name: "root_link"
           child_link_name: "link1"
           limits {
             lower: -1.7976931348623157e+308
             upper: 1.7976931348623157e+308
             velocity: 1.7976931348623157e+308
             acceleration: 1.7976931348623157e+308
             jerk: 1.7976931348623157e+308
             effort: 1.7976931348623157e+308
           }
           dynamics {}
           calibration {}
           type: REVOLUTE
           parent_t_joint { ty: 0.5 rw: 1 }
           axis { vec: 0 vec: 1 vec: 0 }
         }
         joints {
           name: "joint4"
           parent_link_name: "link3"
           child_link_name: "link4"
           limits {
             lower: -1.7976931348623157e+308
             upper: 1.7976931348623157e+308
             velocity: 1.7976931348623157e+308
             acceleration: 1.7976931348623157e+308
             jerk: 1.7976931348623157e+308
             effort: 1.7976931348623157e+308
           }
           dynamics {}
           calibration {}
           type: REVOLUTE
           parent_t_joint { tz: 1 rw: 1 }
           axis { vec: 1 vec: 0 vec: 0 }
         }
         joints {
           name: "attachEndEffector"
           parent_link_name: "link4"
           child_link_name: "end_effector_link"
           limits {}
           dynamics {}
           calibration {}
           type: FIXED
           parent_t_joint { tx: 1 rw: 1 }
           axis { vec: 1 vec: 0 vec: 0 }
         }
         joints {
           name: "joint2"
           parent_link_name: "link1"
           child_link_name: "link2"
           limits {
             lower: -1.7976931348623157e+308
             upper: 1.7976931348623157e+308
             velocity: 1.7976931348623157e+308
             acceleration: 1.7976931348623157e+308
             jerk: 1.7976931348623157e+308
             effort: 1.7976931348623157e+308
           }
           dynamics {}
           calibration {}
           type: REVOLUTE
           parent_t_joint { tz: 1 rw: 1 }
           axis { vec: 1 vec: 0 vec: 0 }
         }
         links {
           name: "root_link"
           visual_shapes {
             primitive { box { size { vec: 1 vec: 1 vec: 1 } } }
             link_t_shape { rw: 1 }
           }
           inertial {
             mass: 1
             link_t_inertial { rw: 1 }
             inertia {
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
             }
           }
         }
         links {
           name: "link3"
           visual_shapes {
             primitive { box { size { vec: 1 vec: 1 vec: 1 } } }
             link_t_shape { rw: 1 }
           }
           inertial {
             mass: 1
             link_t_inertial { rw: 1 }
             inertia {
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
             }
           }
         }
         links {
           name: "link4"
           visual_shapes {
             primitive { box { size { vec: 1 vec: 1 vec: 1 } } }
             link_t_shape { rw: 1 }
           }
           inertial {
             mass: 1
             link_t_inertial { rw: 1 }
             inertia {
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
             }
           }
         }
         links {
           name: "link1"
           visual_shapes {
             primitive { box { size { vec: 1 vec: 1 vec: 1 } } }
             link_t_shape { rw: 1 }
           }
           inertial {
             mass: 5
             link_t_inertial { tx: 0.25 ty: 0.2 tz: 0.1 rw: 1 }
             inertia {
               mat: 1
               mat: 0.2
               mat: 0
               mat: 0.2
               mat: 0.8
               mat: 0
               mat: 0
               mat: 0
               mat: 0.3
             }
           }
         }
         links {
           name: "link2"
           visual_shapes {
             primitive { capsule { length: 2 radius: 0.3 } }
             link_t_shape { rw: 1 }
           }
           inertial {
             mass: 1
             link_t_inertial { rw: 1 }
             inertia {
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
             }
           }
         }
         links {
           name: "end_effector_link"
           visual_shapes {
             primitive { box { size { vec: 1 vec: 1 vec: 1 } } }
             link_t_shape { rw: 1 }
           }
           inertial {
             mass: 1
             link_t_inertial { rw: 1 }
             inertia {
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
               mat: 0
             }
           }
         }
    )pb";

TEST(CreateAssemblyTest, CreateExplicitly) {
  Assembly assembly{"kinematics"};

  geometry_shapes::Box box_geo_params(Eigen::Vector3d::Constant(1.0));

  // Links
  Link& root_link = assembly.CreateLink("root_link", Link::Parameters{});
  assembly.CreateGeometry("box", Geometry::Type::VISUAL, box_geo_params,
                          &root_link);

  Link& link1 = assembly.CreateLink("link1", Link::Parameters{});
  assembly.CreateGeometry("box", Geometry::Type::VISUAL, box_geo_params,
                          &link1);

  Link& link2 = assembly.CreateLink("link2", Link::Parameters{});
  assembly.CreateGeometry("box", Geometry::Type::VISUAL, box_geo_params,
                          &link2);

  Link& link3 = assembly.CreateLink("link3", Link::Parameters{});
  assembly.CreateGeometry("box", Geometry::Type::VISUAL, box_geo_params,
                          &link3);

  Link& link4 = assembly.CreateLink("link4", Link::Parameters{});
  assembly.CreateGeometry("box", Geometry::Type::VISUAL, box_geo_params,
                          &link4);

  // Joints
  Joint::Parameters joint1_params;
  joint1_params.parent_pose_joint.translation() = Vector3d{0, 0.5, 0};
  assembly.CreateJoint("joint1", joint1_params, root_link.GetName(),
                       link1.GetName());

  Joint::Parameters joint2_params;
  joint2_params.parent_pose_joint.translation() = Vector3d{0, 0, 1};
  assembly.CreateJoint("joint2", joint2_params, link1.GetName(),
                       link2.GetName());

  Joint::Parameters joint3_params;
  joint3_params.parent_pose_joint.translation() = Vector3d{0, -0.5, 0};
  assembly.CreateJoint("joint3", joint3_params, root_link.GetName(),
                       link3.GetName());

  Joint::Parameters joint4_params;
  joint4_params.parent_pose_joint.translation() = Vector3d{0, 0, 1};
  assembly.CreateJoint("joint4", joint4_params, link3.GetName(),
                       link4.GetName());

  // Finalize.
  ASSERT_EQ(assembly.Finalize().code(), absl::StatusCode::kOk);

  // Some expectations
  ASSERT_TRUE(assembly.HasBranching());
  ASSERT_FALSE(assembly.IsSerialChain());
  ASSERT_EQ(4, assembly.GetDofCount());
}

TEST(CreateAssemblyTest, CreateFromConfig) {
  const auto assembly_proto =
      ParseTextProtoOrDie<proto::Assembly>(kExampleLinkage);
  auto assembly = FromProto(assembly_proto);
  ASSERT_TRUE(assembly.ok());

  // Some expectations
  ASSERT_TRUE(assembly->HasBranching());
  ASSERT_FALSE(assembly->IsSerialChain());
  ASSERT_EQ(4, assembly->GetDofCount());
}

TEST(CreateAssemblyTest, Clone) {
  const auto assembly_proto =
      ParseTextProtoOrDie<proto::Assembly>(kExampleLinkage);
  auto assembly = FromProto(assembly_proto);
  ASSERT_TRUE(assembly.ok());

  auto non_finalized_clone = assembly->Clone(/*auto_finalize=*/false);
  ASSERT_NE(non_finalized_clone, nullptr);
  EXPECT_FALSE(non_finalized_clone->IsFinalized());

  auto clone = assembly->Clone();
  ASSERT_NE(clone, nullptr);
  EXPECT_TRUE(clone->IsFinalized());

  EXPECT_EQ(assembly->GetName(), clone->GetName());
  EXPECT_EQ(assembly->HasBranching(), clone->HasBranching());
  EXPECT_EQ(assembly->IsSerialChain(), clone->IsSerialChain());
  EXPECT_EQ(assembly->GetDofCount(), clone->GetDofCount());
  ASSERT_EQ(assembly->GetLinks().size(), clone->GetLinks().size());
  ASSERT_EQ(assembly->GetJoints().size(), clone->GetJoints().size());

  for (int link = 0; link < assembly->GetLinks().size(); link++) {
    auto& link1 = assembly->GetLink(link);
    auto& link2 = clone->GetLink(link);
    EXPECT_EQ(link1.GetIndex(), link2.GetIndex());
    EXPECT_EQ(link1.GetName(), link2.GetName());
    EXPECT_EQ(link1.GetChildJoints().size(), link2.GetChildJoints().size());
    auto& params1 = link1.GetParameters();
    auto& params2 = link2.GetParameters();
    EXPECT_EQ(params1.mass, params2.mass);
    EXPECT_TRUE(
        params1.link_pose_inertial.isApprox(params2.link_pose_inertial));
    EXPECT_TRUE(params1.inertia.isApprox(params2.inertia));
  }

  for (int joint = 0; joint < assembly->GetJoints().size(); joint++) {
    auto& joint1 = assembly->GetJoint(joint);
    auto& joint2 = clone->GetJoint(joint);

    EXPECT_EQ(joint1.GetIndex(), joint2.GetIndex());
    EXPECT_EQ(joint1.GetName(), joint2.GetName());
    EXPECT_EQ(joint1.GetDofCount(), joint2.GetDofCount());
    EXPECT_EQ(joint1.GetDofIndex(), joint2.GetDofIndex());
    EXPECT_EQ(joint1.GetParentLink().GetName(),
              joint2.GetParentLink().GetName());

    auto& params1 = joint1.GetParameters();
    auto& params2 = joint2.GetParameters();
    EXPECT_EQ(params1.type, params2.type);
    EXPECT_TRUE(params1.axis.isApprox(params2.axis));
    EXPECT_TRUE(params1.parent_pose_joint.isApprox(params2.parent_pose_joint));
    EXPECT_DOUBLE_EQ(params1.limits.jerk, params2.limits.jerk);
    EXPECT_DOUBLE_EQ(params1.limits.acceleration, params2.limits.acceleration);
    EXPECT_DOUBLE_EQ(params1.limits.velocity, params2.limits.velocity);
    EXPECT_DOUBLE_EQ(params1.limits.upper, params2.limits.upper);
    EXPECT_DOUBLE_EQ(params1.limits.lower, params2.limits.lower);
    EXPECT_DOUBLE_EQ(params1.limits.effort, params2.limits.effort);
    EXPECT_DOUBLE_EQ(params1.dynamics.damping, params2.dynamics.damping);
    EXPECT_DOUBLE_EQ(params1.dynamics.friction, params2.dynamics.friction);
    EXPECT_DOUBLE_EQ(params1.calibration.falling, params2.calibration.falling);
    EXPECT_DOUBLE_EQ(params1.calibration.rising, params2.calibration.rising);
    EXPECT_DOUBLE_EQ(params1.calibration.home, params2.calibration.home);
  }

  ASSERT_EQ(assembly->GetGeometries().size(), clone->GetGeometries().size());

  auto geo1 = assembly->GetGeometries().begin();
  auto geo2 = clone->GetGeometries().begin();
  while (geo1 != assembly->GetGeometries().end() &&
         geo2 != clone->GetGeometries().end()) {
    EXPECT_EQ(geo1->GetName(), geo2->GetName());
    EXPECT_EQ(geo1->GetType(), geo2->GetType());
    EXPECT_EQ(geo1->GetLink().GetName(), geo2->GetLink().GetName());
    EXPECT_EQ(geo1->GetShape().GetType(), geo2->GetShape().GetType());
    geo1++;
    geo2++;
  }
}

TEST(CreateAssemblyTest, JointParametersAreNormalized) {
  Assembly assembly{"assembly"};

  assembly.CreateLink("root_link", Link::Parameters{});
  assembly.CreateLink("child_link", Link::Parameters{});
  assembly.CreateLink("grandchild_link", Link::Parameters{});
  Joint::Parameters parameters{};
  parameters.axis = Vector3d(-0.1, 0, 0);
  Joint& joint1 =
      assembly.CreateJoint("joint1", parameters, "root_link", "child_link");
  parameters.axis.setZero();
  parameters.type = Joint::FIXED;
  Joint& joint2 = assembly.CreateJoint("joint2", parameters, "child_link",
                                       "grandchild_link");

  EXPECT_THAT(joint1.GetParameters().axis, IsApprox(Vector3d(-1, 0, 0)));
  EXPECT_THAT(joint2.GetParameters().axis, IsApprox(Vector3d(0, 0, 0)));
}

TEST(CreateAssemblyTest, MissingPosesAreConvertedToIdentity) {
  const auto assembly_proto = ParseTextProtoOrDie<proto::Assembly>(
      R"pb(name: "TheAssembly"
           joints {
             name: "joint0"
             parent_link_name: "link0"
             child_link_name: "link1"
             type: FIXED
           }
           links { name: "link0" }
           links { name: "link1" }
      )pb");
  auto assembly = FromProto(assembly_proto);
  ASSERT_TRUE(assembly.ok());

  EXPECT_THAT(assembly->GetJoint(0).GetParameters().parent_pose_joint,
              IsApprox(Pose3d()));
  for (const auto& link : assembly->GetLinks()) {
    EXPECT_THAT(link.GetParameters().link_pose_inertial, IsApprox(Pose3d()));
  }
}

TEST(CreateAssemblyDeathTest, DiesIfAxisIsZero) {
  Assembly assembly{"assembly"};

  assembly.CreateLink("root_link", Link::Parameters{});
  assembly.CreateLink("child_link", Link::Parameters{});
  Joint::Parameters parameters{};
  parameters.axis = Vector3d(0, 0, 0);
  EXPECT_DEATH(
      assembly.CreateJoint("joint1", parameters, "root_link", "child_link"),
      "zero axis.");
}

}  // namespace
}  // namespace collision_checking

