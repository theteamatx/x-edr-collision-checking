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

#include "collision_checking/assembly/proto_utils.h"

#include "collision_checking/assembly/assembly.pb.h"
#include "collision_checking/test_utils.h"
#include "eigenmath/matchers.h"

namespace collision_checking {
namespace {

using ::eigenmath::testing::IsApprox;
using testing::ParseTextProtoOrDie;
using testing::ProtoIsEquivTo;
using testing::StatusCodeIs;

TEST(BadDecodeTest, EmptyAssemblyProto) {
  proto::Assembly assembly_proto;
  EXPECT_THAT(FromProto(assembly_proto),
              StatusCodeIs(absl::StatusCode::kInternal));
}

TEST(BadDecodeTest, MissingInertiaInLink) {
  proto::Assembly assembly_proto = ParseTextProtoOrDie<AssemblyProto>(R"pb(
    links {
      name: "link"
      inertial {
        link_t_inertial: { rw: 1 }
        mass: 2.0
      }
    }
  )pb");

  // Missing inertia gets initialized to zero.
  CC_ASSERT_OK_AND_ASSIGN(auto assembly, FromProto(assembly_proto));
  EXPECT_THAT(assembly.GetLink(0).GetParameters().inertia,
              IsApprox(Matrix3d::Zero()));
}

TEST(BadDecodeDeathTest, BadInertiaInLink) {
  proto::Assembly assembly_proto = ParseTextProtoOrDie<AssemblyProto>(R"pb(
    links {
      name: "link"
      inertial {
        link_t_inertial: { rw: 1 }
        mass: 2.0
        # Not positive (semi-) definite
        inertia: { mat: [ -1, 0, 0, 0, -1, 0, 0, 0, 1 ] }
      }
    }
  )pb");

  EXPECT_DEATH(FromProto(assembly_proto).IgnoreError(), "");
}

TEST(BadDecodeTest, EmptyVisualShapeInLink) {
  proto::Assembly assembly_proto = ParseTextProtoOrDie<AssemblyProto>(R"pb(
    links {
      name: "link"
      visual_shapes { link_t_shape: { rw: 1 } }
    }
  )pb");

  EXPECT_THAT(FromProto(assembly_proto),
              StatusCodeIs(absl::StatusCode::kInvalidArgument));
}

TEST(BadDecodeTest, EmptyCollisionShapeInLink) {
  proto::Assembly assembly_proto = ParseTextProtoOrDie<AssemblyProto>(R"pb(
    links {
      name: "link"
      collision_shapes { link_t_shape: { rw: 1 } }
    }
  )pb");

  EXPECT_THAT(FromProto(assembly_proto),
              StatusCodeIs(absl::StatusCode::kInvalidArgument));
}

TEST(BadDecodeTest, BadJointType) {
  proto::Assembly assembly_proto;
  auto joint = assembly_proto.mutable_joints()->Add();
  joint->set_type(static_cast<decltype(joint->type())>(-1));
  EXPECT_THAT(FromProto(assembly_proto),
              StatusCodeIs(absl::StatusCode::kOutOfRange));
}

TEST(BadDecodeTest, NonUniqueLinkNames) {
  proto::Assembly assembly_proto = ParseTextProtoOrDie<AssemblyProto>(R"pb(
    links { name: "link" }
    links { name: "link" }
  )pb");

  EXPECT_DEATH(FromProto(assembly_proto).value(), "is not unique");
}

TEST(ProtoUtilsTest, NonUniqueJointNames) {
  proto::Assembly assembly_proto = ParseTextProtoOrDie<AssemblyProto>(R"pb(
    links {
      name: "l0"
      collision_shapes {
        link_t_shape: { rw: 1 }
        primitive {
          box {
            size {
              vec: 1.0,
              vec: 1.0,
              vec: 1.0,
            }
          }
        }
      }
    }
    links {
      name: "l1"
      collision_shapes {
        link_t_shape: { rw: 1 }
        primitive {
          box {
            size {
              vec: 1.0,
              vec: 1.0,
              vec: 1.0,
            }
          }
        }
      }
    }
    joints {
      name: "q0"
      type: FIXED
      parent_link_name: "l0"
      child_link_name: "l1"
      parent_t_joint: { rw: 1 }
      axis: { vec: 1 vec: 0 vec: 0 }
    }
    joints {
      name: "q0"
      type: FIXED
      parent_link_name: "l0"
      child_link_name: "l1"
      parent_t_joint: { rw: 1 }
      axis: { vec: 1 vec: 0 vec: 0 }
    }
  )pb");

  EXPECT_DEATH(FromProto(assembly_proto).value(), "is not unique");
}

TEST(ProtoUtilsTest, BasicAssemblyJoints) {
  proto::Assembly assembly_proto = ParseTextProtoOrDie<AssemblyProto>(R"pb(
    links {
      name: "l0"
      inertial {
        mass: 65.47
        inertia {
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
        }
        link_t_inertial: { rw: 1 }
      }
      collision_shapes {
        link_t_shape: { rw: 1 }
        primitive {
          box {
            size {
              vec: 1.0,
              vec: 1.0,
              vec: 1.0,
            }
          }
        }
      }
    }
    links {
      name: "l1"
      inertial {
        mass: 23.08
        inertia {
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
        }
        link_t_inertial: { rw: 1 }
      }
      visual_shapes {
        link_t_shape: { rw: 1 }
        primitive {
          box {
            size {
              vec: 1.0,
              vec: 1.0,
              vec: 1.0,
            }
          }
        }
      }
    }
    joints {
      name: "q0"
      type: REVOLUTE
      parent_link_name: "l0"
      child_link_name: "l1"
      parent_t_joint: { rw: 1 }
      axis: { vec: 1 vec: 0 vec: 0 }
      limits {
        lower: -0.5
        upper: 0.1
        effort: 0.2
        velocity: 0.3
        acceleration: 0.4
        jerk: 0.5
      }
      dynamics { friction: 0.6 damping: 0.7 }
      calibration { rising: 0.8 falling: 0.9 home: 1.0 }
    }
  )pb");

  CC_ASSERT_OK_AND_ASSIGN(auto assembly, FromProto(assembly_proto));

  const auto l0_link = assembly.FindLink("l0");
  ASSERT_NE(nullptr, l0_link);
  EXPECT_EQ(65.47, l0_link->GetParameters().mass);
  EXPECT_EQ(0, l0_link->GetVisualGeometries().size());
  EXPECT_EQ(1, l0_link->GetCollisionGeometries().size());

  const auto l1_link = assembly.FindLink("l1");
  ASSERT_NE(nullptr, l1_link);
  EXPECT_EQ(23.08, l1_link->GetParameters().mass);
  EXPECT_EQ(1, l1_link->GetVisualGeometries().size());
  EXPECT_EQ(0, l1_link->GetCollisionGeometries().size());

  const auto joint = assembly.FindJoint("q0");
  ASSERT_NE(nullptr, joint);
  EXPECT_EQ(l0_link, &joint->GetParentLink());
  EXPECT_EQ(l1_link, &joint->GetChildLink());

  EXPECT_EQ(joint, l0_link->FindChildJoint("q0"));
  EXPECT_EQ(joint, l1_link->GetParentJoint());

  auto& joint_params = joint->GetParameters();
  EXPECT_EQ(Joint::Type::REVOLUTE, joint_params.type);
  EXPECT_EQ(-0.5, joint_params.limits.lower);
  EXPECT_EQ(0.1, joint_params.limits.upper);
  EXPECT_EQ(0.2, joint_params.limits.effort);
  EXPECT_EQ(0.3, joint_params.limits.velocity);
  EXPECT_EQ(0.4, joint_params.limits.acceleration);
  EXPECT_EQ(0.5, joint_params.limits.jerk);
  EXPECT_EQ(0.6, joint_params.dynamics.friction);
  EXPECT_EQ(0.7, joint_params.dynamics.damping);
  EXPECT_EQ(0.8, joint_params.calibration.rising);
  EXPECT_EQ(0.9, joint_params.calibration.falling);
  EXPECT_EQ(1.0, joint_params.calibration.home);
}

TEST(ProtoUtilsTest, FromProtoFixedJointsHaveZeroParams) {
  proto::Assembly assembly_proto = ParseTextProtoOrDie<AssemblyProto>(R"pb(
    links {
      name: "l0"
      collision_shapes {
        link_t_shape: { rw: 1 }
        primitive {
          box {
            size {
              vec: 1.0,
              vec: 1.0,
              vec: 1.0,
            }
          }
        }
      }
    }
    links {
      name: "l1"
      collision_shapes {
        link_t_shape: { rw: 1 }
        primitive {
          box {
            size {
              vec: 1.0,
              vec: 1.0,
              vec: 1.0,
            }
          }
        }
      }
    }
    joints {
      name: "q0"
      type: FIXED
      parent_link_name: "l0"
      child_link_name: "l1"
      parent_t_joint: { rw: 1 }
      axis: { vec: 1 vec: 0 vec: 0 }
      limits {
        lower: -0.5
        upper: 0.1
        effort: 0.2
        velocity: 0.3
        acceleration: 0.4
        jerk: 0.5
      }
      dynamics { friction: 0.6 damping: 0.7 }
      calibration { rising: 0.8 falling: 0.9 home: 1.0 }
    }
  )pb");

  CC_ASSERT_OK_AND_ASSIGN(auto assembly, FromProto(assembly_proto));

  const auto joint = assembly.FindJoint("q0");
  auto& joint_params = joint->GetParameters();

  EXPECT_EQ(Joint::Type::FIXED, joint_params.type);
  EXPECT_EQ(0.0, joint_params.limits.lower);
  EXPECT_EQ(0.0, joint_params.limits.upper);
  EXPECT_EQ(0.0, joint_params.limits.effort);
  EXPECT_EQ(0.0, joint_params.limits.velocity);
  EXPECT_EQ(0.0, joint_params.limits.acceleration);
  EXPECT_EQ(0.0, joint_params.limits.jerk);
  EXPECT_EQ(0.0, joint_params.dynamics.friction);
  EXPECT_EQ(0.0, joint_params.dynamics.damping);
  EXPECT_EQ(0.0, joint_params.calibration.rising);
  EXPECT_EQ(0.0, joint_params.calibration.falling);
  EXPECT_EQ(0.0, joint_params.calibration.home);
}

TEST(ProtoUtilsTest, ToProtoFixedJointsHaveZeroParams) {
  Assembly assembly;
  assembly.CreateLink("l0", Link::Parameters{});
  assembly.CreateLink("l1", Link::Parameters{});

  Joint::Parameters params;
  params.type = Joint::FIXED;
  params.limits.lower = 1;
  params.limits.upper = 2;
  params.limits.effort = 3;
  params.limits.velocity = 4;
  params.limits.acceleration = 5;
  params.limits.jerk = 6;
  params.dynamics.friction = 7;
  params.dynamics.damping = 8;
  params.calibration.rising = 9;
  params.calibration.falling = 10;
  params.calibration.home = 11;
  assembly.CreateJoint("q0", params, "l0", "l1");

  proto::Assembly assembly_proto;
  CC_ASSERT_OK(ToProto(assembly, &assembly_proto));
  ASSERT_EQ(assembly_proto.joints_size(), 1);

  EXPECT_EQ(0.0, assembly_proto.joints(0).limits().lower());
  EXPECT_EQ(0.0, assembly_proto.joints(0).limits().upper());
  EXPECT_EQ(0.0, assembly_proto.joints(0).limits().effort());
  EXPECT_EQ(0.0, assembly_proto.joints(0).limits().velocity());
  EXPECT_EQ(0.0, assembly_proto.joints(0).limits().acceleration());
  EXPECT_EQ(0.0, assembly_proto.joints(0).limits().jerk());
  EXPECT_EQ(0.0, assembly_proto.joints(0).dynamics().friction());
  EXPECT_EQ(0.0, assembly_proto.joints(0).dynamics().damping());
  EXPECT_EQ(0.0, assembly_proto.joints(0).calibration().rising());
  EXPECT_EQ(0.0, assembly_proto.joints(0).calibration().falling());
  EXPECT_EQ(0.0, assembly_proto.joints(0).calibration().home());
}

TEST(ProtoUtilsTest, ToProto) {
  proto::Assembly assembly_proto = ParseTextProtoOrDie<AssemblyProto>(R"pb(
    joints {
      name: "q0"
      parent_link_name: "l0"
      child_link_name: "l1"
      type: REVOLUTE
      parent_t_joint { tx: 0 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
      axis { vec: [ 1, 0, 0 ] }
      limits {
        lower: -0.5
        upper: 0.1
        velocity: 0.3
        acceleration: 0.4
        jerk: 0.5
        effort: 0.2
      }
      dynamics { friction: 0.6 damping: 0.7 }
      calibration { rising: 0.8 falling: 0.9 home: 1 }
    }
    links {
      name: "l0"
      collision_shapes {
        primitive { box { size { vec: 1 vec: 1 vec: 1 } } }
        link_t_shape { tx: 0 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
      }
      inertial {
        link_t_inertial { tx: 0 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
        inertia {
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
        }
        mass: 65.47
      }
    }
    links {
      name: "l1"
      collision_shapes {
        primitive { capsule { length: 1.0 radius: 2.0 } }
        link_t_shape { tx: 1 ty: 2 tz: 3 rx: 0 ry: 0 rz: 0 rw: 1 }
      }
      visual_shapes {
        primitive { box { size { vec: 1 vec: 1 vec: 1 } } }
        link_t_shape { tx: 0 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
      }
      inertial {
        link_t_inertial { tx: 0 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
        inertia {
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
          mat: 1
        }
        mass: 23.08
      }
    }
  )pb");

  CC_ASSERT_OK_AND_ASSIGN(auto assembly, FromProto(assembly_proto));

  proto::Assembly assembly_proto_result;
  EXPECT_EQ(ToProto(assembly, &assembly_proto_result).code(),
            absl::StatusCode::kOk);

  EXPECT_THAT(assembly_proto_result, ProtoIsEquivTo(assembly_proto));
}

}  // namespace
}  // namespace collision_checking
