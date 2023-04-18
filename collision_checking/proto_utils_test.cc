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

#include "collision_checking/proto_utils.h"

#include "collision_checking/assembly/assembly.pb.h"
#include "collision_checking/assembly/proto_utils.h"
#include "collision_checking/assembly_collision_checker.h"
#include "collision_checking/bounding_box.pb.h"
#include "collision_checking/collision_state.pb.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/object_id.h"
#include "collision_checking/options.h"
#include "collision_checking/test_utils.h"
#include "collision_checking/vector.h"
#include "eigenmath/matchers.h"
#include "gtest/gtest.h"

namespace collision_checking {
namespace {
using ::collision_checking::testing::ParseTextProtoOrDie;
using ::collision_checking::testing::ProtoIsEquivTo;
using ::collision_checking::testing::
    ProtoIsEquivToIgnoringRepeatedFieldOrdering;
using ::collision_checking::testing::StatusCodeIs;
using ::eigenmath::testing::IsApprox;

TEST(ProtoUtils, SphereToMarkerProto) {
  // Only tests double precision version.
  using Scalar = double;

  Sphere<Scalar> sphere;
  sphere.radius = 1.2;
  sphere.center = Vector3d(-1, 2, -3);

  geometry_shapes::proto::Marker marker_proto = ToMarkerProto(sphere);

  auto expected_proto = ParseTextProtoOrDie<geometry_shapes::proto::Marker>(
      R"pb(
        pose { tx: -1 ty: 2 tz: -3 rx: 0 ry: 0 rz: 0 rw: 1 }
        shape { sphere { radius: 1.2 } })pb");
  EXPECT_THAT(marker_proto, ProtoIsEquivTo(expected_proto));
}

TEST(ProtoUtils, CapsuleToMarkerProto) {
  // Only tests double precision version.
  using Scalar = double;

  Capsule<Scalar> capsule;
  capsule.radius = 1.2;
  capsule.half_length = 2.0;
  capsule.center = Vector3d(-1, 2, -3);
  capsule.direction = Vector3d(1, 1, 0).normalized();

  geometry_shapes::proto::Marker marker_proto = ToMarkerProto(capsule);

  ASSERT_TRUE(marker_proto.has_pose());
  ASSERT_TRUE(marker_proto.has_shape());
  ASSERT_TRUE(marker_proto.shape().has_capsule());

  constexpr double kError = 1e-6;
  Quaterniond quaternion(marker_proto.pose().rw(), marker_proto.pose().rx(),
                         marker_proto.pose().ry(), marker_proto.pose().rz());

  Vector3d direction_from_unitz = quaternion * Vector3d::UnitZ();

  EXPECT_THAT(direction_from_unitz, IsApprox(capsule.direction, kError));
  EXPECT_NEAR(quaternion.norm(), 1.0, kError);

  EXPECT_NEAR(marker_proto.pose().tx(), -1.0, kError);
  EXPECT_NEAR(marker_proto.pose().ty(), 2.0, kError);
  EXPECT_NEAR(marker_proto.pose().tz(), -3.0, kError);

  EXPECT_DOUBLE_EQ(marker_proto.shape().capsule().length(), 4.0);
  EXPECT_DOUBLE_EQ(marker_proto.shape().capsule().radius(), 1.2);
}

TEST(ProtoUtils, ToBoundingBoxProto) {
  // Only tests double precision version.
  using Scalar = double;
  AlignedBox<Scalar> box;
  box.high << 1, 2, 3;
  box.low << -3, -4, 2;

  const AxisAlignedBoundingBox3dProto box_proto = ToBoundingBoxProto(box);
  const auto expected_proto =
      ParseTextProtoOrDie<AxisAlignedBoundingBox3dProto>(
          R"pb(
            min_corner: { vec: -3 vec: -4 vec: 2 }
            max_corner: { vec: 1 vec: 2 vec: 3 }
          )pb");

  EXPECT_THAT(box_proto, ProtoIsEquivTo(expected_proto));
}

TEST(ProtoUtils, MakeCollisionStateProtoValidCase) {
  // Only tests double precision version.
  using Scalar = double;
  using Checker = AssemblyCollisionChecker<Scalar, DefaultAllocatorTraits>;
  const auto assembly_proto = ParseTextProtoOrDie<proto::Assembly>(
      R"pb(
        joints: {
          name: "joint_0"
          parent_link_name: "link_0"
          child_link_name: "link_1"
          parent_t_joint: { tx: 0 ty: 0 tz: 1 rx: 0 ry: 0 rz: 0 rw: 1 }
          axis: { vec: 1 vec: 0 vec: 0 }
          limits { lower: -3.14 upper: 3.14 }
          type: REVOLUTE
        }

        links: {
          name: "link_0"
          collision_shapes: {
            primitive { sphere { radius: 0.1 } }
            link_t_shape { tx: 0.0 ty: 0.0 tz: 0.0 rw: 1.0 }
          }
        }
        links: {
          name: "link_1"
          collision_shapes: {
            primitive { sphere { radius: 0.1 } }
            link_t_shape { tx: 0.0 ty: 0.0 tz: 0.0 rw: 1.0 }
          }
          collision_shapes: {
            primitive { sphere { radius: 0.1 } }
            link_t_shape { tx: 0.0 ty: 0.85 tz: 0.0 rw: 1.0 }
          }
        }
      )pb");
  auto assembly = FromProto(assembly_proto).value();
  Checker collision;
  ASSERT_THAT(
      collision.AddAssembly(assembly, {"joint_0"},
                            Checker::DefaultOptions().SetAABBPadding(10.0)),
      StatusCodeIs(absl::StatusCode::kOk));

  // A simple environment model.
  VoxelMapObject<Scalar> obstacles;
  obstacles.ResizeBuffers(2);
  obstacles.AddSphere(Vector3<Scalar>(1, 2, 3), 0.02, 0);
  obstacles.AddSphere(Vector3<Scalar>(-2, -3, -4), 0.1, 1);

  collision.DisableCollisionPair("link_0", "link_1");

  QueryOptions options =
      QueryOptions().SetType(QueryOptions::Type::kComputeObjectDistances);

  // No collisions at zero joint angle.
  VectorX<Scalar> joint_values = VectorX<Scalar>::Zero(1);
  typename Checker::Scratch scratch = collision.CreateScratch();
  CollisionResult<Scalar, DefaultAllocatorTraits> result =
      collision.CreateCollisionResult();

  EXPECT_EQ(collision
                .ComputeCollisions(obstacles, Pose3<Scalar>::Identity(),
                                   joint_values, options, scratch, result)
                .code(),
            absl::StatusCode::kOk);

  CC_ASSERT_OK_AND_ASSIGN(
      CollisionStateProto collision_state_proto,
      MakeCollisionStateProto(
          absl::bind_front(&Checker::GetObjectNameFromIdSet, &collision),
          obstacles, scratch.transformed_objects, result));
  auto expected_collision_state_proto =
      ParseTextProtoOrDie<CollisionStateProto>(R"pb(
        collision_objects {
          id_info: {}  # set below
          colliding: false
          marker {
            name: "link_0"
            pose { tx: 0 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
            markers {
              pose { tx: 0 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
              shape { sphere { radius: 0.1 } }
            }
          }
          bounding_box {
            min_corner { vec: -10.1 vec: -10.1 vec: -10.1 }
            max_corner { vec: 10.1 vec: 10.1 vec: 10.1 }
          }
        }
        collision_objects {
          id_info: {}  # set below
          colliding: false
          marker {
            name: "link_1"
            pose { tx: 0 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
            markers {
              pose { tx: 0 ty: 0 tz: 1 rx: 0 ry: 0 rz: 0 rw: 1 }
              shape { sphere { radius: 0.1 } }
            }
            markers {
              pose { tx: 0 ty: 0.85 tz: 1 rx: 0 ry: 0 rz: 0 rw: 1 }
              shape { sphere { radius: 0.1 } }
            }
          }
          bounding_box {
            min_corner { vec: -10.1 vec: -10.1 vec: -9.1 }
            max_corner { vec: 10.1 vec: 10.95 vec: 11.1 }
          }
        }
        collision_objects_bounding_box {
          min_corner { vec: -10.1 vec: -10.1 vec: -10.1 }
          max_corner { vec: 10.1 vec: 10.95 vec: 11.1 }
        }

        environment_map_object_info {
          id_info { id_set: 0xFF00000000000000 inclusion_set: 0xFFFFFFFFFFFFFF }
          spheres {
            name: "obstacles"
            pose { rw: 1 }
            markers {
              pose { tx: 1.0 ty: 2.0 tz: 3.0 rx: 0 ry: 0 rz: 0 rw: 1 }
              shape { sphere { radius: 0.02 } }
            }
            markers {
              pose { tx: -2.0 ty: -3.0 tz: -4.0 rx: 0 ry: 0 rz: 0 rw: 1 }
              shape { sphere { radius: 0.1 } }
            }
          }
          sphere_ids: 0x100000000000000
          sphere_ids: 0x100000000000000
        }
        # Augmented below.
        object_names { id_set: 0x100000000000000 name: "obstacles_0" }
        object_names { id_set: 0x200000000000000 name: "obstacles_1" }
        object_names { id_set: 0x400000000000000 name: "obstacles_2" }
        object_names { id_set: 0x800000000000000 name: "obstacles_3" }
        object_names { id_set: 0x1000000000000000 name: "obstacles_4" }
        object_names { id_set: 0x2000000000000000 name: "obstacles_5" }
        object_names { id_set: 0x4000000000000000 name: "obstacles_6" }
        object_names { id_set: 0x8000000000000000 name: "obstacles_7" }
      )pb");

  ObjectIdSet link_0_set = ObjectIdSet(collision.GetObjectId("link_0"));
  ObjectIdSet link_1_set = ObjectIdSet(collision.GetObjectId("link_1"));
  auto* object_name = expected_collision_state_proto.add_object_names();
  object_name->set_name("link_0");
  object_name->set_id_set(link_0_set.AsSetType());
  object_name = expected_collision_state_proto.add_object_names();
  object_name->set_name("link_1");
  object_name->set_id_set(link_1_set.AsSetType());

  expected_collision_state_proto.mutable_collision_objects(0)
      ->mutable_id_info()
      ->set_id_set(link_0_set.AsSetType());
  expected_collision_state_proto.mutable_collision_objects(0)
      ->mutable_id_info()
      ->set_inclusion_set(ObjectIdSet::kAll()
                              .Remove(link_1_set)
                              .Remove(link_0_set)
                              .AsSetType());

  expected_collision_state_proto.mutable_collision_objects(1)
      ->mutable_id_info()
      ->set_id_set(link_1_set.AsSetType());
  expected_collision_state_proto.mutable_collision_objects(1)
      ->mutable_id_info()
      ->set_inclusion_set(ObjectIdSet::kAll()
                              .Remove(link_1_set)
                              .Remove(link_0_set)
                              .AsSetType());

  EXPECT_THAT(collision_state_proto,
              ProtoIsEquivToIgnoringRepeatedFieldOrdering(
                  expected_collision_state_proto));

  CC_ASSERT_OK_AND_ASSIGN(
      collision_state_proto,
      collision.GetCollisionStateProto(obstacles, scratch, result));
  EXPECT_THAT(collision_state_proto,
              ProtoIsEquivToIgnoringRepeatedFieldOrdering(
                  expected_collision_state_proto));
}

}  // namespace
}  // namespace collision_checking
