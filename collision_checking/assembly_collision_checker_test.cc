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

#include "collision_checking/assembly_collision_checker.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "collision_checking/assembly/proto_utils.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/geometry_shapes/box.h"
#include "collision_checking/id_assigner.h"
#include "collision_checking/object_id.h"
#include "collision_checking/options.h"
#include "collision_checking/test_utils.h"
#include "collision_checking/vector.h"
#include "eigenmath/matchers.h"
#include "gtest/gtest.h"

namespace collision_checking {

namespace {
using ::eigenmath::testing::IsApprox;
using ::testing::Eq;
using ::testing::ExplainMatchResult;
using ::testing::HasSubstr;
using testing::ParseTextProtoOrDie;
using ::testing::Pointwise;
using testing::StatusCodeIs;

template <typename Scalar>
void AddToResultListener(const CompositeObject<Scalar>& object,
                         ::testing::MatchResultListener* result_listener) {
  *result_listener << "{ spheres: ";
  for (const auto& sphere : object.spheres) {
    *result_listener << ToString(sphere);
  }
  *result_listener << "}\n";

  *result_listener << "{ capsules: ";
  for (const auto& capsule : object.capsules) {
    *result_listener << ToString(capsule);
  }
  *result_listener << "}\n";

  *result_listener << "{ boxes: ";
  for (const auto& box : object.boxes) {
    *result_listener << ToString(box);
  }
  *result_listener << "}\n";
}

MATCHER(SphereEq, "") {
  const auto& actual = std::get<0>(arg);
  const auto& expected = std::get<1>(arg);
  return ExplainMatchResult(Eq(expected.radius), actual.radius,
                            result_listener) &&
         ExplainMatchResult(IsApprox(expected.center, 0), actual.center,
                            result_listener);
}
MATCHER(CapsuleEq, "") {
  const auto& actual = std::get<0>(arg);
  const auto& expected = std::get<1>(arg);
  return ExplainMatchResult(Eq(expected.radius), actual.radius,
                            result_listener) &&
         ExplainMatchResult(IsApprox(expected.center, 0), actual.center,
                            result_listener) &&
         ExplainMatchResult(IsApprox(expected.direction, 0), actual.direction,
                            result_listener) &&
         ExplainMatchResult(Eq(expected.half_length), actual.half_length,
                            result_listener);
}
MATCHER(BoxEq, "") {
  const auto& actual = std::get<0>(arg);
  const auto& expected = std::get<1>(arg);
  return ExplainMatchResult(IsApprox(expected.box_rotation_world, 0),
                            actual.box_rotation_world, result_listener) &&
         ExplainMatchResult(IsApprox(expected.half_lengths, 0),
                            actual.half_lengths, result_listener);
}

MATCHER(ObjectEq, "") {
  const auto& actual = std::get<0>(arg);
  const auto& expected = std::get<1>(arg);
  return ExplainMatchResult(Pointwise(SphereEq(), expected.spheres),
                            actual.spheres, result_listener) &&
         ExplainMatchResult(Pointwise(CapsuleEq(), expected.capsules),
                            actual.capsules, result_listener) &&
         ExplainMatchResult(Pointwise(BoxEq(), expected.boxes), actual.boxes,
                            result_listener);
}

MATCHER_P(ObjectMatches, expected, "") {
  *result_listener << "\nexpected:";
  AddToResultListener(expected, result_listener);
  *result_listener << "\nactual:";
  AddToResultListener(arg, result_listener);

  return ExplainMatchResult(Pointwise(SphereEq(), expected.spheres),
                            arg.spheres, result_listener) &&
         ExplainMatchResult(Pointwise(CapsuleEq(), expected.capsules),
                            arg.capsules, result_listener) &&
         ExplainMatchResult(Pointwise(BoxEq(), expected.boxes), arg.boxes,
                            result_listener);
}

MATCHER_P(ObjectsMatch, expected, "") {
  *result_listener << "\nexpected:";
  for (const auto& object : expected.objects) {
    AddToResultListener(object, result_listener);
  }
  *result_listener << "\nactual:";
  for (const auto& object : arg.objects) {
    AddToResultListener(object, result_listener);
  }

  return ExplainMatchResult(Pointwise(ObjectEq(), expected.objects),
                            arg.objects, result_listener);
}

template <typename T>
class AssemblyCollisionCheckerTest : public ::testing::Test {
 public:
  static constexpr T kExpectedPrecision = 1e-6;
};

TYPED_TEST_SUITE_P(AssemblyCollisionCheckerTest);

TYPED_TEST_P(AssemblyCollisionCheckerTest, Options) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;

  constexpr Scalar kExpectedPrecision =
      AssemblyCollisionCheckerTest<Scalar>::kExpectedPrecision;
  typename AssemblyCollisionCheckerCpu::Options default_options =
      AssemblyCollisionCheckerCpu::DefaultOptions();

  EXPECT_NEAR(default_options.GetAABBPadding(), 0.1, kExpectedPrecision);
  EXPECT_EQ(default_options.GetMaskingPolicy(),
            CollisionMaskingPolicy::kNothing);
  EXPECT_EQ(default_options.GetAssemblyPadding(), 0.0);
  typename AssemblyCollisionCheckerCpu::Options options =
      AssemblyCollisionCheckerCpu::DefaultOptions()
          .SetAABBPadding(0.2)
          .SetMaskingPolicy(CollisionMaskingPolicy::kConnectedLinks)
          .SetAssemblyPadding(0.4);
  EXPECT_NEAR(options.GetAABBPadding(), 0.2, kExpectedPrecision);
  EXPECT_NEAR(options.GetAssemblyPadding(), 0.4, kExpectedPrecision);
  EXPECT_EQ(options.GetMaskingPolicy(),
            CollisionMaskingPolicy::kConnectedLinks);
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, AddAssemblyChecksErrors) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;

  // Make a simple assembly from a protbuf, verify it loads correctly.
  const auto proto = ParseTextProtoOrDie<AssemblyProto>(
      R"pb(
        joints: {
          name: "revolute_joint"
          parent_link_name: "link_0"
          child_link_name: "link_1"
          parent_t_joint: { tx: 1 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
          axis: { vec: 0 vec: 0 vec: 1 }
          type: REVOLUTE
        }
        joints: {
          name: "fixed_joint"
          parent_link_name: "link_1"
          child_link_name: "link_2"
          parent_t_joint: { rw: 1 }
          axis: { vec: 0 vec: 0 vec: 0 }
          type: FIXED
        }
        links: { name: "link_0" }
        links: { name: "link_1" }
        links: { name: "link_2" }
      )pb");

  Assembly assembly = FromProto(proto).value();
  std::vector<std::string> joint_order;
  AssemblyCollisionCheckerCpu checker;

  // Wrong joint_order dimension.
  EXPECT_THAT(
      checker.AddAssembly(assembly, joint_order,
                          AssemblyCollisionCheckerCpu::DefaultOptions()),
      StatusCodeIs(absl::StatusCode::kInvalidArgument));

  // Unknown joint name.
  joint_order = {"no_such_joint"};
  EXPECT_THAT(
      checker.AddAssembly(assembly, joint_order,
                          AssemblyCollisionCheckerCpu::DefaultOptions()),
      StatusCodeIs(absl::StatusCode::kInvalidArgument));
  // Referencing fixed joint.
  joint_order = {"fixed_joint"};
  EXPECT_THAT(
      checker.AddAssembly(assembly, joint_order,
                          AssemblyCollisionCheckerCpu::DefaultOptions()),
      StatusCodeIs(absl::StatusCode::kInvalidArgument));

  // Too many links.
  Assembly large_assembly;
  std::vector<std::string> large_assembly_joint_names;
  large_assembly.CreateLink("link_0", Link::Parameters());
  for (int ii = 1; ii < kMaxObjectCount + 1; ii++) {
    large_assembly.CreateLink(absl::Substitute("link_$0", ii),
                              Link::Parameters());

    large_assembly_joint_names.push_back(absl::Substitute("joint_$0", ii));
    large_assembly.CreateJoint(
        large_assembly_joint_names.back(), Joint::Parameters(),
        absl::Substitute("link_$0", ii - 1), absl::Substitute("link_$0", ii));
  }
  EXPECT_EQ(large_assembly.Finalize().code(), absl::StatusCode::kOk);

  // Too many links.
  EXPECT_THAT(
      checker.AddAssembly(large_assembly, large_assembly_joint_names,
                          AssemblyCollisionCheckerCpu::DefaultOptions()),
      StatusCodeIs(absl::StatusCode::kOutOfRange));

  // Too many assemblies.
  joint_order = {"revolute_joint"};
  CC_ASSERT_OK_AND_ASSIGN(
      AssemblyId id,
      checker.AddAssembly(assembly, joint_order,
                          AssemblyCollisionCheckerCpu::DefaultOptions()));
  EXPECT_NE(id, AssemblyIdAssigner::kInvalidId);
  EXPECT_THAT(
      checker.AddAssembly(assembly, joint_order,
                          AssemblyCollisionCheckerCpu::DefaultOptions()),
      StatusCodeIs(absl::StatusCode::kUnimplemented));
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, CreateOK) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;

  constexpr Scalar kExpectedPrecision =
      AssemblyCollisionCheckerTest<Scalar>::kExpectedPrecision;

  // Make a simple assembly from a protbuf, verify it loads correctly.
  const auto proto = ParseTextProtoOrDie<AssemblyProto>(
      R"pb(
        joints: {
          name: "joint_0"
          parent_link_name: "link_0"
          child_link_name: "link_1"
          parent_t_joint: { tx: 1 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
          axis: { vec: 0 vec: 0 vec: 1 }
          type: REVOLUTE
        }
        joints: {
          name: "fixed_joint"
          parent_link_name: "link_1"
          child_link_name: "empty_leaf"
          parent_t_joint: { tx: 1 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
          type: FIXED
        }
        links: {
          name: "link_0"
          collision_shapes: {
            primitive { capsule { length: 0.2 radius: 0.1 } }
            link_t_shape { tx: 0.1 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
          collision_shapes: {
            primitive { sphere { radius: 0.2 } }
            link_t_shape { tx: 0.15 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }
        links: { name: "empty_leaf" }
        links: {
          name: "link_1"
          collision_shapes: {
            primitive { capsule { length: 0.4 radius: 0.01 } }
            link_t_shape { tx: 0.0 ty: 0.1 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
          collision_shapes: {
            primitive { capsule { length: 0.3 radius: 0.01 } }
            link_t_shape { tx: 0.0 ty: 0.1 tz: 1.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
          collision_shapes: {
            primitive { sphere { radius: 0.1 } }
            link_t_shape { tx: -0.15 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }
      )pb");
  std::vector<std::string> joint_order({"joint_0"});

  Assembly assembly = FromProto(proto).value();
  AssemblyCollisionCheckerCpu collision;
  ASSERT_THAT(
      collision.AddAssembly(assembly, joint_order,
                            AssemblyCollisionCheckerCpu::DefaultOptions()),
      StatusCodeIs(absl::StatusCode::kOk));

  const auto& robot = collision.GetObjectsAtZeroConfiguration();

  EXPECT_EQ(robot.objects.size(), 3);

  const int link_0 = collision.GetObjectIndexForName("link_0");
  const int link_1 = collision.GetObjectIndexForName("link_1");
  const int empty_leaf = collision.GetObjectIndexForName("empty_leaf");
  // Link_0 shapes.
  EXPECT_EQ(robot.objects[link_0].capsules.size(), 1);
  EXPECT_EQ(robot.objects[link_0].spheres.size(), 1);
  EXPECT_EQ(robot.objects[link_0].flags.id_set, ObjectIdSet(0b01));
  EXPECT_EQ(robot.objects[link_0].flags.inclusion_set, ~ObjectIdSet(0b01));
  EXPECT_NEAR(robot.objects[link_0].spheres[0].center[0], 0.15,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].spheres[0].center[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].spheres[0].center[2], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].spheres[0].radius, 0.2, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[0], 0.1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[2], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[2], 1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[0], 0.1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[2], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[2], 1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].radius, 0.1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].half_length, 0.1,
              kExpectedPrecision);

  // Link_1 shapes.
  EXPECT_EQ(robot.objects[link_1].capsules.size(), 2);
  EXPECT_EQ(robot.objects[link_1].spheres.size(), 1);
  EXPECT_EQ(robot.objects[link_1].flags.id_set, ObjectIdSet(0b10));
  EXPECT_EQ(robot.objects[link_1].flags.inclusion_set, ~ObjectIdSet(0b10));
  EXPECT_NEAR(robot.objects[link_1].spheres[0].center[0], -0.15,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].spheres[0].center[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].spheres[0].center[2], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].spheres[0].radius, 0.1, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].center[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].center[1], 0.1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].center[2], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].direction[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].direction[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].direction[2], 1.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].radius, 0.01,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].center[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].center[1], 0.1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].center[2], 1.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].direction[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].direction[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].direction[2], 1.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].radius, 0.01,
              kExpectedPrecision);

  EXPECT_EQ(robot.objects[empty_leaf].flags.inclusion_set,
            ObjectIdSet::kEmpty());
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, Attachments) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu =
      AssemblyCollisionChecker<Scalar, DefaultAllocatorTraits>;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;

  // Initial assembly.
  const auto proto = ParseTextProtoOrDie<AssemblyProto>(
      R"pb(
        joints: {
          name: "joint_0"
          parent_link_name: "link_0"
          child_link_name: "link_1"
          parent_t_joint: { tx: 1 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
          axis: { vec: 0 vec: 0 vec: 1 }
          type: REVOLUTE
        }
        links: {
          name: "link_0"
          collision_shapes: {
            primitive { capsule { length: 0.2 radius: 0.1 } }
            link_t_shape { tx: 0.1 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
          collision_shapes: {
            primitive { sphere { radius: 0.1 } }
            link_t_shape { tx: 0.5 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }
        links: {
          name: "link_1"
          collision_shapes: {
            primitive { capsule { length: 0.3 radius: 0.01 } }
            link_t_shape { tx: 0.0 ty: 0.1 tz: 1.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }
      )pb");

  std::vector<std::string> joint_order({"joint_0"});

  Assembly assembly = FromProto(proto).value();
  AssemblyCollisionCheckerCpu collision;
  ASSERT_THAT(
      collision.AddAssembly(assembly, joint_order,
                            AssemblyCollisionCheckerCpu::DefaultOptions()),
      StatusCodeIs(absl::StatusCode::kOk));

  const auto& robot = collision.GetObjectsAtZeroConfiguration();
  EXPECT_EQ(robot.objects.size(), 2);
  auto result = collision.CreateCollisionResult();

  auto scratch = collision.CreateScratch();

  // Make a copy of the initial collision geometry.
  CollisionObjects<Scalar> assembly_objects =
      collision.GetObjectsAtZeroConfiguration();

  // RemoveAllAttachments is a noop if there are none.
  collision.RemoveAllAttachments();
  EXPECT_THAT(robot, ObjectsMatch(assembly_objects));

  CC_MALLOC_COUNTER_INIT();
  Status status = collision.ComputeCollisions(
      VoxelMapObjectCpu(), Pose3<Scalar>::Identity(), VectorN<Scalar>::Zero(1),
      QueryOptions(), scratch, result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);

  // Attach more geometry to a link.
  const auto attachment_proto = ParseTextProtoOrDie<AssemblyProto>(
      R"pb(
        links: {
          name: "attachment"
          collision_shapes: {
            primitive { capsule { length: 0.2 radius: 0.1 } }
            link_t_shape { tx: 0.1 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
          collision_shapes: {
            primitive { sphere { radius: 0.1 } }
            link_t_shape { tx: 0.5 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }
      )pb");
  Assembly attachment_assembly = FromProto(attachment_proto).value();
  std::vector<const geometry_shapes::ShapeBase*> attachment_shapes;
  attachment_shapes.reserve(
      attachment_assembly.GetLink(0).GetCollisionGeometries().size());
  for (const auto& geometry :
       attachment_assembly.GetLink(0).GetCollisionGeometries()) {
    attachment_shapes.push_back(&geometry.GetShape());
  }
  ASSERT_EQ(attachment_assembly.GetLinkCount(), 1);
  ASSERT_EQ(attachment_assembly.GetLink(0).GetCollisionGeometries().size(), 2);
  //  Expect attachment to invalid id to fail.
  EXPECT_THAT(
      collision.AddRelativeObject("object_name", ObjectId{0xdeadbeaf},
                                  /*reference_pose_object=*/Pose3<Scalar>(),
                                  /*shapes=*/
                                  attachment_shapes,
                                  /*padding=*/Scalar{0}),
      StatusCodeIs(absl::StatusCode::kFailedPrecondition));

  // Expect attachment to a valid index to succeed.
  const ObjectId link_1_id = collision.GetObjectId("link_1");
  ASSERT_NE(link_1_id, kInvalidObjectId);
  CC_ASSERT_OK_AND_ASSIGN(
      const ObjectId link_1_attachment_id,
      collision.AddRelativeObject("link_1_attachment", link_1_id,
                                  /*reference_pose_object=*/Pose3<Scalar>(),
                                  /*shapes=*/
                                  attachment_shapes,
                                  /*padding=*/Scalar{0}));
  EXPECT_NE(link_1_attachment_id, kInvalidObjectId);

  // ComputeCollisions should fail if buffers have not been resized.
  EXPECT_EQ(collision
                .ComputeCollisions(
                    VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                    VectorN<Scalar>::Zero(1), QueryOptions(), scratch, result)
                .code(),
            absl::StatusCode::kInvalidArgument);

  // Compute collisions should work after resizing buffers.
  collision.ResizeScratch(scratch);
  collision.ResizeCollisionResult(result);
  EXPECT_EQ(collision
                .ComputeCollisions(
                    VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                    VectorN<Scalar>::Zero(1), QueryOptions(), scratch, result)
                .code(),
            absl::StatusCode::kOk);

  // After clearing attachments, the collision model should be what we started
  // out with.
  collision.RemoveAllAttachments();
  EXPECT_THAT(robot, ObjectsMatch(assembly_objects));

  // Compute collisions should fail if buffers have not been resized.
  EXPECT_EQ(collision
                .ComputeCollisions(
                    VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                    VectorN<Scalar>::Zero(1), QueryOptions(), scratch, result)
                .code(),
            absl::StatusCode::kInvalidArgument);

  // Compute collisions should work after resizing buffers.
  collision.ResizeScratch(scratch);
  collision.ResizeCollisionResult(result);
  EXPECT_EQ(collision
                .ComputeCollisions(
                    VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                    VectorN<Scalar>::Zero(1), QueryOptions(), scratch, result)
                .code(),
            absl::StatusCode::kOk);
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, IndexIdNameUtilities) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;

  const auto proto = ParseTextProtoOrDie<AssemblyProto>(
      R"pb(
        name: "robot"
        joints: {
          name: "joint_0"
          parent_link_name: "link_0"
          child_link_name: "link_1"
          parent_t_joint: { rw: 1 }
          axis: { vec: 0 vec: 0 vec: 1 }
          type: REVOLUTE
        }
        joints: {
          name: "joint_1"
          parent_link_name: "link_1"
          child_link_name: "link_2"
          parent_t_joint: { rw: 1 }
          axis: { vec: 0 vec: 0 vec: 1 }
          type: REVOLUTE
        }
        links: { name: "link_0" }
        links: { name: "link_1" }
        links: { name: "link_2" }
      )pb");
  std::vector<std::string> joint_order({"joint_0", "joint_1"});

  Assembly assembly = FromProto(proto).value();
  AssemblyCollisionCheckerCpu collision;
  CC_ASSERT_OK_AND_ASSIGN(
      AssemblyId assembly_id,
      collision.AddAssembly(assembly, joint_order,
                            AssemblyCollisionCheckerCpu::DefaultOptions()));

  EXPECT_TRUE(collision.IsValidAssembly(assembly_id));
  EXPECT_FALSE(collision.IsValidAssembly(assembly_id + AssemblyId{1}));

  // Id / name conversions.
  {
    CC_MALLOC_COUNTER_INIT();
    absl::string_view name = collision.GetAssemblyName(assembly_id);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();
    EXPECT_EQ(name, "robot");
  }

  {
    CC_MALLOC_COUNTER_INIT();
    absl::string_view name =
        collision.GetAssemblyName(assembly_id + AssemblyId{1});
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();
    EXPECT_TRUE(name.empty());
  }

  {
    CC_MALLOC_COUNTER_INIT();
    AssemblyId id = collision.GetAssemblyId("no_such_assembly");
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();
    EXPECT_EQ(id, AssemblyIdAssigner::kInvalidId);
  }

  {
    CC_MALLOC_COUNTER_INIT();
    AssemblyId id = collision.GetAssemblyId("robot");
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();
    EXPECT_EQ(id, assembly_id);
  }

  {
    CC_MALLOC_COUNTER_INIT();
    const ObjectId id = collision.GetObjectId("No_Such_Link");
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();
    EXPECT_EQ(id, kInvalidObjectId);
  }

  for (const auto& link : assembly.GetLinks()) {
    CC_MALLOC_COUNTER_INIT();
    ObjectId id = collision.GetObjectId(link.GetName());
    absl::string_view name = collision.GetObjectName(id);
    ObjectId id_round_trip = collision.GetObjectId(name);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();
    EXPECT_EQ(id, id_round_trip);
    EXPECT_EQ(name, link.GetName());
    EXPECT_TRUE(collision.IsValidObject(id));
  }

  // Names from bitmask.
  std::vector<std::string> names_from_bitmask =
      collision.GetObjectNamesFromSet(ObjectIdSet(0x0));
  EXPECT_TRUE(names_from_bitmask.empty());
  names_from_bitmask = collision.GetObjectNamesFromSet(ObjectIdSet(0b10));

  ASSERT_EQ(names_from_bitmask.size(), 1);
  EXPECT_EQ(names_from_bitmask[0], "link_1");
  names_from_bitmask =
      collision.GetObjectNamesFromSet(kVoxelMapIdSet | ObjectIdSet(0b01));
  ASSERT_EQ(names_from_bitmask.size(), 2);
  EXPECT_EQ(names_from_bitmask[0], "link_0");
  EXPECT_EQ(names_from_bitmask[1], kVoxelMapObjectName);

  names_from_bitmask = collision.GetObjectNamesFromSet(
      kVoxelMapIdSet | ObjectIdSet(0b01) | ObjectIdSet(0b1000000000));
  ASSERT_EQ(names_from_bitmask.size(), 3);
  EXPECT_EQ(names_from_bitmask[0], "link_0");
  EXPECT_EQ(names_from_bitmask[1], kVoxelMapObjectName);
  EXPECT_THAT(names_from_bitmask[2], HasSubstr("unknown"));

  // Bitmask from names.
  std::vector<std::string> link_names = {"link_1", "link_0"};
  ObjectIdSet set_from_link_names;
  EXPECT_EQ(
      collision.GetObjectSetFromObjectNames(link_names, set_from_link_names)
          .code(),
      absl::StatusCode::kOk);
  EXPECT_EQ(set_from_link_names, ObjectIdSet(0b11));
  link_names.push_back("invalid_name");
  EXPECT_EQ(
      collision.GetObjectSetFromObjectNames(link_names, set_from_link_names)
          .code(),
      absl::StatusCode::kInvalidArgument);
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, AssemblyPadding) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;

  constexpr Scalar kExpectedPrecision =
      AssemblyCollisionCheckerTest<Scalar>::kExpectedPrecision;

  // Make a simple assembly from a protbuf, verify it loads correctly.
  const auto proto = ParseTextProtoOrDie<AssemblyProto>(
      R"pb(
        joints: {
          name: "joint_0"
          parent_link_name: "link_0"
          child_link_name: "link_1"
          parent_t_joint: { rw: 1 }
          axis: { vec: [ 1, 0, 0 ] }
          type: REVOLUTE
        }
        links: {
          name: "link_0"
          collision_shapes: {
            primitive { capsule { length: 0.2 radius: 0.1 } }
            link_t_shape { tx: 0.1 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
          collision_shapes: {
            primitive { sphere { radius: 0.2 } }
            link_t_shape { tx: 0.15 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }
        links: {
          name: "link_1"
          collision_shapes: {
            primitive { capsule { length: 0.4 radius: 0.01 } }
            link_t_shape { tx: 0.0 ty: 0.1 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
          collision_shapes: {
            primitive { capsule { length: 0.3 radius: 0.01 } }
            link_t_shape { tx: 0.0 ty: 0.1 tz: 1.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
          collision_shapes: {
            primitive { sphere { radius: 0.1 } }
            link_t_shape { tx: -0.15 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }
      )pb");
  std::vector<std::string> joint_order({"joint_0"});

  Assembly assembly = FromProto(proto).value();
  AssemblyCollisionCheckerCpu collision;
  ASSERT_THAT(
      collision.AddAssembly(
          assembly, joint_order,
          AssemblyCollisionCheckerCpu::DefaultOptions().SetAssemblyPadding(
              0.02)),
      StatusCodeIs(absl::StatusCode::kOk));

  const auto& robot = collision.GetObjectsAtZeroConfiguration();

  EXPECT_EQ(robot.objects.size(), 2);

  const int link_0 = collision.GetObjectIndexForName("link_0");
  const int link_1 = collision.GetObjectIndexForName("link_1");

  // Link_0 shapes.
  EXPECT_EQ(robot.objects[link_0].capsules.size(), 1);
  EXPECT_EQ(robot.objects[link_0].spheres.size(), 1);
  EXPECT_EQ(robot.objects[link_0].flags.id_set, ObjectIdSet(0b01));
  EXPECT_EQ(robot.objects[link_0].flags.inclusion_set, ~ObjectIdSet(0b01));
  EXPECT_NEAR(robot.objects[link_0].spheres[0].center[0], 0.15,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].spheres[0].center[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].spheres[0].center[2], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].spheres[0].radius, 0.22,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[0], 0.1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[2], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[2], 1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[0], 0.1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].center[2], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].direction[2], 1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].radius, 0.12,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_0].capsules[0].half_length, 0.1,
              kExpectedPrecision);

  // Link_1 shapes.
  EXPECT_EQ(robot.objects[link_1].capsules.size(), 2);
  EXPECT_EQ(robot.objects[link_1].spheres.size(), 1);
  EXPECT_EQ(robot.objects[link_1].flags.id_set, ObjectIdSet(0b10));
  EXPECT_EQ(robot.objects[link_1].flags.inclusion_set, ~ObjectIdSet(0b10));
  EXPECT_NEAR(robot.objects[link_1].spheres[0].center[0], -0.15,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].spheres[0].center[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].spheres[0].center[2], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].spheres[0].radius, 0.12,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].center[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].center[1], 0.1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].center[2], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].direction[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].direction[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].direction[2], 1.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[0].radius, 0.03,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].center[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].center[1], 0.1,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].center[2], 1.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].direction[0], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].direction[1], 0.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].direction[2], 1.0,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].radius, 0.03,
              kExpectedPrecision);
  EXPECT_NEAR(robot.objects[link_1].capsules[1].half_length, 0.15,
              kExpectedPrecision);
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, ComputeCollisionsErrors) {
  using Scalar = TypeParam;
  using CollisionResultCpu = CollisionResult<Scalar>;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;
  const auto proto = ParseTextProtoOrDie<AssemblyProto>(
      R"pb(
        joints: {
          name: "joint_0"
          parent_link_name: "link_0"
          child_link_name: "link_1"
          type: REVOLUTE
          parent_t_joint: { rw: 1 }
          axis: { vec: [ 1, 0, 0 ] }
        }

        links: { name: "link_0" }
        links: { name: "link_1" }
      )pb");
  auto assembly = FromProto(proto).value();
  AssemblyCollisionCheckerCpu collision;
  ASSERT_THAT(collision
                  .AddAssembly(assembly, {"joint_0"},
                               AssemblyCollisionCheckerCpu::DefaultOptions())
                  .status()
                  .code(),
              absl::StatusCode::kOk);
  typename AssemblyCollisionCheckerCpu::Scratch scratch =
      collision.CreateScratch();
  CollisionResultCpu result = collision.CreateCollisionResult();
  Status status;
  VectorN<Scalar> zero_vec = VectorN<Scalar>::Zero(10);

  // Wrong joint vector size.
  CC_MALLOC_COUNTER_INIT();
  status = collision.ComputeCollisions(VoxelMapObjectCpu(),
                                       Pose3<Scalar>::Identity(), zero_vec,
                                       QueryOptions(), scratch, result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kInvalidArgument);

  // Wrong scratch struct size.
  scratch.Resize(10);
  zero_vec.resize(1);

  CC_MALLOC_COUNTER_INIT();
  status = collision.ComputeCollisions(VoxelMapObjectCpu(),
                                       Pose3<Scalar>::Identity(), zero_vec,
                                       QueryOptions(), scratch, result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();
  EXPECT_EQ(status.code(), absl::StatusCode::kInvalidArgument);

  // Wrong result struct size (number of bodies doesn't match system).
  scratch = collision.CreateScratch();
  result.Allocate(10);
  CC_MALLOC_COUNTER_INIT();
  status = collision.ComputeCollisions(VoxelMapObjectCpu(),
                                       Pose3<Scalar>::Identity(), zero_vec,
                                       QueryOptions(), scratch, result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kInvalidArgument);
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, ComputeCollisionsSphereSphere) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;
  using CollisionResultCpu = CollisionResult<Scalar>;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;

  constexpr Scalar kExpectedPrecision =
      AssemblyCollisionCheckerTest<Scalar>::kExpectedPrecision;

  // Two links with three spheres, one pair in collision, the other not.
  // At zero pose:
  // O==O
  // |
  // O
  // At ~-1.3 rad, before collision:
  // O
  // |\
  // O O
  // At -pi the two lower spheres overlap.
  const auto proto = ParseTextProtoOrDie<AssemblyProto>(
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
            link_t_shape { tx: 0.0 ty: 0.0 tz: 0.0 rw: 1 }
          }
        }
        links: {
          name: "link_1"
          # this doesn't collide
          collision_shapes: {
            primitive { sphere { radius: 0.1 } }
            link_t_shape { tx: 0.0 ty: 0.0 tz: 0.0 rw: 1 }
          }
          # this collides at at joint angle pi/2.
          collision_shapes: {
            primitive { sphere { radius: 0.1 } }
            link_t_shape { tx: 0.0 ty: 0.85 tz: 0.0 rw: 1 }
          }
        }
      )pb");
  auto assembly = FromProto(proto).value();
  AssemblyCollisionCheckerCpu collision;
  ASSERT_THAT(
      collision.AddAssembly(
          assembly, {"joint_0"},
          AssemblyCollisionCheckerCpu::DefaultOptions().SetAABBPadding(10.0)),
      StatusCodeIs(absl::StatusCode::kOk));

  // An empty environment model.
  VoxelMapObjectCpu obstacles;
  QueryOptions options =
      QueryOptions().SetType(QueryOptions::Type::kComputeObjectDistances);

  // No collisions at zero joint angle.
  VectorN<Scalar> joint_values = VectorN<Scalar>::Zero(1);
  typename AssemblyCollisionCheckerCpu::Scratch scratch =
      collision.CreateScratch();
  CollisionResultCpu result = collision.CreateCollisionResult();
  Status status;
  CC_MALLOC_COUNTER_INIT();
  status = collision.ComputeCollisions(obstacles, Pose3<Scalar>::Identity(),
                                       joint_values, options, scratch, result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();
  EXPECT_EQ(status.code(), absl::StatusCode::kOk);

  EXPECT_FALSE(result.GetHasCollisions());
  const int link_0 = collision.GetObjectIndexForName("link_0");
  const int link_1 = collision.GetObjectIndexForName("link_1");
  EXPECT_EQ(result.GetObjectHits(link_0), ObjectIdSet::kEmpty());
  EXPECT_EQ(result.GetObjectHits(link_1), ObjectIdSet::kEmpty());
  EXPECT_NEAR(result.GetMinimumDistance(link_0), 0.8, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(link_1), 0.8, kExpectedPrecision);

  // Expect collision at -pi/2 joint angle.
  joint_values[0] = -M_PI_2;
  CC_MALLOC_COUNTER_INIT();
  status = collision.ComputeCollisions(obstacles, Pose3<Scalar>::Identity(),
                                       joint_values, options, scratch, result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();
  EXPECT_EQ(status.code(), absl::StatusCode::kOk);

  EXPECT_TRUE(result.GetHasCollisions());
  EXPECT_EQ(result.GetObjectHits(link_0), ObjectIdSet(0b10));
  EXPECT_EQ(result.GetObjectHits(link_1), ObjectIdSet(0b01));

  EXPECT_NEAR(result.GetMinimumDistance(link_0), -0.05, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(link_1), -0.05, kExpectedPrecision);
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, ComputeCollisionsCapsuleSphere) {
  using Scalar = TypeParam;
  using CollisionResultCpu = CollisionResult<Scalar>;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;

  constexpr Scalar kExpectedPrecision =
      AssemblyCollisionCheckerTest<Scalar>::kExpectedPrecision;

  struct CollisionTestCase {
    AssemblyProto proto;
    bool has_collisions;
    ObjectIdSet link0_hits;
    Scalar distance;
  };

  const auto link0_proto = ParseTextProtoOrDie<AssemblyProto>(
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
            primitive { capsule { length: 1.0 radius: 0.1 } }
            link_t_shape {
              tx: 0.0
              ty: 0.0
              tz: 0.0
              rx: 0.70711
              ry: 0
              rz: 0
              rw: 0.70711
            }
          }
        })pb");
  std::vector<CollisionTestCase> test_data = {
      {ParseTextProtoOrDie<AssemblyProto>(
           R"pb(
             links: {
               name: "link_1"
               # closest point on line segment, no collision.
               collision_shapes: {
                 primitive { sphere { radius: 0.1 } }
                 link_t_shape { tx: 0.0 ty: 0.0 tz: 0.0 rw: 1 }
               }
             }
           )pb"),
       false, ObjectIdSet::kEmpty(), Scalar(0.8)},
      {ParseTextProtoOrDie<AssemblyProto>(
           R"pb(
             links: {
               name: "link_1"
               # closest point on line segment, collision.
               collision_shapes: {
                 primitive { sphere { radius: 0.1 } }
                 link_t_shape { tx: -0.0 ty: 0.0 tz: -0.9 rw: 1 }
               }
             }
           )pb"),
       true, ObjectIdSet(0b10), Scalar(-0.1)},
      {
          ParseTextProtoOrDie<AssemblyProto>(
              R"pb(
                links: {
                  name: "link_1"
                  # closest point is first endpoint, no collision.
                  collision_shapes: {
                    primitive { sphere { radius: 0.1 } }
                    link_t_shape { tx: 0.0 ty: -1.0 tz: 0.0 rw: 1 }
                  }
                }
              )pb"),
          false,
          ObjectIdSet::kEmpty(),
          Scalar(std::sqrt(0.5 * 0.5 + 1.0) - 0.2),
      },
      {ParseTextProtoOrDie<AssemblyProto>(
           R"pb(
             links: {
               name: "link_1"
               # closest point is first endpoint, collision.
               collision_shapes: {
                 primitive { sphere { radius: 0.1 } }
                 link_t_shape { tx: -0.0 ty: -0.6 tz: -0.9 rw: 1 }
               }
             }
           )pb"),
       true, ObjectIdSet(0b10), Scalar(std::sqrt(0.02) - 0.2)},
      {
          ParseTextProtoOrDie<AssemblyProto>(
              R"pb(
                links: {
                  name: "link_1"
                  # closest point is second endpoint, no collision.
                  collision_shapes: {
                    primitive { sphere { radius: 0.1 } }
                    link_t_shape { tx: 0.0 ty: 1.0 tz: 0.0 rw: 1 }
                  }
                }
              )pb"),
          false,
          ObjectIdSet::kEmpty(),
          Scalar(std::sqrt(0.5 * 0.5 + 1.0) - 0.2),
      },
      {ParseTextProtoOrDie<AssemblyProto>(
           R"pb(
             links: {
               name: "link_1"
               # closest point is second endpoint, collision.
               collision_shapes: {
                 primitive { sphere { radius: 0.1 } }
                 link_t_shape { tx: -0.0 ty: 0.6 tz: -0.9 rw: 1 }
               }
             }
           )pb"),
       true, ObjectIdSet(0b10), Scalar(std::sqrt(0.02) - 0.2)},
  };

  QueryOptions query_options =
      QueryOptions().SetType(QueryOptions::Type::kComputeObjectDistances);

  for (auto& d : test_data) {
    d.proto.MergeFrom(link0_proto);
    auto assembly = FromProto(d.proto).value();

    AssemblyCollisionCheckerCpu collision;
    ASSERT_THAT(
        collision.AddAssembly(
            assembly, {"joint_0"},
            AssemblyCollisionCheckerCpu::DefaultOptions().SetAABBPadding(10.0)),
        StatusCodeIs(absl::StatusCode::kOk));
    VectorN<Scalar> joint_values = VectorN<Scalar>::Zero(1);
    typename AssemblyCollisionCheckerCpu::Scratch scratch =
        collision.CreateScratch();
    CollisionResultCpu result = collision.CreateCollisionResult();
    CC_MALLOC_COUNTER_INIT();
    Status status = collision.ComputeCollisions(
        VoxelMapObjectCpu(), Pose3<Scalar>::Identity(), joint_values,
        query_options, scratch, result);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();
    EXPECT_EQ(status.code(), absl::StatusCode::kOk);

    const int link_0 = collision.GetObjectIndexForName("link_0");
    EXPECT_EQ(result.GetHasCollisions(), d.has_collisions);
    EXPECT_EQ(result.GetObjectHits(link_0), d.link0_hits);
    EXPECT_NEAR(result.GetMinimumDistance(link_0), d.distance,
                kExpectedPrecision);
  }
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, ComputeCollisionsCapsuleCapsule) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;
  using CollisionResultCpu = CollisionResult<Scalar>;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;

  constexpr Scalar kExpectedPrecision =
      AssemblyCollisionCheckerTest<Scalar>::kExpectedPrecision;

  struct CollisionTestCase {
    std::string name;
    AssemblyProto proto;
    bool has_collisions;
    ObjectIdSet link0_hits;
    Scalar distance;
  };

  // Test cases
  // 1 Non parallel
  // 1.1 in segment, in segment
  // 1.2 in segment, point0
  // 1.3 in segment, point1
  // 1.4 point0, in segment
  // 1.5 point1, in segment
  // 1.6 point0, point0
  // 1.7 point0, point1
  // 1.8 point1, point0
  // 1.9 point1, point1
  // 2 Parallel
  // 2.1 overlap, distance = dist(line,line)
  // 2.2 no overlap, point0, point0
  // 2.3 no overlap, point1, point0
  // 2.4 no overlap, point0, point1
  // 2.5 no overlap, point1, point1

  const auto link0_proto = ParseTextProtoOrDie<AssemblyProto>(R"pb(
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
        primitive { capsule { length: 1.0 radius: 0.1 } }
        link_t_shape {
          tx: 0.0
          ty: 0.0
          tz: 0.0
          rx: 0.70711
          ry: 0
          rz: 0
          rw: 0.70711
        }
      }
    })pb");

  // Reference distances computed with Mathematica's Minimize function.
  std::vector<CollisionTestCase> test_data = {
      {"non parallel, no collision",
       ParseTextProtoOrDie<AssemblyProto>(
           R"pb(links: {
                  name: "link_1"
                  collision_shapes: {
                    primitive { capsule { length: 1.0 radius: 0.1 } }
                    link_t_shape {
                      tx: 0.1
                      ty: 0.3
                      tz: 0.0
                      rx: 0
                      ry: 0.70711
                      rz: 0
                      rw: 0.70711
                    }
                  }
                })pb"),
       false, ObjectIdSet::kEmpty(), 0.8},
      {"non parallel, collision.",
       ParseTextProtoOrDie<AssemblyProto>(
           R"pb(links: {
                  name: "link_1"
                  collision_shapes: {
                    primitive { capsule { length: 1.0 radius: 0.1 } }
                    link_t_shape {
                      tx: -0.1
                      ty: -0.1
                      tz: -0.85
                      rx: 0
                      ry: 0.70711
                      rz: 0
                      rw: 0.70711
                    }
                  }
                })pb"),
       true, ObjectIdSet(0b10), -0.05},

      {
          "non parallel, no collision.",
          ParseTextProtoOrDie<AssemblyProto>(R"pb(
            links: {
              name: "link_1"
              collision_shapes: {
                primitive { capsule { length: 1.0 radius: 0.1 } }
                link_t_shape {
                  tx: 0.0
                  ty: -1.0
                  tz: 0.0
                  rx: 0
                  ry: 0.70711
                  rz: 0
                  rw: 0.70711
                }
              }
            })pb"),
          false,
          ObjectIdSet::kEmpty(),
          0.918033972422929,
      },
      {"non parallel, collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: 0.1
               ty: -0.6
               tz: -0.85
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       true, ObjectIdSet(0b10), -0.01972245905176942},
      {"non parallel, no collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: -0.1
               ty: 1.0
               tz: 0.0
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 0.9180340310615753},
      {"non parallel, collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: 0.1
               ty: 0.6
               tz: -0.85
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       true, ObjectIdSet(0b10), -0.019722409461582963},
      {"non parallel, no collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: 1.0
               ty: -1
               tz: -1.85
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 0.9056673074419832},
      {"non parallel, collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: 0.55
               ty: -0.55
               tz: -1.15
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       true, ObjectIdSet(0b10), -0.03416870595386107},
      {"non parallel, no collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: -1.0
               ty: -1
               tz: -1.85
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 0.905667307441983},
      {"non parallel, collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: -0.55
               ty: -0.55
               tz: -1.15
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       true, ObjectIdSet(0b10), -0.03416870595386107},
      {"non parallel, no collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: 1.0
               ty: 1
               tz: -1.85
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 0.905667307441983},
      {"non parallel, collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: 0.55
               ty: 0.55
               tz: -1.15
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       true, ObjectIdSet(0b10), -0.03416870595386107},
      {"non parallel, no collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: -1.0
               ty: 1
               tz: -1.85
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 0.905667307441983},
      {"non parallel, collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: -0.55
               ty: 0.55
               tz: -1.15
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       true, ObjectIdSet(0b10), -0.03416870595386107},
      {"non parallel, no collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: 2
               ty: 0.3
               tz: -1.15
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 1.3074813686381375},
      {"non parallel, collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: -2
               ty: -0.3
               tz: -1.15
               rx: 0
               ry: 0.70711
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 1.3074813686381375},
      {"non parallel, no collision.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 1.0 radius: 0.1 } }
             link_t_shape {
               tx: 0.3
               ty: 0.4
               tz: -1
               rx: 0.154
               ry: 0.19
               rz: -0.21
               rw: 0.946
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 0.08443631208535382},
      {" parallel, with overlap.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 0.6 radius: 0.1 } }
             link_t_shape {
               tx: 0.1
               ty: 0.4
               tz: 0.0
               rx: 0.70711
               ry: 0.0
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 0.8049875355396545},
      {"parallel, with overlap.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 0.6 radius: 0.1 } }
             link_t_shape {
               tx: -0.1
               ty: -0.4
               tz: 0.0
               rx: 0.70711
               ry: 0.0
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 0.8049875355396545},
      {"parallel, no overlap.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 0.6 radius: 0.1 } }
             link_t_shape {
               tx: -0.1
               ty: -2
               tz: -1.3
               rx: 0.70711
               ry: 0.0
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 1.0409673732609603},
      {"parallel, no overlap.", ParseTextProtoOrDie<AssemblyProto>(R"pb(
         links: {
           name: "link_1"
           collision_shapes: {
             primitive { capsule { length: 0.6 radius: 0.1 } }
             link_t_shape {
               tx: -0.1
               ty: 2
               tz: -1.3
               rx: 0.70711
               ry: 0.0
               rz: 0
               rw: 0.70711
             }
           }
         })pb"),
       false, ObjectIdSet::kEmpty(), 1.0409673732609603},
  };

  QueryOptions query_options =
      QueryOptions().SetType(QueryOptions::Type::kComputeObjectDistances);

  for (auto& d : test_data) {
    d.proto.MergeFrom(link0_proto);
    SCOPED_TRACE(::testing::Message()
                 << "name= " << d.name
                 << "merged assembly_proto= " << d.proto.DebugString());
    auto assembly = FromProto(d.proto).value();

    AssemblyCollisionCheckerCpu collision;
    ASSERT_THAT(
        collision.AddAssembly(
            assembly, {"joint_0"},
            AssemblyCollisionCheckerCpu::DefaultOptions().SetAABBPadding(10.0)),
        StatusCodeIs(absl::StatusCode::kOk));
    VectorN<Scalar> joint_values = VectorN<Scalar>::Zero(1);
    typename AssemblyCollisionCheckerCpu::Scratch scratch =
        collision.CreateScratch();
    CollisionResultCpu result = collision.CreateCollisionResult();
    CC_MALLOC_COUNTER_INIT();
    Status status = collision.ComputeCollisions(
        VoxelMapObjectCpu(), Pose3<Scalar>::Identity(), joint_values,
        query_options, scratch, result);
    CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

    const int link_0 = collision.GetObjectIndexForName("link_0");
    EXPECT_EQ(status.code(), absl::StatusCode::kOk);
    EXPECT_EQ(result.GetHasCollisions(), d.has_collisions);
    EXPECT_EQ(result.GetObjectHits(link_0), d.link0_hits);

    ASSERT_NEAR(result.GetMinimumDistance(link_0), d.distance,
                kExpectedPrecision);
  }
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, ComputeCollisionsBox) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;
  using CollisionResultCpu = CollisionResult<Scalar>;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;

  // Two links with a box, sphere & capsule in the first and a box in the
  // second. Depending on the joint angle, the minimum distance is box/box,
  // box/sphere or box/capsule.
  const auto proto = ParseTextProtoOrDie<AssemblyProto>(
      R"pb(
        joints: {
          name: "joint_0"
          parent_link_name: "link_0"
          child_link_name: "link_1"
          parent_t_joint: { tx: 1 ty: 0 tz: 0 rx: 0 ry: 0 rz: 0 rw: 1 }
          axis: { vec: [ 0, 1, 0 ] }
          limits { lower: -3.14 upper: 3.14 }
          type: REVOLUTE
        }
        links: {
          name: "link_0"
          collision_shapes: {
            primitive { capsule { length: 1.0 radius: 0.1 } }
            link_t_shape { tx: 0.0 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
          collision_shapes: {
            primitive { sphere { radius: 0.2 } }
            link_t_shape { tx: 0.0 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
          collision_shapes: {
            primitive { box: { size: { vec: [ 0.2, 0.2, 0.2 ] } } }
            link_t_shape { tx: 0.0 ty: 0.0 tz: -1.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }

        links: {
          name: "link_1"
          collision_shapes: {
            primitive { box: { size: { vec: [ 0.2, 0.2, 0.2 ] } } }
            link_t_shape { tx: 1.0 ty: 0.0 tz: 0.0 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }
      )pb");
  auto assembly = FromProto(proto).value();

  AssemblyCollisionCheckerCpu collision;
  ASSERT_THAT(
      collision.AddAssembly(
          assembly, {"joint_0"},
          AssemblyCollisionCheckerCpu::DefaultOptions().SetAABBPadding(10.0)),
      StatusCodeIs(absl::StatusCode::kOk));

  typename AssemblyCollisionCheckerCpu::Scratch scratch =
      collision.CreateScratch();
  CollisionResultCpu result = collision.CreateCollisionResult();
  Status status;

  // At joint angle 0, the minimum distance should be box/sphere.
  CC_MALLOC_COUNTER_INIT();
  status = collision.ComputeCollisions(
      VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
      VectorN<Scalar>({{Scalar{0.0}}}),
      QueryOptions().SetType(QueryOptions::Type::kComputeObjectDistances),
      scratch, result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  const int link_0 = collision.GetObjectIndexForName("link_0");
  const int link_1 = collision.GetObjectIndexForName("link_1");
  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_TRUE(result.GetObjectHits(link_0).Empty());
  EXPECT_TRUE(result.GetObjectHits(link_1).Empty());
  EXPECT_NEAR(result.GetMinimumDistance(), Scalar{2.0 - 0.2 - 0.1},
              std::numeric_limits<Scalar>::epsilon() * 10);

  // At joint angle -pi/2, the minimum distance should be box/capsule.
  CC_MALLOC_COUNTER_INIT();
  status = collision.ComputeCollisions(
      VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
      VectorN<Scalar>({{Scalar{-M_PI_2}}}),
      QueryOptions().SetType(QueryOptions::Type::kComputeObjectDistances),
      scratch, result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_TRUE(result.GetObjectHits(link_0).Empty());
  EXPECT_TRUE(result.GetObjectHits(link_1).Empty());
  const Vector3<Scalar> capsule_tip(0, 0, 0.5);
  const Vector3<Scalar> box_corner(0.9, 0, 0.9);
  const Scalar expected_distance_capsule_box =
      (capsule_tip - box_corner).norm() - Scalar{0.1};
  EXPECT_NEAR(result.GetMinimumDistance(), expected_distance_capsule_box,
              std::numeric_limits<Scalar>::epsilon() * 100);

  // At joint angle pi/2, the minimum distance should be box/box.
  CC_MALLOC_COUNTER_INIT();
  status = collision.ComputeCollisions(
      VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
      VectorN<Scalar>({{Scalar{M_PI_2}}}),
      QueryOptions().SetType(QueryOptions::Type::kComputeObjectDistances),
      scratch, result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_TRUE(result.GetObjectHits(link_0).Empty());
  EXPECT_TRUE(result.GetObjectHits(link_1).Empty());
  const Vector3<Scalar> link_0_box_face_center(0.1, 0, -1.0);
  const Vector3<Scalar> link_1_box_face_center(0.9, 0, -1.0);
  const Scalar expected_distance_box_box =
      (link_0_box_face_center - link_1_box_face_center).norm();
  EXPECT_NEAR(result.GetMinimumDistance(), expected_distance_box_box,
              std::numeric_limits<Scalar>::epsilon() * 100);
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, DisableConnectedLinks) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;
  using CollisionResultCpu = CollisionResult<Scalar>;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;

  // Three links with one capsule each.
  const auto proto = ParseTextProtoOrDie<AssemblyProto>(
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
        joints: {
          name: "joint_1"
          parent_link_name: "link_1"
          child_link_name: "link_2"
          parent_t_joint: { tx: 0 ty: 0 tz: 1 rx: 0 ry: 0 rz: 0 rw: 1 }
          axis: { vec: 1 vec: 0 vec: 0 }
          limits { lower: -3.14 upper: 3.14 }
          type: REVOLUTE
        }

        links: {
          name: "link_0"
          collision_shapes: {
            primitive { capsule { length: 1.0 radius: 0.1 } }
            link_t_shape { tx: 0.0 ty: 0.0 tz: 0.5 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }

        links: {
          name: "link_1"
          collision_shapes: {
            primitive { capsule { length: 1.0 radius: 0.1 } }
            link_t_shape { tx: 0.0 ty: 0.0 tz: 0.5 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }

        links: {
          name: "link_2"
          collision_shapes: {
            primitive { capsule { length: 1.0 radius: 0.1 } }
            link_t_shape { tx: 0.0 ty: 0.0 tz: 0.5 rx: 0 ry: 0 rz: 0 rw: 1 }
          }
        }
      )pb");
  auto assembly = FromProto(proto).value();

  AssemblyCollisionCheckerCpu collision;
  ASSERT_THAT(
      collision.AddAssembly(assembly, {"joint_0", "joint_1"},
                            AssemblyCollisionCheckerCpu::DefaultOptions()),
      StatusCodeIs(absl::StatusCode::kOk));

  VectorN<Scalar> joint_values = VectorN<Scalar>::Zero(2);
  typename AssemblyCollisionCheckerCpu::Scratch scratch =
      collision.CreateScratch();
  CollisionResultCpu result = collision.CreateCollisionResult();
  Status status;

  // Compute collisions with default collision masks and expect collisions.
  // Use "kComputeObjectDistances," so we get all collision pairs.
  CC_MALLOC_COUNTER_INIT();
  status = collision.ComputeCollisions(
      VoxelMapObjectCpu(), Pose3<Scalar>::Identity(), joint_values,
      QueryOptions().SetType(QueryOptions::Type::kComputeObjectDistances),
      scratch, result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  const int link_0 = collision.GetObjectIndexForName("link_0");
  const int link_1 = collision.GetObjectIndexForName("link_1");
  const int link_2 = collision.GetObjectIndexForName("link_2");
  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result.GetHasCollisions());
  EXPECT_EQ(result.GetObjectHits(link_0), ObjectIdSet(0b010));
  EXPECT_EQ(result.GetObjectHits(link_1), ObjectIdSet(0b101));
  EXPECT_EQ(result.GetObjectHits(link_2), ObjectIdSet(0b010));

  // If the masking option is set to connected links, there shouldn't be any
  // collisions.
  AssemblyCollisionCheckerCpu collision_with_masking;
  ASSERT_THAT(
      collision_with_masking.AddAssembly(
          assembly, {"joint_0", "joint_1"},
          AssemblyCollisionCheckerCpu::DefaultOptions().SetMaskingPolicy(
              CollisionMaskingPolicy::kConnectedLinks)),
      StatusCodeIs(absl::StatusCode::kOk));

  // Compute collisions with default collision masks and expect collisions.
  // Use "kComputeObjectDistanes," so we get all collision pairs.
  CC_MALLOC_COUNTER_INIT();
  status = collision_with_masking.ComputeCollisions(
      VoxelMapObjectCpu(), Pose3<Scalar>::Identity(), joint_values,
      QueryOptions().SetType(QueryOptions::Type::kComputeObjectDistances),
      scratch, result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_TRUE(result.GetObjectHits(link_0).Empty());
  EXPECT_TRUE(result.GetObjectHits(link_1).Empty());
  EXPECT_TRUE(result.GetObjectHits(link_2).Empty());
}

TYPED_TEST_P(AssemblyCollisionCheckerTest,
             AddRemoveObjectFromInclusionMaskWorks) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;
  using CollisionResultCpu = CollisionResult<Scalar>;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;

  // A box and a capsule connected with a rotational joint.
  const auto proto = ParseTextProtoOrDie<AssemblyProto>(
      R"pb(joints: {
             name: "joint_0"
             parent_link_name: "link_0"
             child_link_name: "link_1"
             parent_t_joint: { tx: 0 ty: 0 tz: 0.3 rx: 0 ry: 0 rz: 0 rw: 1 }
             axis: { vec: 1 vec: 0 vec: 0 }
             type: REVOLUTE
             limits: { upper: 3.14 lower: -3.14 }
           }
           links: {
             name: "link_0"
             collision_shapes: {
               primitive { box { size: { vec: [ 0.5, 0.5, 0.2 ] } } }
               link_t_shape { tx: 0.0 ty: 0.0 tz: 0.1 rx: 0 ry: 0 rz: 0 rw: 1 }
             }
           }
           links: {
             name: "link_1"
             collision_shapes: {
               primitive { capsule { length: 0.4 radius: 0.01 } }
               link_t_shape { tx: 0.0 ty: 0.0 tz: 0.2 rx: 0 ry: 0 rz: 0 rw: 1 }
             }
           }
      )pb");
  auto assembly = FromProto(proto).value();

  AssemblyCollisionCheckerCpu collision;
  ASSERT_THAT(
      collision.AddAssembly(
          assembly, {"joint_0"},
          AssemblyCollisionCheckerCpu::DefaultOptions().SetAABBPadding(10.0)),
      StatusCodeIs(absl::StatusCode::kOk));

  typename AssemblyCollisionCheckerCpu::Scratch scratch =
      collision.CreateScratch();
  CollisionResultCpu result = collision.CreateCollisionResult();

  // Cache the original filter state.
  const auto original_flags = collision.GetAllObjectFlags();
  // Without any filtering, there should be a collision at a large joint angle.
  EXPECT_EQ(
      collision
          .ComputeCollisions(VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                             VectorN<Scalar>({{Scalar{M_PI}}}),
                             QueryOptions().SetType(
                                 QueryOptions::Type::kComputeObjectDistances),
                             scratch, result)
          .code(),
      absl::StatusCode::kOk);

  const int link_0 = collision.GetObjectIndexForName("link_0");
  const int link_1 = collision.GetObjectIndexForName("link_1");
  EXPECT_TRUE(result.GetHasCollisions());
  EXPECT_EQ(result.GetObjectHits(link_0), ObjectIdSet(0b10));
  EXPECT_EQ(result.GetObjectHits(link_1), ObjectIdSet(0b01));
  EXPECT_LT(result.GetMinimumDistance(), Scalar{0});
  CC_ASSERT_OK_AND_ASSIGN(
      const auto original_state,
      collision.GetCollisionStateProto(VoxelMapObjectCpu(), scratch, result));

  // Verify that removing the object from the inclusion mask
  // removes the collision.
  CC_ASSERT_OK(collision.RemoveFromObjectInclusionSet(ObjectId{0}, ObjectId{1})
                   .ToAbslStatus());
  const auto modified_flags = collision.GetAllObjectFlags();

  EXPECT_EQ(
      collision
          .ComputeCollisions(VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                             VectorN<Scalar>({{Scalar{M_PI}}}),
                             QueryOptions().SetType(
                                 QueryOptions::Type::kComputeObjectDistances),
                             scratch, result)
          .code(),
      absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_TRUE(result.GetObjectHits(link_0).Empty());
  EXPECT_TRUE(result.GetObjectHits(link_1).Empty());
  EXPECT_GT(result.GetMinimumDistance(), Scalar{0});

  // Verify that adding the object from the inclusion mask adds the collision
  // back again.
  CC_ASSERT_OK(collision.AddToObjectInclusionSet(ObjectId{0}, ObjectId{1})
                   .ToAbslStatus());
  EXPECT_EQ(
      collision
          .ComputeCollisions(VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                             VectorN<Scalar>({{Scalar{M_PI}}}),
                             QueryOptions().SetType(
                                 QueryOptions::Type::kComputeObjectDistances),
                             scratch, result)
          .code(),
      absl::StatusCode::kOk);
  EXPECT_TRUE(result.GetHasCollisions());
  EXPECT_EQ(result.GetObjectHits(link_0), ObjectIdSet(0b10));
  EXPECT_EQ(result.GetObjectHits(link_1), ObjectIdSet(0b01));
  EXPECT_LT(result.GetMinimumDistance(), Scalar{0});

  // Verify that restoring object flags gives the same collision results.
  CC_ASSERT_OK(collision.SetObjectFlags(modified_flags).ToAbslStatus());
  EXPECT_EQ(
      collision
          .ComputeCollisions(VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                             VectorN<Scalar>({{Scalar{M_PI}}}),
                             QueryOptions().SetType(
                                 QueryOptions::Type::kComputeObjectDistances),
                             scratch, result)
          .code(),
      absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_TRUE(result.GetObjectHits(link_0).Empty());
  EXPECT_TRUE(result.GetObjectHits(link_1).Empty());
  EXPECT_GT(result.GetMinimumDistance(), Scalar{0});

  CC_ASSERT_OK(collision.SetObjectFlags(original_flags).ToAbslStatus());

  EXPECT_EQ(
      collision
          .ComputeCollisions(VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                             VectorN<Scalar>({{Scalar{M_PI}}}),
                             QueryOptions().SetType(
                                 QueryOptions::Type::kComputeObjectDistances),
                             scratch, result)
          .code(),
      absl::StatusCode::kOk);

  CC_ASSERT_OK_AND_ASSIGN(
      const auto restored_state,
      collision.GetCollisionStateProto(VoxelMapObjectCpu(), scratch, result));
  EXPECT_TRUE(result.GetHasCollisions());
  EXPECT_EQ(result.GetObjectHits(link_0), ObjectIdSet(0b10));
  EXPECT_EQ(result.GetObjectHits(link_1), ObjectIdSet(0b01));
  EXPECT_LT(result.GetMinimumDistance(), Scalar{0});
}

TYPED_TEST_P(AssemblyCollisionCheckerTest, StaticObjectWorks) {
  using Scalar = TypeParam;
  using AssemblyCollisionCheckerCpu = AssemblyCollisionChecker<Scalar>;
  using CollisionResultCpu = CollisionResult<Scalar>;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;

  // A simple assembly to test against static objects.
  // The box-shaped link has its lower face at z = 0.
  const auto proto = ParseTextProtoOrDie<AssemblyProto>(
      R"pb(joints: {
             name: "joint_0"
             parent_link_name: "link_0"
             child_link_name: "link_1"
             parent_t_joint: { tx: 0 ty: 0 tz: 0.3 rx: 0 ry: 0 rz: 0 rw: 1 }
             axis: { vec: 1 vec: 0 vec: 0 }
             type: REVOLUTE
             limits: { upper: 3.14 lower: -3.14 }
           }
           links: {
             name: "link_0"
             collision_shapes: {
               primitive { box { size: { vec: [ 0.5, 0.5, 0.2 ] } } }
               link_t_shape { tx: 0.0 ty: 0.0 tz: 0.1 rx: 0 ry: 0 rz: 0 rw: 1 }
             }
           }
           links: {
             name: "link_1"
             collision_shapes: {
               primitive { capsule { length: 0.4 radius: 0.01 } }
               link_t_shape { tx: 0.0 ty: 0.0 tz: 0.2 rx: 0 ry: 0 rz: 0 rw: 1 }
             }
           }
      )pb");
  auto assembly = FromProto(proto).value();

  AssemblyCollisionCheckerCpu collision;
  CC_ASSERT_OK_AND_ASSIGN(
      auto assembly_id,
      collision.AddAssembly(
          assembly, {"joint_0"},
          AssemblyCollisionCheckerCpu::DefaultOptions().SetAABBPadding(10.0)));
  ASSERT_NE(assembly_id, AssemblyIdAssigner::kInvalidId);

  typename AssemblyCollisionCheckerCpu::Scratch scratch =
      collision.CreateScratch();
  CollisionResultCpu result = collision.CreateCollisionResult();
  // Without additional objects, there should be no collision at a zero joint
  // angle.
  EXPECT_EQ(
      collision
          .ComputeCollisions(VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                             VectorN<Scalar>({{Scalar{0}}}),
                             QueryOptions().SetType(
                                 QueryOptions::Type::kComputeObjectDistances),
                             scratch, result)
          .code(),
      absl::StatusCode::kOk);

  int link_0_index = collision.GetObjectIndexForName("link_0");
  int link_1_index = collision.GetObjectIndexForName("link_1");
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_TRUE(result.GetObjectHits(link_0_index).Empty());
  EXPECT_TRUE(result.GetObjectHits(link_1_index).Empty());
  EXPECT_GT(result.GetMinimumDistance(), Scalar{0});

  // Add a static object and check that the distance is as expected given
  // the geometry, and world_pose_object and padding.
  geometry_shapes::Box box(Vector3d(1, 1, 0.1));
  Pose3<Scalar> world_pose_object(Vector3<Scalar>(0, 0, -0.1));
  CC_ASSERT_OK_AND_ASSIGN(
      auto object_below,
      collision.AddRelativeObject("lower_box", kInvalidObjectId,
                                  world_pose_object, {&box}, Scalar{0.01}));
  EXPECT_NE(object_below, kInvalidObjectId);

  collision.ResizeScratch(scratch);
  collision.ResizeCollisionResult(result);

  EXPECT_EQ(
      collision
          .ComputeCollisions(VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                             VectorN<Scalar>({{Scalar{0}}}),
                             QueryOptions().SetType(
                                 QueryOptions::Type::kComputeObjectDistances),
                             scratch, result)
          .code(),
      absl::StatusCode::kOk);

  link_0_index = collision.GetObjectIndexForName("link_0");
  link_1_index = collision.GetObjectIndexForName("link_1");
  int lower_box_index = collision.GetObjectIndexForName("lower_box");
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_TRUE(result.GetObjectHits(link_0_index).Empty());
  EXPECT_TRUE(result.GetObjectHits(link_1_index).Empty());
  EXPECT_TRUE(result.GetObjectHits(lower_box_index).Empty());
  EXPECT_GT(result.GetMinimumDistance(), Scalar{0});
  // Box height is 0.1, offset is 0.1, padding 0.01, so distance to z=0 plane is
  // 0.1-0.05-0.01.
  EXPECT_NEAR(result.GetMinimumDistance(lower_box_index), 0.04,
              std::numeric_limits<Scalar>::epsilon() * 10);

  // Removing the assembly and checking with just the static object should work.
  CC_ASSERT_OK(collision.RemoveAssembly(assembly_id));
  collision.ResizeScratch(scratch);
  collision.ResizeCollisionResult(result);
  EXPECT_EQ(
      collision
          .ComputeCollisions(VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                             VectorN<Scalar>(),
                             QueryOptions().SetType(
                                 QueryOptions::Type::kComputeObjectDistances),
                             scratch, result)
          .code(),
      absl::StatusCode::kOk);

  lower_box_index = collision.GetObjectIndexForName("lower_box");
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_TRUE(result.GetObjectHits(lower_box_index).Empty());
  EXPECT_EQ(result.GetMinimumDistance(lower_box_index),
            std::numeric_limits<Scalar>::infinity());

  // Add the assembly again, plus a second box that intersects the assembly
  // capsule link.
  CC_ASSERT_OK_AND_ASSIGN(
      assembly_id,
      collision.AddAssembly(
          assembly, {"joint_0"},
          AssemblyCollisionCheckerCpu::DefaultOptions().SetAABBPadding(10.0)));
  ASSERT_NE(assembly_id, AssemblyIdAssigner::kInvalidId);
  world_pose_object.translation().z() = +0.5;
  CC_ASSERT_OK_AND_ASSIGN(
      auto object_above,
      collision.AddRelativeObject("upper_box", kInvalidObjectId,
                                  world_pose_object, {&box}, Scalar{0}));
  EXPECT_NE(object_above, kInvalidObjectId);

  collision.ResizeScratch(scratch);
  collision.ResizeCollisionResult(result);

  EXPECT_EQ(
      collision
          .ComputeCollisions(VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                             VectorN<Scalar>({{Scalar{0}}}),
                             QueryOptions().SetType(
                                 QueryOptions::Type::kComputeObjectDistances),
                             scratch, result)
          .code(),
      absl::StatusCode::kOk);

  link_0_index = collision.GetObjectIndexForName("link_0");
  link_1_index = collision.GetObjectIndexForName("link_1");
  lower_box_index = collision.GetObjectIndexForName("lower_box");
  int upper_box_index = collision.GetObjectIndexForName("upper_box");
  EXPECT_TRUE(result.GetHasCollisions());
  EXPECT_TRUE(result.GetObjectHits(link_0_index).Empty());
  EXPECT_EQ(result.GetObjectHits(link_1_index),
            result.GetObjectIdSet(upper_box_index));
  EXPECT_TRUE(result.GetObjectHits(lower_box_index).Empty());
  EXPECT_EQ(result.GetObjectHits(upper_box_index),
            result.GetObjectIdSet(link_1_index));
  EXPECT_LE(result.GetMinimumDistance(), Scalar{0});
  EXPECT_NEAR(result.GetMinimumDistance(lower_box_index), Scalar{0.04},
              std::numeric_limits<Scalar>::epsilon() * 10);
  EXPECT_LE(result.GetMinimumDistance(upper_box_index), Scalar{0});

  // Update the object pose and expect the minimum distance to change
  // accordingly (box is shifted by 0.01 compared to the test case above).
  EXPECT_EQ(collision
                .SetRelativeObjectPose(
                    object_below, Pose3<Scalar>(Vector3<Scalar>(0, 0, -0.09)))
                .code(),
            absl::StatusCode::kOk);
  EXPECT_EQ(
      collision
          .ComputeCollisions(VoxelMapObjectCpu(), Pose3<Scalar>::Identity(),
                             VectorN<Scalar>({{Scalar{0}}}),
                             QueryOptions().SetType(
                                 QueryOptions::Type::kComputeObjectDistances),
                             scratch, result)
          .code(),
      absl::StatusCode::kOk);
  lower_box_index = collision.GetObjectIndexForName("lower_box");
  EXPECT_NEAR(result.GetMinimumDistance(lower_box_index), 0.03,
              std::numeric_limits<Scalar>::epsilon() * 10);
  // Expect the original result of the assembly root pose is shifted as well.
  EXPECT_EQ(
      collision
          .ComputeCollisions(VoxelMapObjectCpu(),
                             Pose3<Scalar>(Vector3<Scalar>(0, 0, 0.01)),
                             VectorN<Scalar>({{Scalar{0}}}),
                             QueryOptions().SetType(
                                 QueryOptions::Type::kComputeObjectDistances),
                             scratch, result)
          .code(),
      absl::StatusCode::kOk);
  EXPECT_NEAR(result.GetMinimumDistance(lower_box_index), 0.04,
              std::numeric_limits<Scalar>::epsilon() * 10);

  // Should be able to remove static objects.
  EXPECT_EQ(collision.RemoveRelativeObject(object_above).code(),
            absl::StatusCode::kOk);
  // Second attempt to remove should fail.
  EXPECT_EQ(collision.RemoveRelativeObject(object_above).code(),
            absl::StatusCode::kFailedPrecondition);

  // Put the checker back into a state with > 1 static objects.
  // Verify the 'lower_box' is still present, then add back the 'upper_box'
  EXPECT_NE(collision.GetObjectId("lower_box"), kInvalidObjectId);
  CC_ASSERT_OK_AND_ASSIGN(
      object_below,
      collision.AddRelativeObject("lower_box", kInvalidObjectId,
                                  world_pose_object, {&box}, Scalar{0.01}));
  EXPECT_NE(object_below, kInvalidObjectId);

  collision.RemoveAllStaticObjects();
  EXPECT_EQ(collision.GetObjectId("lower_box"), kInvalidObjectId);
  EXPECT_EQ(collision.GetObjectId("upper_box"), kInvalidObjectId);
}

REGISTER_TYPED_TEST_SUITE_P(
    AssemblyCollisionCheckerTest, Options, AddAssemblyChecksErrors, CreateOK,
    IndexIdNameUtilities, AssemblyPadding, ComputeCollisionsErrors,
    ComputeCollisionsSphereSphere, ComputeCollisionsCapsuleSphere,
    ComputeCollisionsCapsuleCapsule, ComputeCollisionsBox,
    DisableConnectedLinks, Attachments, AddRemoveObjectFromInclusionMaskWorks,
    StaticObjectWorks);

typedef ::testing::Types<float, double> FPTypes;

INSTANTIATE_TYPED_TEST_SUITE_P(AssemblyCollisionCheckerTestSuite,
                               AssemblyCollisionCheckerTest, FPTypes);

}  // namespace
}  // namespace collision_checking
