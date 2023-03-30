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

#include "collision_checking/compute_collisions.h"

#include <vector>

#include "absl/status/status.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/object_id.h"
#include "collision_checking/options.h"
#include "collision_checking/test_utils.h"
#include "eigenmath/matchers.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace collision_checking {
namespace {

using ::eigenmath::testing::IsApprox;
using testing::MinimumGeometryInfoIs;

template <typename T>
class CollisionCheckerTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(CollisionCheckerTest);

TYPED_TEST_P(CollisionCheckerTest, ComputeCollisionsArgumentCheckingTest) {
  using Scalar = TypeParam;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;
  using MovingObjectsCpu = CollisionObjects<Scalar, DefaultAllocatorTraits>;
  using CollisionResultCpu = CollisionResult<Scalar, DefaultAllocatorTraits>;

  VoxelMapObjectCpu environment;
  MovingObjectsCpu robot;
  CollisionResultCpu result(0, QueryOptions());
  Status status;

  result.Allocate(10);
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kInvalidArgument);
  result.Allocate(0);

  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());

  // Result allocation doesn't match query type.
  result.Allocate(10);
  robot.ResizeBuffers({{1, 2, 3}});
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeContactPoints), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kInvalidArgument);
}

// Expects that contact point info is consistent and matches the expected
// contact pair bitmask provided by `other_id_bitmasks`.
template <typename Scalar, typename Traits>
void ExpectCollisionResultIsConsistent(
    CollisionResult<Scalar, Traits>& result,
    const std::vector<ObjectIdSet> other_id_sets) {
  constexpr Scalar kExpectedPrecision =
      100 * std::numeric_limits<Scalar>::epsilon();

  ASSERT_EQ(result.GetObjectCount(), other_id_sets.size());
  ASSERT_TRUE(result.MemoryIsAllocatedFor(
      QueryOptions().SetType(QueryOptions::kComputeContactPoints)));

  for (int id = 0; id < result.GetObjectCount(); id++) {
    SCOPED_TRACE(absl::StrCat("id= ", id));
    const auto& contact_info = result.GetContactPointInfo(id);
    const auto distance = result.GetMinimumDistance(id);
    EXPECT_EQ(contact_info.other_id_set, other_id_sets[id]);
    EXPECT_NEAR(std::copysign((contact_info.contact_point_other -
                               contact_info.contact_point_this)
                                  .norm(),
                              distance),
                distance, kExpectedPrecision)
        << "contact_point_this= " << contact_info.contact_point_this.transpose()
        << "\ncontact_point_other= "
        << contact_info.contact_point_other.transpose()
        << "\ndistance= " << distance;

    EXPECT_THAT(contact_info.normal_this_other * distance,
                IsApprox(contact_info.contact_point_other -
                         contact_info.contact_point_this))
        << "contact_point_this= " << contact_info.contact_point_this.transpose()
        << "\ncontact_point_other= "
        << contact_info.contact_point_other.transpose()
        << "\nnormal_this_other= " << contact_info.normal_this_other.transpose()
        << "\ndistance= " << distance;
  }
}

// Test case with no collisions.
TYPED_TEST_P(CollisionCheckerTest, ComputeCollisionsCase01Test) {
  using Scalar = TypeParam;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;
  using MovingObjectsCpu = CollisionObjects<Scalar, DefaultAllocatorTraits>;
  using CollisionResultCpu = CollisionResult<Scalar, DefaultAllocatorTraits>;

  constexpr Scalar kExpectedPrecision =
      10 * std::numeric_limits<Scalar>::epsilon();

  VoxelMapObjectCpu environment;
  MovingObjectsCpu robot;
  CollisionResultCpu result(0, QueryOptions());
  Status status;

  robot.ResizeBuffers({{.num_spheres = 1, .num_capsules = 0, .num_boxes = 0},
                       {.num_spheres = 0, .num_capsules = 1, .num_boxes = 0}});
  robot.objects[0].flags.id_set = ObjectIdSet(0b01);
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].flags.id_set = ObjectIdSet(0b10);
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();

  environment.ResizeBuffers(1);

  result.Allocate(robot.objects.size());

  // No robot self collision, minimum distance between robot objects.
  robot.objects[0].spheres[0].center << 0, 0, 0;
  robot.objects[0].spheres[0].radius = 0.1;
  robot.objects[1].capsules[0].center << 1, 0, 0;
  robot.objects[1].capsules[0].direction << 0, 1, 0;
  robot.objects[1].capsules[0].radius = 0.2;
  robot.objects[1].capsules[0].half_length = 0.1;

  // 1 No AABB padding, so AABBs do not overlap.
  UpdateAlignedBoxes(/*padding=*/Scalar{0}, robot);
  Sphere<Scalar> the_voxel{{Vector3<Scalar>(0, 0, 10)}, {0.02}};
  environment.AddSphere(the_voxel, 42);
  environment.UpdateQueryStructures();

  // 1.1 Default options.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_EQ(result.GetMinimumDistance(0),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(1),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetObjectHits(0), ObjectIdSet(0));
  EXPECT_EQ(result.GetObjectHits(1), ObjectIdSet(0));
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 1.2 ComputeDistances option.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_EQ(result.GetMinimumDistance(0),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(1),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 2 Enough AABB padding so AABBs do overlap.
  UpdateAlignedBoxes(/*padding=*/Scalar{10}, robot);

  // 2.1 Default options.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  // Distances not computed!
  EXPECT_EQ(result.GetMinimumDistance(0),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(1),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 2.2 ComputeDistances option.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_NEAR(result.GetMinimumDistance(0), 0.7, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(1), 0.7, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(), 0.7, kExpectedPrecision);
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 2.3 ComputeContactPoints option.
  CollisionResultCpu result_with_contacts(
      robot.objects.size(),
      QueryOptions().SetType(QueryOptions::kComputeContactPoints));
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeContactPoints),
      result_with_contacts);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result_with_contacts.GetHasCollisions());
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), 0.7,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(1), 0.7,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(), 0.7,
              kExpectedPrecision);
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  ExpectCollisionResultIsConsistent(result_with_contacts,
                                    {ObjectIdSet(0b10), ObjectIdSet(0b01)});

  // 3.1 Mask object / object collision and expect minimum distance to
  // obstacle, not other object.
  robot.objects[0].flags.inclusion_set.Remove(robot.objects[1].flags.id_set);
  robot.objects[1].flags.inclusion_set.Remove(robot.objects[0].flags.id_set);

  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_NEAR(result.GetMinimumDistance(0), 9.88, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(1),
              std::sqrt(Scalar{10 * 10 + 1 * 1}) - Scalar{0.2} - Scalar{0.02},
              kExpectedPrecision);
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(the_voxel.center, 42));

  // 3.2 Same as 3.1, but with contact point computation.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeContactPoints),
      result_with_contacts);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result_with_contacts.GetHasCollisions());
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), 9.88,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(1),
              std::sqrt(Scalar{10 * 10 + 1 * 1}) - Scalar{0.2} - Scalar{0.02},
              kExpectedPrecision);
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(the_voxel.center, 42));
  ExpectCollisionResultIsConsistent(
      result_with_contacts,
      {ObjectIdSet(kDefaultVoxelMapId), ObjectIdSet(kDefaultVoxelMapId)});
}

// Test case with self collision.
TYPED_TEST_P(CollisionCheckerTest, ComputeCollisionsCase02Test) {
  using Scalar = TypeParam;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;
  using MovingObjectsCpu = CollisionObjects<Scalar, DefaultAllocatorTraits>;
  using CollisionResultCpu = CollisionResult<Scalar, DefaultAllocatorTraits>;

  constexpr Scalar kExpectedPrecision =
      10 * std::numeric_limits<Scalar>::epsilon();

  VoxelMapObjectCpu environment;
  MovingObjectsCpu robot;
  CollisionResultCpu result(0, QueryOptions());
  Status status;

  robot.ResizeBuffers({{.num_spheres = 1}, {.num_capsules = 1}});
  robot.objects[0].flags.id_set = ObjectIdSet(0b01);
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].flags.id_set = ObjectIdSet(0b10);
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();
  environment.ResizeBuffers(1);
  result.Allocate(robot.objects.size());

  // 2. Robot self collision, minimum distance between robot objects.
  robot.objects[0].spheres[0].center << 0, 0, 0;
  robot.objects[0].spheres[0].radius = 0.1;
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].capsules[0].center << 0.2, 0, 0;
  robot.objects[1].capsules[0].direction << 0, 1, 0;
  robot.objects[1].capsules[0].radius = 0.2;
  robot.objects[1].capsules[0].half_length = 0.1;

  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();

  // No AABB padding, but AABBs do overlap.
  UpdateAlignedBoxes(/*padding=*/Scalar{0}, robot);

  Sphere<Scalar> the_voxel{{Vector3<Scalar>(0, 0, 10)}, {0.02}};
  environment.AddSphere(the_voxel, 42);
  environment.UpdateQueryStructures();

  // 1 Default options.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_TRUE(result.GetHasCollisions());
  // Distances not computed!
  EXPECT_EQ(result.GetMinimumDistance(0),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(1),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetObjectHits(0), ObjectIdSet(0b10));
  EXPECT_EQ(result.GetObjectHits(1), ObjectIdSet(0b01));
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 2 ComputeDistances option.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result.GetHasCollisions());
  EXPECT_NEAR(result.GetMinimumDistance(0), -0.1, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(1), -0.1, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(), -0.1, kExpectedPrecision);
  EXPECT_EQ(result.GetObjectHits(0), ObjectIdSet(0b10));
  EXPECT_EQ(result.GetObjectHits(1), ObjectIdSet(0b01));
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 3 ComputeContactPoints option.
  CollisionResultCpu result_with_contacts(
      robot.objects.size(),
      QueryOptions().SetType(QueryOptions::kComputeContactPoints));
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeContactPoints),
      result_with_contacts);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result_with_contacts.GetHasCollisions());
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), -0.1,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(1), -0.1,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(), -0.1,
              kExpectedPrecision);
  EXPECT_EQ(result_with_contacts.GetObjectHits(0), ObjectIdSet(0b10));
  EXPECT_EQ(result_with_contacts.GetObjectHits(1), ObjectIdSet(0b01));
  ExpectCollisionResultIsConsistent(result_with_contacts,
                                    {ObjectIdSet(0b10), ObjectIdSet(0b01)});
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 4.1 Mask object / object collision and expect minimum distance to
  // obstacle, not other object.
  // Update bounding box with large padding, so environment collision is
  // computed.
  UpdateAlignedBoxes(/*padding=*/Scalar{10}, robot);
  robot.objects[0].flags.inclusion_set.Remove(robot.objects[1].flags.id_set);
  robot.objects[1].flags.inclusion_set.Remove(robot.objects[0].flags.id_set);

  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_NEAR(result.GetMinimumDistance(0), 9.88, kExpectedPrecision);
  EXPECT_NEAR(
      result.GetMinimumDistance(1),
      std::sqrt(Scalar{10 * 10 + 0.2 * 0.2}) - Scalar{0.2} - Scalar{0.02},
      kExpectedPrecision);
  EXPECT_EQ(result.GetMinimumDistance(1), result.GetMinimumDistance());
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(the_voxel.center, 42));

  // 4.2 Same as 4.1, but with contact point computation.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeContactPoints),
      result_with_contacts);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_FALSE(result_with_contacts.GetHasCollisions());
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), 9.88,
              kExpectedPrecision);
  EXPECT_NEAR(
      result_with_contacts.GetMinimumDistance(1),
      std::sqrt(Scalar{10 * 10 + 0.2 * 0.2}) - Scalar{0.2} - Scalar{0.02},
      kExpectedPrecision);
  EXPECT_EQ(result_with_contacts.GetMinimumDistance(1),
            result_with_contacts.GetMinimumDistance());
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());

  ExpectCollisionResultIsConsistent(
      result_with_contacts,
      {ObjectIdSet(kDefaultVoxelMapId), ObjectIdSet(kDefaultVoxelMapId)});

  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(the_voxel.center, 42));
}

// Test case with no collision and minimum distance to environment.
TYPED_TEST_P(CollisionCheckerTest, ComputeCollisionsCase03Test) {
  using Scalar = TypeParam;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;
  using MovingObjectsCpu = CollisionObjects<Scalar, DefaultAllocatorTraits>;
  using CollisionResultCpu = CollisionResult<Scalar, DefaultAllocatorTraits>;

  constexpr Scalar kExpectedPrecision =
      10 * std::numeric_limits<Scalar>::epsilon();

  VoxelMapObjectCpu environment;
  MovingObjectsCpu robot;
  CollisionResultCpu result(0, QueryOptions());
  Status status;

  robot.ResizeBuffers({{.num_spheres = 1}, {.num_capsules = 1}});
  robot.objects[0].flags.id_set = ObjectIdSet(0b01);
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].flags.id_set = ObjectIdSet(0b10);
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();
  environment.ResizeBuffers(1);
  result.Allocate(robot.objects.size());

  // No robot self collision, minimum distance to environment, but no
  // collision.
  robot.objects[0].spheres[0].center << 0, 0, 0;
  robot.objects[0].spheres[0].radius = 0.1;
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].capsules[0].center << 1, 0, 0;
  robot.objects[1].capsules[0].direction << 0, 1, 0;
  robot.objects[1].capsules[0].radius = 0.2;
  robot.objects[1].capsules[0].half_length = 0.1;
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();

  Sphere<Scalar> the_voxel{{Vector3<Scalar>(0, 0, 0.5)}, {0.02}};
  environment.AddSphere(the_voxel, 42);
  environment.UpdateQueryStructures();

  // 1 No AABB padding, so AABBs do not overlap.
  UpdateAlignedBoxes(/*padding=*/Scalar{0}, robot);

  // 1.1 Default options.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_EQ(result.GetMinimumDistance(0),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(1),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 1.2 ComputeDistances option.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_EQ(result.GetMinimumDistance(0),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(1),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 2 Enough AABB padding so AABBs do overlap.
  UpdateAlignedBoxes(/*padding=*/Scalar{10}, robot);
  // 2.1 Default options.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);

  EXPECT_FALSE(result.GetHasCollisions());
  // Distance value not computed.
  EXPECT_EQ(result.GetMinimumDistance(0),
            std::numeric_limits<Scalar>::infinity());
  // Distance value not computed.
  EXPECT_EQ(result.GetMinimumDistance(1),
            std::numeric_limits<Scalar>::infinity());
  // Distance value not computed.
  EXPECT_EQ(result.GetMinimumDistance(),
            std::numeric_limits<Scalar>::infinity());

  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 2.2 ComputeDistances option.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  // Object 0 has minimum distance to the obstacle.
  EXPECT_NEAR(result.GetMinimumDistance(0), 0.38, kExpectedPrecision);
  // Object 1 has minimum distance to object 0.
  EXPECT_NEAR(result.GetMinimumDistance(1), 0.7, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(), 0.38, kExpectedPrecision);

  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(the_voxel.center, 42));

  // 2.3 ComputeContactPoints option.
  CollisionResultCpu result_with_contacts(
      robot.objects.size(),
      QueryOptions().SetType(QueryOptions::kComputeContactPoints));
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeContactPoints),
      result_with_contacts);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result_with_contacts.GetHasCollisions());
  // Object 0 has minimum distance to the obstacle.
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), 0.38,
              kExpectedPrecision);
  // Object 1 has minimum distance to object 0.
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(1), 0.7,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(), 0.38,
              kExpectedPrecision);
  ExpectCollisionResultIsConsistent(
      result_with_contacts,
      {ObjectIdSet(kDefaultVoxelMapId), ObjectIdSet(0b01)});
  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(the_voxel.center, 42));

  // 3.1 Mask obstacle collisons for first object, not other object.
  robot.objects[0].flags.inclusion_set.Remove(kVoxelMapIdSet);
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  // Minimum distance is to object 1.
  EXPECT_NEAR(result.GetMinimumDistance(0), 0.7, kExpectedPrecision);
  // Minimum distance is still to object 0.
  EXPECT_NEAR(result.GetMinimumDistance(1), 0.7, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(0), 0.7, kExpectedPrecision);
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 3.2 Same as 3.1 but with contact point computation.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeContactPoints),
      result_with_contacts);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result_with_contacts.GetHasCollisions());
  // Minimum distance is to obstacle 1.
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), 0.7,
              kExpectedPrecision);
  // Minimum distance is still to obstacle 0.
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(1), 0.7,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), 0.7,
              kExpectedPrecision);
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  ExpectCollisionResultIsConsistent(result_with_contacts,
                                    {ObjectIdSet(0b10), ObjectIdSet(0b01)});
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);
}

// Test case with no self collision and environment collision.
TYPED_TEST_P(CollisionCheckerTest, ComputeCollisionsCase04Test) {
  using Scalar = TypeParam;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;
  using MovingObjectsCpu = CollisionObjects<Scalar, DefaultAllocatorTraits>;
  using CollisionResultCpu = CollisionResult<Scalar, DefaultAllocatorTraits>;

  constexpr Scalar kExpectedPrecision =
      10 * std::numeric_limits<Scalar>::epsilon();

  VoxelMapObjectCpu environment;
  MovingObjectsCpu robot;
  CollisionResultCpu result(0, QueryOptions());
  Status status;

  robot.ResizeBuffers({{.num_spheres = 1}, {.num_capsules = 1}});
  robot.objects[0].flags.id_set = ObjectIdSet(0b01);
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].flags.id_set = ObjectIdSet(0b10);
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();
  environment.ResizeBuffers(1);
  result.Allocate(robot.objects.size());

  // No robot self collision, minimum distance to environment & collision.
  robot.objects[0].spheres[0].center << 0, 0, 0;
  robot.objects[0].spheres[0].radius = 0.1;
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].capsules[0].center << 1, 0, 0;
  robot.objects[1].capsules[0].direction << 0, 1, 0;
  robot.objects[1].capsules[0].radius = 0.2;
  robot.objects[1].capsules[0].half_length = 0.1;
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();

  Sphere<Scalar> the_voxel{{Vector3<Scalar>(0, 0, 0.1)}, {0.02}};
  environment.AddSphere(the_voxel, 42);
  environment.UpdateQueryStructures();

  // 1 No AABB padding.
  UpdateAlignedBoxes(/*padding=*/Scalar{0}, robot);

  // 1.1 Default options.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result.GetHasCollisions());
  // Distances not computed!
  EXPECT_EQ(result.GetMinimumDistance(0),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(1),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(0),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetObjectHits(0), ObjectIdSet(kDefaultVoxelMapId));
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 1.2 ComputeDistances option.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result.GetHasCollisions());
  EXPECT_NEAR(result.GetMinimumDistance(0), -0.02, kExpectedPrecision);
  EXPECT_EQ(result.GetMinimumDistance(1),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_NEAR(result.GetMinimumDistance(), -0.02, kExpectedPrecision);
  EXPECT_EQ(result.GetObjectHits(0), ObjectIdSet(kDefaultVoxelMapId));
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(the_voxel.center, 42));

  // 2 With AABB padding.
  UpdateAlignedBoxes(/*padding=*/Scalar{10}, robot);

  // 2.1 Default options.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result.GetHasCollisions());
  // Distances not computed!
  EXPECT_EQ(result.GetMinimumDistance(0),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(1),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetMinimumDistance(0),
            std::numeric_limits<Scalar>::infinity());
  EXPECT_EQ(result.GetObjectHits(0), ObjectIdSet(kDefaultVoxelMapId));
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 2.2 ComputeDistances option.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result.GetHasCollisions());
  EXPECT_NEAR(result.GetMinimumDistance(0), -0.02, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(1), 0.7, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(), -0.02, kExpectedPrecision);
  EXPECT_EQ(result.GetObjectHits(0), ObjectIdSet(kDefaultVoxelMapId));
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(the_voxel.center, 42));

  // 2.3 ComputeContactPoints option.
  CollisionResultCpu result_with_contacts(
      robot.objects.size(),
      QueryOptions().SetType(QueryOptions::kComputeContactPoints));
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeContactPoints),
      result_with_contacts);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result_with_contacts.GetHasCollisions());
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), -0.02,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(1), 0.7,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(), -0.02,
              kExpectedPrecision);
  EXPECT_EQ(result_with_contacts.GetObjectHits(0),
            ObjectIdSet(kDefaultVoxelMapId));
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  ExpectCollisionResultIsConsistent(
      result_with_contacts,
      {ObjectIdSet(kDefaultVoxelMapId), ObjectIdSet(0b01)});
  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(the_voxel.center, 42));

  // 3.1 Mask obstacle collisons for first object.
  robot.objects[0].flags.inclusion_set.Remove(kVoxelMapIdSet);
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result.GetHasCollisions());
  // Minimum distance is to object 1.
  EXPECT_NEAR(result.GetMinimumDistance(0), 0.7, kExpectedPrecision);
  // Minimum distance is still to object 0.
  EXPECT_NEAR(result.GetMinimumDistance(1), 0.7, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(0), 0.7, kExpectedPrecision);
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // 3.2 Same as 3.1, but with contact point computation.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeContactPoints),
      result_with_contacts);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_FALSE(result_with_contacts.GetHasCollisions());
  // Minimum distance is to obstacle.
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), 0.7,
              kExpectedPrecision);
  // Minimum distance is still to obstacle.
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(1), 0.7,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), 0.7,
              kExpectedPrecision);
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
  ExpectCollisionResultIsConsistent(result_with_contacts,
                                    {ObjectIdSet(0b10), ObjectIdSet(0b01)});
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);
}

// Test case with self & environment collision, self collision distance is
// minimal.
TYPED_TEST_P(CollisionCheckerTest, ComputeCollisionsCase05Test) {
  using Scalar = TypeParam;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;
  using MovingObjectsCpu = CollisionObjects<Scalar, DefaultAllocatorTraits>;
  using CollisionResultCpu = CollisionResult<Scalar, DefaultAllocatorTraits>;

  constexpr Scalar kExpectedPrecision =
      10 * std::numeric_limits<Scalar>::epsilon();

  VoxelMapObjectCpu environment;
  MovingObjectsCpu robot;
  CollisionResultCpu result(0, QueryOptions());
  Status status;

  robot.ResizeBuffers({{.num_spheres = 1}, {.num_capsules = 1}});
  robot.objects[0].flags.id_set = ObjectIdSet(0b01);
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].flags.id_set = ObjectIdSet(0b10);
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();
  environment.ResizeBuffers(1);
  result.Allocate(robot.objects.size());

  // Self collision & environment collision, self collision is minimal.
  // This tests that the correct minimum is determined, so is only tested with
  // AABB padding and distance computation enabled.
  robot.objects[0].spheres[0].center << 0, 0, 0;
  robot.objects[0].spheres[0].radius = 0.1;
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].capsules[0].center << 0.1, 0, 0;
  robot.objects[1].capsules[0].direction << 0, 1, 0;
  robot.objects[1].capsules[0].radius = 0.2;
  robot.objects[1].capsules[0].half_length = 0.1;
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();

  Sphere<Scalar> the_voxel{{Vector3<Scalar>(0, 0, 0.1)}, {0.02}};
  environment.AddSphere(the_voxel, 42);
  environment.UpdateQueryStructures();

  UpdateAlignedBoxes(/*padding=*/Scalar{10}, robot);

  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result.GetHasCollisions());

  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result.GetHasCollisions());
  EXPECT_NEAR(result.GetMinimumDistance(0), -0.2, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(1), -0.2, kExpectedPrecision);
  EXPECT_NEAR(result.GetMinimumDistance(), -0.2, kExpectedPrecision);
  EXPECT_EQ(result.GetObjectHits(0),
            ObjectIdSet(0b10) | ObjectIdSet(kDefaultVoxelMapId));
  EXPECT_EQ(result.GetObjectHits(1),
            ObjectIdSet(0b01) | ObjectIdSet(kDefaultVoxelMapId));
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // Same setup, but with contact point computation.
  CollisionResultCpu result_with_contacts(
      robot.objects.size(),
      QueryOptions().SetType(QueryOptions::kComputeContactPoints));
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeContactPoints),
      result_with_contacts);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result_with_contacts.GetHasCollisions());
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), -0.2,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(1), -0.2,
              kExpectedPrecision);
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(), -0.2,
              kExpectedPrecision);
  EXPECT_EQ(result_with_contacts.GetObjectHits(0),
            ObjectIdSet(0b10) | ObjectIdSet(kDefaultVoxelMapId));
  EXPECT_EQ(result_with_contacts.GetObjectHits(1),
            ObjectIdSet(0b01) | ObjectIdSet(kDefaultVoxelMapId));

  ExpectCollisionResultIsConsistent(result_with_contacts,
                                    {ObjectIdSet(0b10), ObjectIdSet(0b01)});
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);
}

// Test case with self & environment collision, environment collision distance
// is minimal.
TYPED_TEST_P(CollisionCheckerTest, ComputeCollisionsCase06Test) {
  using Scalar = TypeParam;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;
  using MovingObjectsCpu = CollisionObjects<Scalar, DefaultAllocatorTraits>;
  using CollisionResultCpu = CollisionResult<Scalar, DefaultAllocatorTraits>;

  constexpr Scalar kExpectedPrecision =
      10 * std::numeric_limits<Scalar>::epsilon();

  VoxelMapObjectCpu environment;
  MovingObjectsCpu robot;
  CollisionResultCpu result(0, QueryOptions());
  Status status;

  robot.ResizeBuffers({{.num_spheres = 1}, {.num_capsules = 1}});
  robot.objects[0].flags.id_set = ObjectIdSet(0b01);
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].flags.id_set = ObjectIdSet(0b10);
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();
  environment.ResizeBuffers(1);
  result.Allocate(robot.objects.size());

  // Self collision & environment collision, environment collision is
  // minimal.
  robot.objects[0].spheres[0].center << 0, 0, 0;
  robot.objects[0].spheres[0].radius = 0.1;
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].capsules[0].center << 0.29, 0, 0;
  robot.objects[1].capsules[0].direction << 0, 1, 0;
  robot.objects[1].capsules[0].radius = 0.2;
  robot.objects[1].capsules[0].half_length = 0.1;
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();

  Sphere<Scalar> the_voxel{{Vector3<Scalar>(0, 0, 0.1)}, {0.02}};
  environment.AddSphere(the_voxel, 42);
  environment.UpdateQueryStructures();

  UpdateAlignedBoxes(/*padding=*/Scalar{10}, robot);

  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);

  EXPECT_TRUE(result.GetHasCollisions());
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  EXPECT_TRUE(result.GetHasCollisions());
  // This is the distance to the obstacle.
  EXPECT_NEAR(result.GetMinimumDistance(0), -0.02, kExpectedPrecision);
  // This is the distance to the other object.
  EXPECT_NEAR(result.GetMinimumDistance(1), -0.01, kExpectedPrecision);
  // Total minimum is from obstacle to object 0.
  EXPECT_NEAR(result.GetMinimumDistance(), -0.02, kExpectedPrecision);
  EXPECT_EQ(result.GetObjectHits(0),
            ObjectIdSet(0b10) | ObjectIdSet(kDefaultVoxelMapId));
  EXPECT_EQ(result.GetObjectHits(1), ObjectIdSet(0b01));
  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(the_voxel.center, 42));

  // Same setup, but with contact point computation.
  CollisionResultCpu result_with_contacts(
      robot.objects.size(),
      QueryOptions().SetType(QueryOptions::kComputeContactPoints));
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeContactPoints),
      result_with_contacts);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_TRUE(result_with_contacts.GetHasCollisions());
  // This is the distance to the obstacle.
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(0), -0.02,
              kExpectedPrecision);
  // This is the distance to the other object.
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(1), -0.01,
              kExpectedPrecision);
  // Total minimum is from obstacle to object 0.
  EXPECT_NEAR(result_with_contacts.GetMinimumDistance(), -0.02,
              kExpectedPrecision);
  EXPECT_EQ(result_with_contacts.GetObjectHits(0),
            ObjectIdSet(0b10) | ObjectIdSet(kDefaultVoxelMapId));
  EXPECT_EQ(result_with_contacts.GetObjectHits(1), ObjectIdSet(0b01));

  ExpectCollisionResultIsConsistent(
      result_with_contacts,
      {ObjectIdSet(kDefaultVoxelMapId), ObjectIdSet(0b01)});
  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(the_voxel.center, 42));
}

TYPED_TEST_P(CollisionCheckerTest, ComputeCollisionsCaseExtraExclusionBitmask) {
  using Scalar = TypeParam;
  using VoxelMapObjectCpu = VoxelMapObject<Scalar>;
  using MovingObjectsCpu = CollisionObjects<Scalar, DefaultAllocatorTraits>;
  using CollisionResultCpu = CollisionResult<Scalar, DefaultAllocatorTraits>;

  VoxelMapObjectCpu environment;
  MovingObjectsCpu robot;
  CollisionResultCpu result(0, QueryOptions());
  Status status;

  robot.ResizeBuffers({{.num_spheres = 1}, {.num_spheres = 1}});
  robot.objects[0].flags.id_set = ObjectIdSet(0b01);
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();
  robot.objects[1].flags.id_set = ObjectIdSet(0b10);
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();

  robot.objects[0].spheres[0].center << 0, 0, 0;
  robot.objects[0].spheres[0].radius = 0.09;
  robot.objects[1].spheres[0].center << 0.1, 0, 0;
  robot.objects[1].spheres[0].radius = 0.09;

  // A) Test masking with a sphere object.
  environment.ResizeBuffers(1);
  Sphere<Scalar> the_voxel{{Vector3<Scalar>(-0.1, 0, 0.1)}, {0.09}};
  environment.AddSphere(the_voxel, 42);
  environment.UpdateQueryStructures();

  result.Allocate(robot.objects.size());

  UpdateAlignedBoxes(Scalar{10}, robot);

  // 1 Default options.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot, QueryOptions(), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result.GetHasCollisions());

  // Without extra masking, expect two collision.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(
      environment, robot,
      QueryOptions().SetType(QueryOptions::kComputeObjectDistances), result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_EQ(result.GetObjectHits(0),
            ObjectIdSet(0b10) | ObjectIdSet(kDefaultVoxelMapId));
  EXPECT_EQ(result.GetObjectHits(1), ObjectIdSet(0b01));

  // Masking the environment should remove environment collisions.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot,
                             QueryOptions()
                                 .SetType(QueryOptions::kComputeObjectDistances)
                                 .SetExclusionSet(kVoxelMapIdSet),
                             result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_EQ(result.GetObjectHits(0), ObjectIdSet(0b10));
  EXPECT_EQ(result.GetObjectHits(1), ObjectIdSet(0b01));

  // Masking the middle sphere should remove all collisions.
  CC_MALLOC_COUNTER_INIT();
  status = ComputeCollisions(environment, robot,
                             QueryOptions()
                                 .SetType(QueryOptions::kComputeObjectDistances)
                                 .SetExclusionSet(ObjectIdSet(0b01)),
                             result);
  CC_MALLOC_COUNTER_EXPECT_NO_ALLOCATIONS();

  EXPECT_EQ(status.code(), absl::StatusCode::kOk);
  EXPECT_TRUE(result.GetObjectHits(0).Empty());
  EXPECT_TRUE(result.GetObjectHits(1).Empty());
}

REGISTER_TYPED_TEST_SUITE_P(
    CollisionCheckerTest, ComputeCollisionsArgumentCheckingTest,
    ComputeCollisionsCase01Test, ComputeCollisionsCase02Test,
    ComputeCollisionsCase03Test, ComputeCollisionsCase04Test,
    ComputeCollisionsCase05Test, ComputeCollisionsCase06Test,
    ComputeCollisionsCaseExtraExclusionBitmask);

typedef ::testing::Types<float, double> FPTypes;

INSTANTIATE_TYPED_TEST_SUITE_P(CollisionCheckerTestSuite, CollisionCheckerTest,
                               FPTypes);

}  // namespace
}  // namespace collision_checking
