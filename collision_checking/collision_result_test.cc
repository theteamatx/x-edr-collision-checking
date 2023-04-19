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

#include "collision_checking/collision_result.h"

#include "collision_checking/eigenmath.h"
#include "collision_checking/test_utils.h"
#include "gtest/gtest.h"

namespace collision_checking {
namespace {

using ::testing::MinimumGeometryInfoIs;

template <typename T>
class CollisionResultTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(CollisionResultTest);

TYPED_TEST_P(CollisionResultTest, CollisionResultInterface) {
  using Scalar = TypeParam;
  using CollisionResultCpu = CollisionResult<Scalar>;

  CollisionResultCpu result(0, QueryOptions());

  // Default constructed result.
  EXPECT_EQ(result.GetObjectCount(), 0);
  EXPECT_FALSE(result.GetHasCollisions());
  result.SetHasCollisions(true);
  EXPECT_TRUE(result.GetHasCollisions());
  EXPECT_EQ(result.GetMinimumDistance(),
            std::numeric_limits<Scalar>::infinity());
  // Default options do not include contact point computation.
  EXPECT_FALSE(result.MemoryIsAllocatedFor(
      QueryOptions().SetType(QueryOptions::kComputeContactPoints)));

  // Resize should work.
  result.Allocate(42);
  EXPECT_EQ(result.GetObjectCount(), 42);
  // Default options do not include contact point computation.
  EXPECT_FALSE(result.MemoryIsAllocatedFor(
      QueryOptions().SetType(QueryOptions::kComputeContactPoints)));
  EXPECT_FALSE(result.GetHasCollisions());
  EXPECT_EQ(result.GetMinimumDistance(),
            std::numeric_limits<Scalar>::infinity());
  for (int obj = 0; obj < result.GetObjectCount(); obj++) {
    EXPECT_EQ(result.GetObjectHits(obj), ObjectIdSet::kEmpty());
    EXPECT_EQ(result.GetMinimumDistance(obj),
              std::numeric_limits<Scalar>::infinity());
  }
  result.GetObjectHits(24) = ObjectIdSet(0xabcd);
  EXPECT_EQ(result.GetObjectHits(24), ObjectIdSet(0xabcd));

  EXPECT_FALSE(result.GetHasCollisions());
  result.SetHasCollisions(true);
  EXPECT_TRUE(result.GetHasCollisions());

  result.SetMinimumDistance(2);
  EXPECT_EQ(result.GetMinimumDistance(), 2);

  result.SetMinimumObstacleInfo(Vector3<Scalar>(1, 2, 3), 42);
  EXPECT_THAT(result.GetMinimumObstacleInfo(),
              MinimumGeometryInfoIs(Vector3<Scalar>(1, 2, 3), 42));

  // Reset to default.
  result.Reset();
  EXPECT_EQ(result.GetMinimumDistance(),
            std::numeric_limits<Scalar>::infinity());
  for (int obj = 0; obj < result.GetObjectCount(); obj++) {
    EXPECT_TRUE(result.GetObjectHits(obj).Empty());
    EXPECT_EQ(result.GetMinimumDistance(obj),
              std::numeric_limits<Scalar>::infinity());
  }
  EXPECT_FALSE(result.GetMinimumObstacleInfo().has_minimum_distance);

  // Allocation for contact point computation.
  CollisionResultCpu result_with_contacts(
      0, QueryOptions().SetType(QueryOptions::kComputeContactPoints));
  result_with_contacts.Allocate(42);
  EXPECT_EQ(result_with_contacts.GetObjectCount(), 42);
  EXPECT_TRUE(result_with_contacts.MemoryIsAllocatedFor(
      QueryOptions().SetType(QueryOptions::kComputeContactPoints)));
  for (int obj = 0; obj < result_with_contacts.GetObjectCount(); obj++) {
    auto info = result_with_contacts.GetContactPointInfo(obj);
    EXPECT_TRUE(info.other_id_set.Empty());
    for (int i = 0; i < 3; i++) {
      EXPECT_EQ(info.contact_point_this[i],
                std::numeric_limits<Scalar>::infinity());
      EXPECT_EQ(info.contact_point_other[i],
                std::numeric_limits<Scalar>::infinity());
      EXPECT_EQ(info.normal_this_other[i], Scalar(0));
    }
  }
}

REGISTER_TYPED_TEST_SUITE_P(CollisionResultTest, CollisionResultInterface);

typedef ::testing::Types<float, double> FPTypes;

INSTANTIATE_TYPED_TEST_SUITE_P(CollisionResultTestSuite, CollisionResultTest,
                               FPTypes);

}  // namespace
}  // namespace collision_checking
