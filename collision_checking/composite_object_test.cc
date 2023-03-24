#include "experimental/users/buschmann/collision_checking/composite_object.h"

#include <limits>
#include <vector>

#include "experimental/users/buschmann/collision_checking/options.h"
#include "experimental/users/buschmann/collision_checking/eigenmath.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"
#include "experimental/users/buschmann/collision_checking/test_utils.h"

namespace collision_checking {
namespace {

using ::blue::eigenmath::testing::IsApprox;

template <typename T>
class CompositeObjectTest : public ::testing::Test {};

TYPED_TEST_SUITE_P(CompositeObjectTest);

TYPED_TEST_P(CompositeObjectTest, CompositeObjectDistanceWorks) {
  using Scalar = TypeParam;
  using CompositeObjectCpu = CompositeObject<Scalar>;

  constexpr Scalar kExpectedPrecision =
      10 * std::numeric_limits<Scalar>::epsilon();

  CompositeObjectCpu null_object;
  CompositeObjectCpu object_a;
  CompositeObjectCpu object_b;
  const std::vector<QueryOptions> kAllOptions(
      {QueryOptions(),
       QueryOptions().SetType(QueryOptions::kComputeObjectDistances),
       QueryOptions().SetType(QueryOptions::kComputeContactPoints)});

  object_a.aabb.low.setConstant(-100);
  object_a.aabb.high.setConstant(100);
  object_b.aabb.low.setConstant(-100);
  object_b.aabb.high.setConstant(100);

  object_a.ResizeBuffers({.num_spheres = 1, .num_capsules = 1, .num_boxes = 1});

  CompositObjectDistanceResult<Scalar> result;

  constexpr Scalar kInfinity = std::numeric_limits<Scalar>::infinity();
  auto expect_contact_info_ok = [&](const QueryOptions& options,
                                    absl::SourceLocation loc =
                                        absl::SourceLocation::current()) {
    SCOPED_TRACE(
        absl::StrCat("Called from: ", loc.file_name(), ":", loc.line()));
    if (options.GetType() < QueryOptions::kComputeContactPoints) {
      return;
    }
    if (result.distance != kInfinity) {
      // For finite contact distance, expect consistency of distance, normal
      // and contact points.
      EXPECT_NEAR(result.distance,
                  std::copysign((result.contact_a - result.contact_b).norm(),
                                result.distance),
                  kExpectedPrecision)
          << "contact_a= " << result.contact_a.transpose()
          << " contact_b= " << result.contact_b.transpose();
      EXPECT_THAT(result.contact_b - result.contact_a,
                  IsApprox(result.a_normal_b * result.distance))
          << "contact_a= " << result.contact_a.transpose()
          << " contact_b= " << result.contact_b.transpose()
          << "a_normal_b= " << result.a_normal_b.transpose();
    }
  };

  for (const auto& options : kAllOptions) {
    SCOPED_TRACE(ToString(options));
    // Null objects are infinitely far apart.
    result = CompositeObjectDistance(null_object, null_object, options);
    EXPECT_EQ(result.distance, std::numeric_limits<Scalar>::infinity());
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(null_object, object_a, options);
    EXPECT_EQ(result.distance, std::numeric_limits<Scalar>::infinity());
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_a, null_object, options);
    EXPECT_EQ(result.distance, std::numeric_limits<Scalar>::infinity());
    expect_contact_info_ok(options);

    // Sphere / sphere distance is minimal & not colliding.
    object_a.spheres[0].radius = 0.1;
    object_a.spheres[0].center << 0, 0, 0;
    object_a.capsules[0].radius = 0.2;
    object_a.capsules[0].center << 4, 0, 0;
    object_a.capsules[0].direction << 0, 1, 0;
    object_a.capsules[0].half_length = 1;
    object_a.boxes[0].center << 10, 0, 0;
    object_a.boxes[0].box_rotation_world.setIdentity();
    object_a.boxes[0].half_lengths << 0.1, 0.1, 0.1;

    object_b = object_a;
    object_b.capsules[0].center[2] += 10;
    object_b.spheres[0].center[2] += 1;
    object_b.boxes[0].center << 10, 0, 10;
    result = CompositeObjectDistance(object_a, object_b, options);
    EXPECT_NEAR(result.distance, 0.8, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    EXPECT_NEAR(result.distance, 0.8, kExpectedPrecision);
    expect_contact_info_ok(options);

    // Sphere / sphere distance is minimal & colliding.
    object_b.spheres[0].center[2] -= 0.9;
    result = CompositeObjectDistance(object_a, object_b, options);
    EXPECT_NEAR(result.distance, -0.1, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    EXPECT_NEAR(result.distance, -0.1, kExpectedPrecision);
    expect_contact_info_ok(options);

    // Sphere / Capsule distance is minimal & not colliding.
    object_a.spheres[0].radius = 0.1;
    object_a.spheres[0].center << 0, 0, 0;
    object_a.capsules[0].radius = 0.2;
    object_a.capsules[0].center << 4, 0, -10;
    object_a.capsules[0].direction << 0, 1, 0;
    object_a.capsules[0].half_length = 1;
    object_b = object_a;
    object_b.spheres[0].center << 0, 0, 10;
    object_b.capsules[0].center << 4, 0, 3;
    object_b.boxes[0].center << 10, 0, 10;
    result = CompositeObjectDistance(object_a, object_b, options);
    EXPECT_NEAR(result.distance, 4.7, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    EXPECT_NEAR(result.distance, 4.7, kExpectedPrecision);
    expect_contact_info_ok(options);

    // Sphere / Capsule distance is minimal & colliding.
    object_a.spheres[0].radius = 0.1;
    object_a.spheres[0].center << 0, 0, 0;
    object_a.capsules[0].radius = 0.2;
    object_a.capsules[0].center << 4, 0, 0;
    object_a.capsules[0].direction << 0, 1, 0;
    object_a.capsules[0].half_length = 1;
    object_a.boxes[0].center << 10, 0, 0;
    object_b.capsules[0].center << 0, 0, 0.1;
    object_b.boxes[0].center << 10, 0, 10;
    result = CompositeObjectDistance(object_a, object_b, options);
    EXPECT_NEAR(result.distance, -0.2, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    EXPECT_NEAR(result.distance, -0.2, kExpectedPrecision);
    expect_contact_info_ok(options);

    // Capsule / Capsule distance is minimal & not colliding.
    object_a.spheres[0].radius = 0.1;
    object_a.spheres[0].center << 0, 0, -10;
    object_a.capsules[0].radius = 0.2;
    object_a.capsules[0].center << 4, 0, 0;
    object_a.capsules[0].direction << 0, 1, 0;
    object_a.capsules[0].half_length = 1;
    object_a.boxes[0].center << 10, 0, 0;
    object_b = object_a;
    object_b.spheres[0].center << 0, 0, 10;
    object_b.capsules[0].center << 4, 0, 1;
    object_b.boxes[0].center << 10, 0, 10;

    result = CompositeObjectDistance(object_a, object_b, options);
    EXPECT_NEAR(result.distance, 0.6, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    EXPECT_NEAR(result.distance, 0.6, kExpectedPrecision);
    expect_contact_info_ok(options);

    // Capsule / Capsule distance is minimal & colliding.
    object_a.spheres[0].radius = 0.1;
    object_a.spheres[0].center << 0, 0, -10;
    object_a.capsules[0].radius = 0.2;
    object_a.capsules[0].center << 4, 0, 0;
    object_a.capsules[0].direction << 0, 1, 0;
    object_a.capsules[0].half_length = 1;
    object_a.boxes[0].center << 10, 0, 0;
    object_b = object_a;
    object_b.spheres[0].center << 0, 0, 10;
    object_b.capsules[0].center << 4, 0, 0.2;
    object_b.boxes[0].center << 10, 0, 10;

    result = CompositeObjectDistance(object_a, object_b, options);
    EXPECT_NEAR(result.distance, -0.2, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    EXPECT_NEAR(result.distance, -0.2, kExpectedPrecision);
    expect_contact_info_ok(options);

    // Sphere / Box distance is minimal & not colliding.
    object_a.spheres[0].radius = 0.1;
    object_a.spheres[0].center << 0, 0, 0;
    object_a.capsules[0].radius = 0.2;
    object_a.capsules[0].center << 4, 0, 0;
    object_a.capsules[0].direction << 0, 1, 0;
    object_a.capsules[0].half_length = 1;
    object_a.boxes[0].center << 10, 0, 0;
    object_a.boxes[0].box_rotation_world.setIdentity();
    object_a.boxes[0].half_lengths << 0.1, 0.1, 0.1;

    object_b = object_a;
    object_b.capsules[0].center[2] += 10;
    object_b.spheres[0].center[2] += 10;
    object_b.boxes[0].center << 0.5, 0, 0;

    result = CompositeObjectDistance(object_a, object_b, options);
    // Distance = 0.3: box.center-sphere.center-box.half_length-sphere.radius.
    EXPECT_NEAR(result.distance, 0.3, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    EXPECT_NEAR(result.distance, 0.3, kExpectedPrecision);
    expect_contact_info_ok(options);

    // Sphere / Box distance is minimal & colliding.
    object_a.spheres[0].radius = 0.1;
    object_a.spheres[0].center << 0, 0, 0;
    object_a.capsules[0].radius = 0.2;
    object_a.capsules[0].center << 4, 0, 0;
    object_a.capsules[0].direction << 0, 1, 0;
    object_a.capsules[0].half_length = 1;
    object_a.boxes[0].center << 10, 0, 0;
    object_a.boxes[0].box_rotation_world.setIdentity();
    object_a.boxes[0].half_lengths << 0.1, 0.1, 0.1;

    object_b = object_a;
    object_b.capsules[0].center[2] += 10;
    object_b.spheres[0].center[2] += 10;
    object_b.boxes[0].center << 0.15, 0, 0;

    result = CompositeObjectDistance(object_a, object_b, options);
    // Distance = -0.05: box.center-sphere.center-box.half_length-sphere.radius.
    EXPECT_NEAR(result.distance, -0.05, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    EXPECT_NEAR(result.distance, -0.05, kExpectedPrecision);
    expect_contact_info_ok(options);

    // Box / Capsule distance is minimal & not colliding.
    object_a.spheres[0].radius = 0.1;
    object_a.spheres[0].center << 0, 0, 0;
    object_a.capsules[0].radius = 0.2;
    object_a.capsules[0].center << 4, 0, -10;
    object_a.capsules[0].direction << 0, 1, 0;
    object_a.capsules[0].half_length = 1;
    object_a.boxes[0].center << 10, 0, 0;
    object_a.boxes[0].box_rotation_world.setIdentity();
    object_a.boxes[0].half_lengths << 0.1, 0.1, 0.1;

    object_b = object_a;
    object_b.spheres[0].center << 0, 0, 10;
    object_b.capsules[0].center << 4, 0, 13;
    object_b.boxes[0].center << 4, 0, -10.6;
    result = CompositeObjectDistance(object_a, object_b, options);
    EXPECT_NEAR(result.distance, 0.3, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    EXPECT_NEAR(result.distance, 0.3, kExpectedPrecision);
    expect_contact_info_ok(options);

    // Box / Capsule distance is minimal & colliding.
    object_a.spheres[0].radius = 0.1;
    object_a.spheres[0].center << 0, 0, 0;
    object_a.capsules[0].radius = 0.2;
    object_a.capsules[0].center << 4, 0, -10;
    object_a.capsules[0].direction << 0, 1, 0;
    object_a.capsules[0].half_length = 1;
    object_a.boxes[0].center << 10, 0, 0;
    object_a.boxes[0].box_rotation_world.setIdentity();
    object_a.boxes[0].half_lengths << 0.1, 0.1, 0.1;

    object_b = object_a;
    object_b.spheres[0].center << 0, 0, 10;
    object_b.capsules[0].center << 4, 0, 13;
    object_b.boxes[0].center << 4, 0, -10.25;
    result = CompositeObjectDistance(object_a, object_b, options);
    EXPECT_NEAR(result.distance, -0.05, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    EXPECT_NEAR(result.distance, -0.05, kExpectedPrecision);
    expect_contact_info_ok(options);

    // Box / Box distance is minimal & not colliding.
    object_a.spheres[0].radius = 0.1;
    object_a.spheres[0].center << 0, 0, 0;
    object_a.capsules[0].radius = 0.2;
    object_a.capsules[0].center << 4, 0, -10;
    object_a.capsules[0].direction << 0, 1, 0;
    object_a.capsules[0].half_length = 1;
    object_a.boxes[0].center << 10, 0, 0;
    object_a.boxes[0].box_rotation_world.setIdentity();
    object_a.boxes[0].half_lengths << 0.1, 0.1, 0.1;

    object_b = object_a;
    object_b.spheres[0].center << 0, 0, 10;
    object_b.capsules[0].center << 4, 0, 13;
    object_b.boxes[0].center << 10, 0, 0.3;
    result = CompositeObjectDistance(object_a, object_b, options);
    EXPECT_NEAR(result.distance, 0.1, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    EXPECT_NEAR(result.distance, 0.1, kExpectedPrecision);
    expect_contact_info_ok(options);

    // Box / Box distance is minimal & colliding.
    object_a.spheres[0].radius = 0.1;
    object_a.spheres[0].center << 0, 0, 0;
    object_a.capsules[0].radius = 0.2;
    object_a.capsules[0].center << 4, 0, -10;
    object_a.capsules[0].direction << 0, 1, 0;
    object_a.capsules[0].half_length = 1;
    object_a.boxes[0].center << 10, 0, 0;
    object_a.boxes[0].box_rotation_world.setIdentity();
    object_a.boxes[0].half_lengths << 0.1, 0.1, 0.1;

    object_b = object_a;
    object_b.spheres[0].center << 0, 0, 10;
    object_b.capsules[0].center << 4, 0, 13;
    object_b.boxes[0].center << 10, 0, 0.2;
    result = CompositeObjectDistance(object_a, object_b, options);
    // Colliding boxes have zero distance (never negative).
    EXPECT_NEAR(result.distance, 0.0, kExpectedPrecision);
    expect_contact_info_ok(options);

    result = CompositeObjectDistance(object_b, object_a, options);
    // Colliding boxes have zero distance (never negative).
    EXPECT_NEAR(result.distance, 0.0, kExpectedPrecision);
    expect_contact_info_ok(options);
  }
}

TYPED_TEST_P(CompositeObjectTest, CollisionObjectsUpdateAlignedBoxesWorks) {
  using Scalar = TypeParam;
  using MovingObjectsCpu = CollisionObjects<Scalar>;

  constexpr Scalar kExpectedPrecision =
      10 * std::numeric_limits<Scalar>::epsilon();

  MovingObjectsCpu robot;
  robot.ResizeBuffers({{.num_spheres = 1, .num_capsules = 1},
                       {.num_spheres = 1, .num_capsules = 1},
                       {.num_spheres = 1}});
  robot.objects[0].spheres[0].center << 1, 2, 3;
  robot.objects[0].spheres[0].radius = 0.1;
  robot.objects[0].capsules[0].center << 0, 0, 0;
  robot.objects[0].capsules[0].direction << 0, 1, 0;
  robot.objects[0].capsules[0].radius = 0.2;
  robot.objects[0].capsules[0].half_length = 3.0;
  robot.objects[0].flags.inclusion_set = ObjectIdSet::kAll();

  robot.objects[1].spheres[0].center << 1, 2, 3;
  robot.objects[1].spheres[0].radius = 0.1;
  robot.objects[1].capsules[0].center << 0, 0, 0;
  robot.objects[1].capsules[0].direction << 0, 1, 0;
  robot.objects[1].capsules[0].radius = 0.2;
  robot.objects[1].capsules[0].half_length = 3.0;

  robot.objects[1].spheres[0].center -= Vector3<Scalar>(3, 2, 1);
  robot.objects[1].capsules[0].center -= Vector3<Scalar>(3, 2, 1);
  robot.objects[1].flags.inclusion_set = ObjectIdSet::kAll();
  // A huge object that is excluded from all collisions.
  robot.objects[2].flags.inclusion_set = ObjectIdSet::kEmpty();
  robot.objects[2].spheres[0].center.setZero();
  robot.objects[2].spheres[0].radius = Scalar(1000);
  robot.objects[2].aabb.low.setConstant(1);
  robot.objects[2].aabb.high.setConstant(-1);

  UpdateAlignedBoxes(Scalar{0.0}, robot);

  EXPECT_NEAR(robot.objects[0].aabb.low[0], -0.2, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.low[1], -3.2, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.low[2], -0.2, kExpectedPrecision);

  EXPECT_NEAR(robot.objects[0].aabb.high[0], 1.1, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.high[1], 3.2, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.high[2], 3.1, kExpectedPrecision);

  EXPECT_NEAR(robot.objects[1].aabb.low[0], -3.2, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.low[1], -5.2, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.low[2], -1.2, kExpectedPrecision);

  EXPECT_NEAR(robot.objects[1].aabb.high[0], -1.9, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.high[1], 1.2, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.high[2], 2.1, kExpectedPrecision);

  EXPECT_EQ(robot.objects[2].aabb.low[0], 1);
  EXPECT_EQ(robot.objects[2].aabb.low[1], 1);
  EXPECT_EQ(robot.objects[2].aabb.low[2], 1);
  EXPECT_EQ(robot.objects[2].aabb.high[0], -1);
  EXPECT_EQ(robot.objects[2].aabb.high[1], -1);
  EXPECT_EQ(robot.objects[2].aabb.high[2], -1);

  EXPECT_NEAR(robot.aabb.low[0], -3.2, kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.low[1], -5.2, kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.low[2], -1.2, kExpectedPrecision);

  EXPECT_NEAR(robot.aabb.high[0], 1.1, kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.high[1], 3.2, kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.high[2], 3.1, kExpectedPrecision);

  // Same geometry, but with padding.
  UpdateAlignedBoxes(Scalar{0.1}, robot);

  EXPECT_NEAR(robot.objects[0].aabb.low[0], -0.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.low[1], -3.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.low[2], -0.3, kExpectedPrecision);

  EXPECT_NEAR(robot.objects[0].aabb.high[0], 1.2, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.high[1], 3.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.high[2], 3.2, kExpectedPrecision);

  EXPECT_NEAR(robot.objects[1].aabb.low[0], -3.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.low[1], -5.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.low[2], -1.3, kExpectedPrecision);

  EXPECT_NEAR(robot.objects[1].aabb.high[0], -1.8, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.high[1], 1.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.high[2], 2.2, kExpectedPrecision);

  EXPECT_EQ(robot.objects[2].aabb.low[0], 1);
  EXPECT_EQ(robot.objects[2].aabb.low[1], 1);
  EXPECT_EQ(robot.objects[2].aabb.low[2], 1);
  EXPECT_EQ(robot.objects[2].aabb.high[0], -1);
  EXPECT_EQ(robot.objects[2].aabb.high[1], -1);
  EXPECT_EQ(robot.objects[2].aabb.high[2], -1);

  EXPECT_NEAR(robot.aabb.low[0], -3.3, kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.low[1], -5.3, kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.low[2], -1.3, kExpectedPrecision);

  EXPECT_NEAR(robot.aabb.high[0], 1.2, kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.high[1], 3.3, kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.high[2], 3.2, kExpectedPrecision);

  // Exclude object[0] from environment collisions.
  robot.objects[0].flags.inclusion_set = ~kVoxelMapIdSet;
  UpdateAlignedBoxes(Scalar{0.1}, robot);
  EXPECT_NEAR(robot.objects[0].aabb.low[0], -0.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.low[1], -3.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.low[2], -0.3, kExpectedPrecision);

  EXPECT_NEAR(robot.objects[0].aabb.high[0], 1.2, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.high[1], 3.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[0].aabb.high[2], 3.2, kExpectedPrecision);

  EXPECT_NEAR(robot.objects[1].aabb.low[0], -3.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.low[1], -5.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.low[2], -1.3, kExpectedPrecision);

  EXPECT_NEAR(robot.objects[1].aabb.high[0], -1.8, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.high[1], 1.3, kExpectedPrecision);
  EXPECT_NEAR(robot.objects[1].aabb.high[2], 2.2, kExpectedPrecision);

  EXPECT_EQ(robot.objects[2].aabb.low[0], 1);
  EXPECT_EQ(robot.objects[2].aabb.low[1], 1);
  EXPECT_EQ(robot.objects[2].aabb.low[2], 1);
  EXPECT_EQ(robot.objects[2].aabb.high[0], -1);
  EXPECT_EQ(robot.objects[2].aabb.high[1], -1);
  EXPECT_EQ(robot.objects[2].aabb.high[2], -1);

  EXPECT_NEAR(robot.aabb.low[0], robot.objects[1].aabb.low[0],
              kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.low[1], robot.objects[1].aabb.low[1],
              kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.low[2], robot.objects[1].aabb.low[2],
              kExpectedPrecision);

  EXPECT_NEAR(robot.aabb.high[0], robot.objects[1].aabb.high[0],
              kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.high[1], robot.objects[1].aabb.high[1],
              kExpectedPrecision);
  EXPECT_NEAR(robot.aabb.high[2], robot.objects[1].aabb.high[2],
              kExpectedPrecision);
}

REGISTER_TYPED_TEST_SUITE_P(CompositeObjectTest, CompositeObjectDistanceWorks,
                            CollisionObjectsUpdateAlignedBoxesWorks);

typedef ::testing::Types<float, double> FPTypes;

INSTANTIATE_TYPED_TEST_SUITE_P(CompositeObjectTestSuite, CompositeObjectTest,
                               FPTypes);

}  // namespace
}  // namespace collision_checking
