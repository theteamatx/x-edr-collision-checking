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

#include "collision_checking/geometry.h"

#include <cmath>
#include <limits>

#include "collision_checking/eigenmath.h"
#include "collision_checking/object_id.h"
#include "collision_checking/test_utils.h"
#include "eigenmath/matchers.h"
#include "eigenmath/rotation_utils.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace collision_checking {
namespace {

using ::eigenmath::testing::IsApprox;

template <typename T>
class GeometryTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(GeometryTest);

TYPED_TEST_P(GeometryTest, AlignedBoxDoOverlap) {
  using Scalar = TypeParam;
  AlignedBox<Scalar> box_a{Vector3<Scalar>(-1, -2, -3),
                           Vector3<Scalar>(1, 2, 3)};
  // Loop over all permutations of shifts of box_b relative to
  // box_a by multiples (or fractions) of box_a's dimension.
  std::vector<Scalar> relative_shift_values({-2, -0.5, 0, 0.5, 2});
  const int values_count = relative_shift_values.size();
  const int variant_count = std::pow(values_count, 3);
  for (int v = 0; v < variant_count; v++) {
    int index_0 = v % values_count;
    int index_1 = (v / values_count) % values_count;
    int index_2 = (v / values_count / values_count) % values_count;
    Vector3<Scalar> shift(relative_shift_values[index_0],
                          relative_shift_values[index_1],
                          relative_shift_values[index_2]);
    const bool expect_overlap =
        shift.template lpNorm<Eigen::Infinity>() <= Scalar{1};

    AlignedBox<Scalar> box_b;
    box_b.low = box_a.low + shift.cwiseProduct(box_a.high - box_a.low);
    box_b.high = box_a.high + shift.cwiseProduct(box_a.high - box_a.low);

    EXPECT_EQ(DoOverlap(box_a, box_b), expect_overlap)
        << "\nbox_a.low= " << box_a.low.transpose()
        << "\nbox_a.high= " << box_a.high.transpose()
        << "\nbox_b.low= " << box_b.low.transpose()
        << "\nbox_b.high= " << box_b.high.transpose();
  }
}

TYPED_TEST_P(GeometryTest, AlignedBoxIsInside) {
  using Scalar = TypeParam;
  AlignedBox<Scalar> box{Vector3<Scalar>(-1, -2, -3), Vector3<Scalar>(1, 2, 3)};
  // Loop over all permutations of shifts of sphere relative to
  // box by multiples (or fractions) of the box's dimension.
  std::vector<Scalar> relative_shift_values({-2, -0.5, 0, 0.5, 2});
  const int values_count = relative_shift_values.size();
  const int variant_count = std::pow(values_count, 3);
  for (int v = 0; v < variant_count; v++) {
    int index_0 = v % values_count;
    int index_1 = (v / values_count) % values_count;
    int index_2 = (v / values_count / values_count) % values_count;
    Vector3<Scalar> shift(relative_shift_values[index_0],
                          relative_shift_values[index_1],
                          relative_shift_values[index_2]);
    const bool expect_inside =
        shift.template lpNorm<Eigen::Infinity>() <= Scalar{1};

    Vector3<Scalar> point = Scalar{0.5} * (box.low + box.high) +
                            shift.cwiseProduct(box.high - box.low);

    EXPECT_EQ(IsInside(box, point), expect_inside)
        << "\nbox.low= " << box.low.transpose()
        << "\nbox.high= " << box.high.transpose()
        << "\npoint= " << point.transpose();
  }
}

TYPED_TEST_P(GeometryTest, AlignedBoxSetEmptySize) {
  using Scalar = TypeParam;
  AlignedBox<Scalar> box{Vector3<Scalar>(-1, -2, -3), Vector3<Scalar>(1, 2, 3)};
  Vector3<Scalar> point(0, 0, 0);
  EXPECT_TRUE(IsInside(box, point));
  SetEmptySize(&box);
  EXPECT_FALSE(IsInside(box, point));
}

TYPED_TEST_P(GeometryTest, AlignedBoxGrowAlignedBoxAroundSphere) {
  using Scalar = TypeParam;
  AlignedBox<Scalar> box;
  Sphere<Scalar> sphere;
  sphere.center << 1, 2, 3;
  sphere.radius = 3;
  GrowAlignedBoxAround(sphere, &box);
  EXPECT_EQ(box.low[0], 1 - 3);
  EXPECT_EQ(box.low[1], 2 - 3);
  EXPECT_EQ(box.low[2], 3 - 3);

  EXPECT_EQ(box.high[0], 1 + 3);
  EXPECT_EQ(box.high[1], 2 + 3);
  EXPECT_EQ(box.high[2], 3 + 3);
}

TYPED_TEST_P(GeometryTest, AlignedBoxGrowAlignedBoxAroundCapsule) {
  using Scalar = TypeParam;
  AlignedBox<Scalar> box;
  Capsule<Scalar> capsule;
  capsule.center << 1, 2, 3;
  capsule.direction << 1, 0, 0;
  capsule.half_length = 1;
  capsule.radius = 3;
  GrowAlignedBoxAround(capsule, &box);
  EXPECT_EQ(box.low[0], 1 - 3 - 1);
  EXPECT_EQ(box.low[1], 2 - 3);
  EXPECT_EQ(box.low[2], 3 - 3);

  EXPECT_EQ(box.high[0], 1 + 3 + 1);
  EXPECT_EQ(box.high[1], 2 + 3);
  EXPECT_EQ(box.high[2], 3 + 3);

  SetEmptySize(&box);
  capsule.direction << 0, 1, 0;
  GrowAlignedBoxAround(capsule, &box);
  EXPECT_EQ(box.low[0], 1 - 3);
  EXPECT_EQ(box.low[1], 2 - 3 - 1);
  EXPECT_EQ(box.low[2], 3 - 3);

  EXPECT_EQ(box.high[0], 1 + 3);
  EXPECT_EQ(box.high[1], 2 + 3 + 1);
  EXPECT_EQ(box.high[2], 3 + 3);

  SetEmptySize(&box);
  capsule.direction << 0, 0, 1;
  GrowAlignedBoxAround(capsule, &box);
  EXPECT_EQ(box.low[0], 1 - 3);
  EXPECT_EQ(box.low[1], 2 - 3);
  EXPECT_EQ(box.low[2], 3 - 3 - 1);

  EXPECT_EQ(box.high[0], 1 + 3);
  EXPECT_EQ(box.high[1], 2 + 3);
  EXPECT_EQ(box.high[2], 3 + 3 + 1);
}

TYPED_TEST_P(GeometryTest, AlignedBoxGrowAlignedBoxAroundBox) {
  using Scalar = TypeParam;
  const Scalar kExpectedPrecision = Eigen::NumTraits<Scalar>::dummy_precision();

  AlignedBox<Scalar> aabb;
  Box<Scalar> box;
  box.center << 1, 2, 3;
  box.box_rotation_world.setIdentity();
  box.half_lengths << Scalar(0.3), Scalar(0.2), Scalar(0.1);
  GrowAlignedBoxAround(box, &aabb);
  EXPECT_EQ(aabb.low[0], Scalar(0.7));
  EXPECT_EQ(aabb.low[1], Scalar(1.8));
  EXPECT_EQ(aabb.low[2], Scalar(2.9));

  EXPECT_EQ(aabb.high[0], Scalar(1.3));
  EXPECT_EQ(aabb.high[1], Scalar(2.2));
  EXPECT_EQ(aabb.high[2], Scalar(3.1));

  SetEmptySize(&aabb);
  box.box_rotation_world.col(0) = Vector3<Scalar>::UnitX();
  box.box_rotation_world.col(1) = Vector3<Scalar>::UnitZ();
  box.box_rotation_world.col(2) = -Vector3<Scalar>::UnitY();
  GrowAlignedBoxAround(box, &aabb);
  EXPECT_EQ(aabb.low[0], Scalar(0.7));
  EXPECT_EQ(aabb.low[1], Scalar(1.9));
  EXPECT_EQ(aabb.low[2], Scalar(2.8));

  EXPECT_EQ(aabb.high[0], Scalar(1.3));
  EXPECT_EQ(aabb.high[1], Scalar(2.1));
  EXPECT_EQ(aabb.high[2], Scalar(3.2));

  // Case with 3d rotation and offset. Thev values are arbitrary, but ensure
  // the box is not aligned and the offset has positive and negative values.
  SetEmptySize(&aabb);
  box.center << 1.2, -1.4, 1.2345;
  eigenmath::RotationFromRPY(Scalar(-0.123), Scalar(1.23), Scalar(0.5),
                             &box.box_rotation_world);
  GrowAlignedBoxAround(box, &aabb);
  // Brute-force calculation of aligned bounding box by first computing all
  // corners of the box.
  Vector3<Scalar> expected_low =
      Vector3<Scalar>::Constant(std::numeric_limits<Scalar>::max());
  Vector3<Scalar> expected_high =
      Vector3<Scalar>::Constant(std::numeric_limits<Scalar>::lowest());
  const Vector3<Scalar>& h = box.half_lengths;
  const std::array<Vector3<Scalar>, 8> box_corners = {
      Vector3<Scalar>(+h[0], -h[1], +h[2]),
      Vector3<Scalar>(+h[0], +h[1], +h[2]),
      Vector3<Scalar>(+h[0], +h[1], -h[2]),
      Vector3<Scalar>(+h[0], -h[1], -h[2]),
      Vector3<Scalar>(-h[0], -h[1], +h[2]),
      Vector3<Scalar>(-h[0], +h[1], +h[2]),
      Vector3<Scalar>(-h[0], +h[1], -h[2]),
      Vector3<Scalar>(-h[0], -h[1], -h[2]),
  };
  for (const auto& corner : box_corners) {
    const Vector3<Scalar> corner_in_world =
        box.box_rotation_world.transpose() * corner + box.center;
    expected_high = expected_high.cwiseMax(corner_in_world);
    expected_low = expected_low.cwiseMin(corner_in_world);
  }

  EXPECT_THAT(aabb.low, IsApprox(expected_low, kExpectedPrecision));
  EXPECT_THAT(aabb.high, IsApprox(expected_high, kExpectedPrecision));
}

TYPED_TEST_P(GeometryTest, AlignedBoxSphereDoOverlap) {
  using Scalar = TypeParam;
  AlignedBox<Scalar> box{Vector3<Scalar>(-1, -2, -3), Vector3<Scalar>(1, 2, 3)};
  // Loop over all permutations of shifts of sphere relative to
  // box by multiples (or fractions) of the box's dimension.
  std::vector<Scalar> relative_shift_values({-2, -0.5, 0, 0.5, 2});
  const int values_count = relative_shift_values.size();
  const int variant_count = std::pow(values_count, 3);
  for (int v = 0; v < variant_count; v++) {
    int index_0 = v % values_count;
    int index_1 = (v / values_count) % values_count;
    int index_2 = (v / values_count / values_count) % values_count;
    Vector3<Scalar> shift(relative_shift_values[index_0],
                          relative_shift_values[index_1],
                          relative_shift_values[index_2]);

    Sphere<Scalar> sphere;
    sphere.center = Scalar{0.5} * (box.low + box.high) +
                    shift.cwiseProduct(box.high - box.low);

    sphere.radius = 0.0;
    EXPECT_EQ(IsInside(box, sphere.center), DoOverlap(box, sphere))
        << "\nbox.low= " << box.low.transpose()
        << "\nbox.high= " << box.high.transpose()
        << "\nsphere.center= " << sphere.center.transpose()
        << "\nsphere.radius= " << sphere.radius;

    sphere.radius = 0.1;
    AlignedBox<Scalar> sphere_box;
    sphere_box.low = sphere.center - Vector3<Scalar>::Constant(sphere.radius);
    sphere_box.high = sphere.center + Vector3<Scalar>::Constant(sphere.radius);

    EXPECT_EQ(DoOverlap(box, sphere_box), DoOverlap(box, sphere))
        << "\nbox.low= " << box.low.transpose()
        << "\nbox.high= " << box.high.transpose()
        << "\nsphere.center= " << sphere.center.transpose()
        << "\nsphere.radius= " << sphere.radius;
  }
}

REGISTER_TYPED_TEST_SUITE_P(GeometryTest, AlignedBoxDoOverlap,
                            AlignedBoxIsInside, AlignedBoxSetEmptySize,
                            AlignedBoxGrowAlignedBoxAroundSphere,
                            AlignedBoxGrowAlignedBoxAroundCapsule,
                            AlignedBoxGrowAlignedBoxAroundBox,
                            AlignedBoxSphereDoOverlap);

typedef ::testing::Types<float, double> FPTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(GeometryTestSuite, GeometryTest, FPTypes);

}  // namespace
}  // namespace collision_checking
