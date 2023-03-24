#include "experimental/users/buschmann/collision_checking/geometry_shape_conversion.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "experimental/users/buschmann/collision_checking/composite_object.h"
#include "experimental/users/buschmann/collision_checking/eigenmath.h"
#include "experimental/users/buschmann/collision_checking/geometry.h"
#include "experimental/users/buschmann/collision_checking/geometry_shapes/capsule.h"
#include "experimental/users/buschmann/collision_checking/test_utils.h"
#include "third_party/absl/status/status.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"

namespace collision_checking {
namespace {
using ::blue::eigenmath::testing::IsApprox;

typedef ::testing::Types<float, double> FPTypes;
template <typename T>
class GeometryShapeConversionDeathTest : public ::testing::Test {
 public:
};

TYPED_TEST_SUITE_P(GeometryShapeConversionDeathTest);

TYPED_TEST_P(GeometryShapeConversionDeathTest, SphereConversion) {
  geometry_shapes::Capsule capsule(0.1, 0.2);
  EXPECT_DEATH(GeometryShapesSphereToSphere(Pose3d::Identity(), capsule, 0.1),
               "geometry_shapes::ShapeType::SPHERE");
}

TYPED_TEST_P(GeometryShapeConversionDeathTest, CapsuleConversion) {
  geometry_shapes::Sphere sphere(0.1);
  EXPECT_DEATH(GeometryShapesCapsuleToCapsule(Pose3d::Identity(), sphere, 0.1),
               "geometry_shapes::ShapeType::CAPSULE");
}

TYPED_TEST_P(GeometryShapeConversionDeathTest, BoxConversion) {
  geometry_shapes::Sphere sphere(0.1);
  EXPECT_DEATH(GeometryShapesBoxToBox(Pose3d::Identity(), sphere, 0.1),
               "geometry_shapes::ShapeType::BOX");
}

REGISTER_TYPED_TEST_SUITE_P(GeometryShapeConversionDeathTest, SphereConversion,
                            CapsuleConversion, BoxConversion);

INSTANTIATE_TYPED_TEST_SUITE_P(GeometryShapeConversionTestDeathSuite,
                               GeometryShapeConversionDeathTest, FPTypes);

template <typename T>
class GeometryShapeConversionTest : public ::testing::Test {
 public:
};

TYPED_TEST_SUITE_P(GeometryShapeConversionTest);

TYPED_TEST_P(GeometryShapeConversionTest, SphereConversion) {
  using Scalar = TypeParam;
  constexpr Scalar kEpsilon = 10 * std::numeric_limits<Scalar>::epsilon();

  geometry_shapes::Sphere shape(1.0);

  Sphere<Scalar> sphere =
      GeometryShapesSphereToSphere<Scalar>(Pose3d::Identity(), shape, 0.2);

  EXPECT_NEAR(sphere.radius, Scalar(0.2 + 1.0), kEpsilon);

  using CompositeObjectCpu = CompositeObject<Scalar>;
  PrimitivesCount primitives_count;
  EXPECT_EQ(AddGeometryShapesToPrimitivesCount(shape, primitives_count).code(),
            absl::StatusCode::kOk);
  EXPECT_EQ(primitives_count.num_spheres, 1);
  EXPECT_EQ(primitives_count.num_capsules, 0);
  EXPECT_EQ(primitives_count.num_boxes, 0);

  PrimitivesCount indices;
  CompositeObjectCpu objects;
  objects.ResizeBuffers(primitives_count);
  EXPECT_EQ(AddGeometryShapesToCompositeObject(Pose3d::Identity(), shape,
                                               Scalar(0.1), objects, indices)
                .code(),
            absl::StatusCode::kOk);
  EXPECT_EQ(indices.num_spheres, 1);
  EXPECT_EQ(indices.num_capsules, 0);
  EXPECT_EQ(indices.num_boxes, 0);
  ASSERT_EQ(objects.spheres.size(), 1);
  EXPECT_NEAR(objects.spheres[0].radius, Scalar(0.1 + 1.0), kEpsilon);
}

TYPED_TEST_P(GeometryShapeConversionTest, CapsuleConversion) {
  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  constexpr Scalar kEpsilon = 10 * std::numeric_limits<Scalar>::epsilon();

  geometry_shapes::Capsule shape(2.0, 1.0);
  Pose3d local_transform = Pose3d(SO3d(-1, -2, -3), Vector3d(2, 4, 6));
  shape.SetLocalTransform(local_transform);
  Capsule<Scalar> capsule =
      GeometryShapesCapsuleToCapsule<Scalar>(Pose3d::Identity(), shape, 0.2);

  EXPECT_NEAR(capsule.radius, Scalar(0.2 + 1.0), kEpsilon);
  EXPECT_NEAR(capsule.half_length, Scalar(1.0), kEpsilon);
  EXPECT_THAT(capsule.center, IsApprox(Vector3(2, 4, 6), kEpsilon));
  EXPECT_THAT(
      capsule.direction,
      IsApprox(local_transform.rotationMatrix().col(2).template cast<Scalar>(),
               kEpsilon));

  using CompositeObjectCpu = CompositeObject<Scalar>;
  PrimitivesCount primitives_count;
  EXPECT_EQ(AddGeometryShapesToPrimitivesCount(shape, primitives_count).code(),
            absl::StatusCode::kOk);
  EXPECT_EQ(primitives_count.num_spheres, 0);
  EXPECT_EQ(primitives_count.num_capsules, 1);
  EXPECT_EQ(primitives_count.num_boxes, 0);

  PrimitivesCount indices;
  CompositeObjectCpu objects;
  objects.ResizeBuffers(primitives_count);
  EXPECT_EQ(AddGeometryShapesToCompositeObject(Pose3d::Identity(), shape,
                                               Scalar(0.1), objects, indices)
                .code(),
            absl::StatusCode::kOk);
  EXPECT_EQ(indices.num_spheres, 0);
  EXPECT_EQ(indices.num_capsules, 1);
  EXPECT_EQ(indices.num_boxes, 0);
  ASSERT_EQ(objects.capsules.size(), 1);
  EXPECT_NEAR(objects.capsules[0].radius, Scalar(0.1 + 1.0), kEpsilon);
  EXPECT_NEAR(objects.capsules[0].half_length, Scalar(1.0), kEpsilon);
  EXPECT_THAT(objects.capsules[0].center, IsApprox(Vector3(2, 4, 6), kEpsilon));
  EXPECT_THAT(
      objects.capsules[0].direction,
      IsApprox(local_transform.rotationMatrix().col(2).template cast<Scalar>(),
               kEpsilon));
}

TYPED_TEST_P(GeometryShapeConversionTest, BoxConversion) {
  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  constexpr Scalar kEpsilon = 10 * std::numeric_limits<Scalar>::epsilon();
  geometry_shapes::Box shape(Vector3d(1, 2, 3));
  Pose3d local_transform = Pose3d(SO3d(-1, -2, -3), Vector3d(2, 4, 6));
  shape.SetLocalTransform(local_transform);
  Box<Scalar> box =
      GeometryShapesBoxToBox<Scalar>(Pose3d::Identity(), shape, 0.2);

  EXPECT_THAT(
      box.half_lengths,
      IsApprox(Vector3(1 * 0.5 + 0.2, 2 * 0.5 + 0.2, 3 * 0.5 + 0.2), kEpsilon));
  EXPECT_THAT(box.center,
              IsApprox(local_transform.translation().template cast<Scalar>(),
                       kEpsilon));
  EXPECT_THAT(
      box.box_rotation_world,
      IsApprox(
          local_transform.rotationMatrix().transpose().template cast<Scalar>(),
          kEpsilon));

  using CompositeObjectCpu = CompositeObject<Scalar>;
  PrimitivesCount primitives_count;
  EXPECT_EQ(AddGeometryShapesToPrimitivesCount(shape, primitives_count).code(),
            absl::StatusCode::kOk);
  EXPECT_EQ(primitives_count.num_spheres, 0);
  EXPECT_EQ(primitives_count.num_capsules, 0);
  EXPECT_EQ(primitives_count.num_boxes, 1);

  PrimitivesCount indices;
  CompositeObjectCpu objects;
  objects.ResizeBuffers(primitives_count);
  EXPECT_EQ(AddGeometryShapesToCompositeObject(Pose3d::Identity(), shape,
                                               Scalar(0.1), objects, indices)
                .code(),
            absl::StatusCode::kOk);
  EXPECT_EQ(indices.num_spheres, 0);
  EXPECT_EQ(indices.num_capsules, 0);
  EXPECT_EQ(indices.num_boxes, 1);
  ASSERT_EQ(objects.boxes.size(), 1);
  EXPECT_THAT(
      objects.boxes[0].half_lengths,
      IsApprox(Vector3(1 * 0.5 + 0.1, 2 * 0.5 + 0.1, 3 * 0.5 + 0.1), kEpsilon));
  EXPECT_THAT(objects.boxes[0].center,
              IsApprox(local_transform.translation().template cast<Scalar>(),
                       kEpsilon));
  EXPECT_THAT(
      objects.boxes[0].box_rotation_world,
      IsApprox(
          local_transform.rotationMatrix().transpose().template cast<Scalar>(),
          kEpsilon));
}

TYPED_TEST_P(GeometryShapeConversionTest, CompositeShapeConversion) {
  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  constexpr Scalar kEpsilon = 10 * std::numeric_limits<Scalar>::epsilon();

  geometry_shapes::Sphere sphere_shape(1.0);

  geometry_shapes::Capsule capsule_shape(2.0, 1.0);
  Pose3d local_transform = Pose3d(SO3d(-1, -2, -3), Vector3d(2, 4, 6));
  capsule_shape.SetLocalTransform(local_transform);

  geometry_shapes::Box box_shape(Vector3d(1, 2, 3));
  box_shape.SetLocalTransform(local_transform);

  std::vector<std::unique_ptr<geometry_shapes::ShapeBase>> sub_shapes;
  sub_shapes.emplace_back(sphere_shape.Clone());
  sub_shapes.emplace_back(capsule_shape.Clone());
  sub_shapes.emplace_back(box_shape.Clone());
  geometry_shapes::CompositeShape composite_shape(std::move(sub_shapes));

  using CompositeObjectCpu = CompositeObject<Scalar>;
  PrimitivesCount primitives_count;
  EXPECT_EQ(
      AddGeometryShapesToPrimitivesCount(composite_shape, primitives_count)
          .code(),
      absl::StatusCode::kOk);
  EXPECT_EQ(primitives_count.num_spheres, 1);
  EXPECT_EQ(primitives_count.num_capsules, 1);
  EXPECT_EQ(primitives_count.num_boxes, 1);

  PrimitivesCount indices;
  CompositeObjectCpu objects;
  objects.ResizeBuffers(primitives_count);
  EXPECT_EQ(
      AddGeometryShapesToCompositeObject(Pose3d::Identity(), composite_shape,
                                         Scalar(0.1), objects, indices)
          .code(),
      absl::StatusCode::kOk);
  EXPECT_EQ(indices.num_spheres, 1);
  EXPECT_EQ(indices.num_capsules, 1);
  EXPECT_EQ(indices.num_boxes, 1);

  ASSERT_EQ(objects.spheres.size(), 1);
  EXPECT_NEAR(objects.spheres[0].radius, Scalar(0.1 + 1.0), kEpsilon);

  ASSERT_EQ(objects.capsules.size(), 1);

  EXPECT_NEAR(objects.capsules[0].radius, Scalar(0.1 + 1.0), kEpsilon);
  EXPECT_NEAR(objects.capsules[0].half_length, Scalar(1.0), kEpsilon);
  EXPECT_THAT(objects.capsules[0].center, IsApprox(Vector3(2, 4, 6), kEpsilon));
  EXPECT_THAT(
      objects.capsules[0].direction,
      IsApprox(local_transform.rotationMatrix().col(2).template cast<Scalar>(),
               kEpsilon));

  ASSERT_EQ(objects.boxes.size(), 1);
  EXPECT_THAT(
      objects.boxes[0].half_lengths,
      IsApprox(Vector3(1 * 0.5 + 0.1, 2 * 0.5 + 0.1, 3 * 0.5 + 0.1), kEpsilon));
  EXPECT_THAT(objects.boxes[0].center,
              IsApprox(local_transform.translation().template cast<Scalar>(),
                       kEpsilon));
  EXPECT_THAT(
      objects.boxes[0].box_rotation_world,
      IsApprox(
          local_transform.rotationMatrix().transpose().template cast<Scalar>(),
          kEpsilon));
}

TYPED_TEST_P(GeometryShapeConversionTest,
             NestedCompositeShapePropagatesLocalTransform) {
  using Scalar = TypeParam;
  using Vector3 = Vector3<Scalar>;
  constexpr Scalar kEpsilon = 10 * std::numeric_limits<Scalar>::epsilon();

  geometry_shapes::Sphere sphere_shape(1.0);
  geometry_shapes::Capsule capsule_shape(2.0, 1.0);
  Pose3d local_transform = Pose3d(SO3d(), Vector3d(1, 0, 0));
  geometry_shapes::Box box_shape(Vector3d(1, 2, 3));

  sphere_shape.SetLocalTransform(local_transform);
  capsule_shape.SetLocalTransform(local_transform);
  box_shape.SetLocalTransform(local_transform);

  std::vector<std::unique_ptr<geometry_shapes::ShapeBase>> sub_shapes;
  sub_shapes.emplace_back(sphere_shape.Clone());
  sub_shapes.emplace_back(capsule_shape.Clone());
  sub_shapes.emplace_back(box_shape.Clone());
  Pose3d nested_local_transform = local_transform * local_transform;
  geometry_shapes::CompositeShape nested_composite_shape(std::move(sub_shapes));
  nested_composite_shape.SetLocalTransform(local_transform);

  sub_shapes.clear();
  sub_shapes.emplace_back(sphere_shape.Clone());
  sub_shapes.emplace_back(capsule_shape.Clone());
  sub_shapes.emplace_back(box_shape.Clone());
  sub_shapes.emplace_back(nested_composite_shape.Clone());
  geometry_shapes::CompositeShape composite_shape(std::move(sub_shapes));

  PrimitivesCount primitives_count;
  EXPECT_EQ(
      AddGeometryShapesToPrimitivesCount(composite_shape, primitives_count)
          .code(),
      absl::StatusCode::kOk);
  EXPECT_EQ(primitives_count.num_spheres, 2);
  EXPECT_EQ(primitives_count.num_capsules, 2);
  EXPECT_EQ(primitives_count.num_boxes, 2);

  PrimitivesCount indices;
  CompositeObject<Scalar> objects;
  objects.ResizeBuffers(primitives_count);
  EXPECT_EQ(
      AddGeometryShapesToCompositeObject(Pose3d::Identity(), composite_shape,
                                         Scalar(0.1), objects, indices)
          .code(),
      absl::StatusCode::kOk);
  EXPECT_EQ(indices.num_spheres, 2);
  EXPECT_EQ(indices.num_capsules, 2);
  EXPECT_EQ(indices.num_boxes, 2);

  ASSERT_EQ(objects.spheres.size(), 2);
  for (const auto& sphere : objects.spheres) {
    EXPECT_NEAR(sphere.radius, Scalar(0.1 + 1.0), kEpsilon);
  }
  EXPECT_THAT(objects.spheres[0].center,
              IsApprox(local_transform.translation().cast<Scalar>(), kEpsilon));
  EXPECT_THAT(
      objects.spheres[1].center,
      IsApprox(nested_local_transform.translation().cast<Scalar>(), kEpsilon));

  ASSERT_EQ(objects.capsules.size(), 2);
  for (const auto& capsule : objects.capsules) {
    EXPECT_NEAR(capsule.radius, Scalar(0.1 + 1.0), kEpsilon);
    EXPECT_NEAR(capsule.half_length, Scalar(1.0), kEpsilon);
  }
  EXPECT_THAT(objects.capsules[0].center,
              IsApprox(local_transform.translation().cast<Scalar>(), kEpsilon));
  EXPECT_THAT(
      objects.capsules[1].center,
      IsApprox(nested_local_transform.translation().cast<Scalar>(), kEpsilon));
  EXPECT_THAT(
      objects.capsules[0].direction,
      IsApprox(local_transform.rotationMatrix().col(2).template cast<Scalar>(),
               kEpsilon));
  EXPECT_THAT(objects.capsules[1].direction,
              IsApprox(nested_local_transform.rotationMatrix()
                           .col(2)
                           .template cast<Scalar>(),
                       kEpsilon));

  ASSERT_EQ(objects.boxes.size(), 2);
  for (const auto& box : objects.boxes) {
    EXPECT_THAT(box.half_lengths,
                IsApprox(Vector3(1 * 0.5 + 0.1, 2 * 0.5 + 0.1, 3 * 0.5 + 0.1),
                         kEpsilon));
  }
  EXPECT_THAT(objects.boxes[0].center,
              IsApprox(local_transform.translation().template cast<Scalar>(),
                       kEpsilon));
  EXPECT_THAT(
      objects.boxes[1].center,
      IsApprox(nested_local_transform.translation().template cast<Scalar>(),
               kEpsilon));
  EXPECT_THAT(
      objects.boxes[0].box_rotation_world,
      IsApprox(
          local_transform.rotationMatrix().transpose().template cast<Scalar>(),
          kEpsilon));
  EXPECT_THAT(objects.boxes[1].box_rotation_world,
              IsApprox(nested_local_transform.rotationMatrix()
                           .transpose()
                           .template cast<Scalar>(),
                       kEpsilon));
}

REGISTER_TYPED_TEST_SUITE_P(GeometryShapeConversionTest, SphereConversion,
                            CapsuleConversion, BoxConversion,
                            CompositeShapeConversion,
                            NestedCompositeShapePropagatesLocalTransform);

INSTANTIATE_TYPED_TEST_SUITE_P(GeometryShapeConversionTestSuite,
                               GeometryShapeConversionTest, FPTypes);
}  // namespace
}  // namespace collision_checking
