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

#include "collision_checking/voxel_map_object.h"

#include <iostream>
#include <vector>

#include "absl/algorithm/container.h"
#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace collision_checking {
namespace {
using ::testing::ElementsAreArray;

template <typename T>
class VoxelMapObjectTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(VoxelMapObjectTest);

TYPED_TEST_P(VoxelMapObjectTest, BasicOperations) {
  using Scalar = TypeParam;

  VoxelMapObject<Scalar> object;

  // Test the uninitialized state.
  EXPECT_EQ(object.size(), 0);
  EXPECT_EQ(object.capacity(), 0);
  EXPECT_EQ(object.flags().id_set, kVoxelMapIdSet);
  EXPECT_EQ(object.flags().inclusion_set, ~kVoxelMapIdSet);

  // Add some spheres.
  std::vector<Sphere<Scalar>> spheres(2);
  for (int i = 0; i < spheres.size(); ++i) {
    spheres[i].center.setConstant(i);
    spheres[i].radius = i;
  }

  object.ResizeBuffers(spheres.size());
  EXPECT_EQ(object.size(), 0);
  EXPECT_EQ(object.capacity(), spheres.size());

  for (int i = 0; i < spheres.size(); ++i) {
    object.AddSphere(spheres[i], i);
  }

  // Expect that iteration over all elements works.
  std::vector<Sphere<Scalar>> spheres_from_iterator;
  for (auto& it : object.GetVoxelRange()) {
    spheres_from_iterator.push_back(it.sphere);
  }
  ASSERT_EQ(spheres_from_iterator.size(), spheres.size());
  // Sort as iteration order might differ from insertion order.
  absl::c_stable_sort(spheres, [](const auto& left, const auto& right) {
    return left.radius < right.radius;
  });
  absl::c_stable_sort(spheres_from_iterator,
                      [](const auto& left, const auto& right) {
                        return left.radius < right.radius;
                      });

  for (int i = 0; i < object.size(); ++i) {
    EXPECT_EQ(spheres_from_iterator[i].radius, spheres[i].radius);
    EXPECT_THAT(spheres_from_iterator[i].center,
                ElementsAreArray(spheres[i].center));
  }
}

TYPED_TEST_P(VoxelMapObjectTest, InBoxVoxelRange) {
  using Scalar = TypeParam;

  // The query-range iterator should work with or without updating the
  // query structures.
  for (bool with_query_structure_update : {true, false}) {
    SCOPED_TRACE(absl::StrCat("with_query_structure_update: ",
                              with_query_structure_update));
    AlignedBox<Scalar> box;
    box.high = Vector3<Scalar>(1, 2, 3);
    box.low = Vector3<Scalar>(0, -1, 2);

    // One inside, one on the edge and three outside.
    std::vector<Sphere<Scalar>> spheres(5);
    spheres[0].center =
        box.low.array() - Vector3<Scalar>::Constant(Scalar{1.1}).array();
    spheres[1].center = (box.low + box.high) / Scalar{2};
    spheres[2].center = box.low;
    spheres[3].center =
        box.high.array() + Vector3<Scalar>::Constant(Scalar{1}).array();
    spheres[4].center =
        box.low.array() - Vector3<Scalar>::Constant(Scalar{1}).array();

    VoxelMapObject<Scalar> object;
    object.ResizeBuffers(spheres.size());
    for (int i = 0; i < spheres.size(); ++i) {
      spheres[i].radius = Scalar{0.01} * i;
      object.AddSphere(spheres[i], i);
      printf("Debug: input sphere[%d]= %f %f %f\n", i, spheres[i].center[0],
             spheres[i].center[1], spheres[i].center[2]);
    }

    if (with_query_structure_update) {
      object.UpdateQueryStructures();
    }

    // Iteration should return second and third sphere (those inside the box or
    // on the edge).
    std::vector<Sphere<Scalar>> spheres_from_iterator;
    for (auto& it : object.GetInBoxVoxelRange(box)) {
      spheres_from_iterator.push_back(it.sphere);
    }
    absl::c_stable_sort(spheres_from_iterator,
                        [](const auto& left, const auto& right) {
                          return left.radius < right.radius;
                        });
    for (int i = 0; i < spheres_from_iterator.size(); ++i) {
      std::cout << "ITERATOR_OUT: "
                << spheres_from_iterator[i].center.transpose() << std::endl;
    }
    EXPECT_EQ(spheres_from_iterator.size(), 2);
    ASSERT_LT(spheres_from_iterator.size(), spheres.size());
    for (int i = 0; i < spheres_from_iterator.size(); ++i) {
      EXPECT_THAT(spheres_from_iterator[i].center,
                  ElementsAreArray(spheres[i + 1].center));
    }
  }
}

typedef ::testing::Types<float, double> FPTypes;
REGISTER_TYPED_TEST_SUITE_P(VoxelMapObjectTest, BasicOperations,
                            InBoxVoxelRange);

INSTANTIATE_TYPED_TEST_SUITE_P(VoxelMapObjectTestSuite, VoxelMapObjectTest,
                               FPTypes);

}  // namespace
}  // namespace collision_checking
