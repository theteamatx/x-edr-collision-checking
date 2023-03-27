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

#include "collision_checking/typed_id_int.h"

#include <cstdint>

#include "absl/container/flat_hash_set.h"
#include "gtest/gtest.h"

namespace collision_checking {
namespace {
typedef ::testing::Types<uint8_t, uint16_t, uint32_t, uint64_t, int, char>
    ValueTypes;

template <typename T>
class TypedIdIntTest : public ::testing::Test {
 public:
};

TYPED_TEST_SUITE_P(TypedIdIntTest);

TYPED_TEST_P(TypedIdIntTest, OperatorsWork) {
  const TypeParam one(1);
  const TypeParam two(2);

  EXPECT_EQ(one + two, 3);
  EXPECT_EQ(two - one, 1);

  EXPECT_TRUE(one == 1);
  EXPECT_TRUE(one == one);
  EXPECT_TRUE(one != two);
  EXPECT_TRUE(one != 2);
}

TYPED_TEST_P(TypedIdIntTest, HashingWorks) {
  absl::flat_hash_set<TypeParam> set;
  set.insert(TypeParam(0));
  set.insert(TypeParam(1));

  EXPECT_TRUE(set.contains(TypeParam(0)));
  EXPECT_TRUE(set.contains(TypeParam(1)));
}

REGISTER_TYPED_TEST_SUITE_P(TypedIdIntTest, OperatorsWork, HashingWorks);

INSTANTIATE_TYPED_TEST_SUITE_P(TypedIdIntTestSuite, TypedIdIntTest, ValueTypes);
}  // namespace
}  // namespace collision_checking
