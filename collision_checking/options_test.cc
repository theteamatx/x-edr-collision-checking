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

#include "collision_checking/options.h"

#include "gtest/gtest.h"

namespace collision_checking {
namespace {

TEST(CollisionChecking, QueryOptions) {
  QueryOptions defaults = QueryOptions();
  EXPECT_EQ(defaults.GetType(), QueryOptions::Type::kIsCollisionFree);
  EXPECT_TRUE(defaults.GetExclusionSet().Empty());

  QueryOptions options =
      QueryOptions()
          .SetType(QueryOptions::Type::kComputeObjectDistances)
          .SetExclusionSet(ObjectIdSet(0b1001001001));
  EXPECT_EQ(options.GetType(), QueryOptions::Type::kComputeObjectDistances);
  EXPECT_TRUE(options.GetExclusionSet() == ObjectIdSet(0b1001001001));
}

TEST(CollisionChecking, QueryOptionsDebugString) {
  QueryOptions options = QueryOptions();
  EXPECT_EQ(options.DebugString(),
            "type: kIsCollisionFree; exclusion_set: "
            "0000000000000000000000000000000000000000000000000000000000000000; "
            "batch_early_exit_ok: 0");
  options.SetType(QueryOptions::Type::kComputeObjectDistances)
      .SetExclusionSet(ObjectIdSet(0b1001001001))
      .SetBatchEarlyExitOk(true);
  EXPECT_EQ(options.DebugString(),
            "type: kComputeObjectDistances; exclusion_set: "
            "0000000000000000000000000000000000000000000000000000001001001001; "
            "batch_early_exit_ok: 1");
}

}  // namespace
}  // namespace collision_checking
