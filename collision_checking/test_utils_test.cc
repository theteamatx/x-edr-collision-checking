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

#include "collision_checking/test_utils.h"


#include "absl/log/absl_log.h"
#include "benchmark/benchmark.h"
#include "gtest/gtest.h"

namespace collision_checking {
namespace testing {
namespace {

TEST(TestUtils, MallocCounterSeesNewAndDelete) {
  if (!MallocCounterIsAvailable()) {
    ABSL_LOG(INFO) << "Skipping test.";
    return;
  }
  MallocCounterInitAndClear();
  EXPECT_EQ(MallocCounterGetAllocations(), 0);
  EXPECT_EQ(MallocCounterGetFrees(), 0);

  char *ptr = new char[10];
  benchmark::DoNotOptimize(ptr);
  EXPECT_EQ(MallocCounterGetAllocations(), 1);
  EXPECT_EQ(MallocCounterGetFrees(), 0);

  delete[] ptr;

  EXPECT_EQ(MallocCounterGetAllocations(), 1);
  EXPECT_EQ(MallocCounterGetFrees(), 1);
}

TEST(TestUtils, MallocCounterSeesMallocAndFree) {
  if (!MallocCounterIsAvailable()) {
    ABSL_LOG(INFO) << "Skipping test.";
    return;
  }
  MallocCounterInitAndClear();
  EXPECT_EQ(MallocCounterGetAllocations(), 0);
  EXPECT_EQ(MallocCounterGetFrees(), 0);

  void *ptr = malloc(10);
  benchmark::DoNotOptimize(ptr);
  EXPECT_EQ(MallocCounterGetAllocations(), 1);
  EXPECT_EQ(MallocCounterGetFrees(), 0);

  free(ptr);

  EXPECT_EQ(MallocCounterGetAllocations(), 1);
  EXPECT_EQ(MallocCounterGetFrees(), 1);
}

// Consider adding tests for calloc/realloc/etc.

}  // namespace
}  // namespace testing
}  // namespace collision_checking
