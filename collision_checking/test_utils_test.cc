#include "collision_checking/test_utils.h"


#include "third_party/absl/log/absl_log.h"
#include "third_party/benchmark/include/benchmark/benchmark.h"
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
