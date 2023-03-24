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
