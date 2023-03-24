#include "experimental/users/buschmann/collision_checking/logging.h"

#include "third_party/googletest/googletest/include/gtest/gtest.h"

namespace collision_checking {
namespace {

// TEST(CollisionCheckEqDeathTest, WorksAsExpected){
//     // EXPECT_DEATH((CC_CHECK_EQ(1, 2, "Message")), "barf");
//     CC_CHECK_EQ(1, 2, "Message");
// }

TEST(CollisionCheckLtDeathTest, WorksAsExpected) {
  //  EXPECT_DEATH((CC_CHECK_LT(2, 1, "Message")), "barf");
  // CC_CHECK_LT(2, 1, "Message");
  //  CC_CHECK_LT(2, 1);

  // Panic({}, "message %d", 1);
  // Panic({}, "message ");
  // Panic({});
}

}  // namespace
}  // namespace collision_checking