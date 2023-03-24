#include "experimental/users/buschmann/collision_checking/typed_id_int.h"

#include <cstdint>

#include "third_party/absl/container/flat_hash_set.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"

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
