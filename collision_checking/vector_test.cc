#include "collision_checking/vector.h"

#include <memory>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace collision_checking {
namespace {

template <typename T>
class VectorCpuTest : public testing::Test {};
TYPED_TEST_SUITE_P(VectorCpuTest);

TYPED_TEST_P(VectorCpuTest, BasicFunctionalityWorks) {
  Vector<int, DefaultAllocatorTraits> vec;

  EXPECT_EQ(vec.size(), 0);
  EXPECT_TRUE(vec.empty());

  vec.resize(0);
  EXPECT_EQ(vec.size(), 0);

  vec.resize(3);
  EXPECT_EQ(vec.size(), 3);
  vec.resize(3);
  EXPECT_EQ(vec.size(), 3);
  EXPECT_FALSE(vec.empty());

  vec[0] = 1;
  vec[1] = 3;
  vec[2] = 2;

  EXPECT_EQ(vec[0], 1);
  EXPECT_EQ(vec[1], 3);
  EXPECT_EQ(vec[2], 2);

  int sum = 0;
  for (const auto& v : vec) {
    sum += v;
  }
  EXPECT_EQ(sum, 6);

  EXPECT_EQ(vec.data(), &vec[0]);

  Vector<int, DefaultAllocatorTraits> copy = vec;

  EXPECT_EQ(vec[0], copy[0]);
  EXPECT_EQ(vec[1], copy[1]);
  EXPECT_EQ(vec[2], copy[2]);
}

TYPED_TEST_P(VectorCpuTest, ShrinkingMaintainsData) {
  Vector<int, DefaultAllocatorTraits> vec(10);

  for (int i = 0; i < vec.size(); ++i) {
    vec[i] = i;
  }
  vec.resize(5);
  ASSERT_EQ(vec.size(), 5);
  for (int i = 0; i < vec.size(); ++i) {
    EXPECT_EQ(vec[i], i);
  }
}

TYPED_TEST_P(VectorCpuTest, GrowingMaintainsData) {
  Vector<int, DefaultAllocatorTraits> vec(10);

  for (int i = 0; i < vec.size(); ++i) {
    vec[i] = i;
  }
  vec.resize(15);
  ASSERT_EQ(vec.size(), 15);
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(vec[i], i);
  }
}

REGISTER_TYPED_TEST_SUITE_P(VectorCpuTest, BasicFunctionalityWorks,
                            ShrinkingMaintainsData, GrowingMaintainsData);

using MyTypes = ::testing::Types<char, int, float>;
INSTANTIATE_TYPED_TEST_SUITE_P(My, VectorCpuTest, MyTypes);

}  // namespace
}  // namespace collision_checking
