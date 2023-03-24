#include "collision_checking/id_assigner.h"

#include "gtest/gtest.h"

namespace collision_checking {
namespace {
typedef ::testing::Types<ObjectIdAssigner, AssemblyIdAssigner> IdAssignerTypes;

template <typename T>
class ObjectIdAssignerDeathTest : public ::testing::Test {
 public:
};

TYPED_TEST_SUITE_P(ObjectIdAssignerDeathTest);

TYPED_TEST_P(ObjectIdAssignerDeathTest, DoubleRelease) {
  using Id = typename TypeParam::IdType;
  TypeParam id_assigner;
  EXPECT_DEATH(id_assigner.ReleaseId(Id(1)), "hasn't been assigned");
}

REGISTER_TYPED_TEST_SUITE_P(ObjectIdAssignerDeathTest, DoubleRelease);

INSTANTIATE_TYPED_TEST_SUITE_P(ObjectIdAssignerDeathTestSuite,
                               ObjectIdAssignerDeathTest, IdAssignerTypes);

template <typename T>
class ObjectIdAssignerTest : public ::testing::Test {
 public:
};

TYPED_TEST_SUITE_P(ObjectIdAssignerTest);

TYPED_TEST_P(ObjectIdAssignerTest, AllAssignments) {
  constexpr auto kNumberOfIds = TypeParam::kNumberOfIds;
  TypeParam id_assigner;
  using Id = typename TypeParam::IdType;

  EXPECT_EQ(id_assigner.NumFreeIds(),
            static_cast<size_t>(TypeParam::kMaxValidId) + 1);

  std::vector<Id> ids;
  absl::flat_hash_set<Id> id_set;

  for (int i = 0; i < kNumberOfIds; i++) {
    Id id = id_assigner.GetFreeId();
    EXPECT_NE(id, TypeParam::kInvalidId);
    if constexpr (std::is_same_v<typename TypeParam::IdType, ObjectId>) {
      for (int v = 0; v < kVoxelMapIds.size(); ++v) {
        EXPECT_NE(id, kVoxelMapIds[v]);
      }
    }
    EXPECT_EQ(id_assigner.NumFreeIds(), kNumberOfIds - i - 1);
    EXPECT_FALSE(id_set.contains(id));
    ids.push_back(id);
    id_set.insert(id);
  }
  EXPECT_EQ(id_assigner.GetFreeId(), TypeParam::kInvalidId);
  EXPECT_EQ(id_assigner.NumFreeIds(), 0);

  for (const auto& id : ids) {
    id_assigner.ReleaseId(id);
  }
  EXPECT_EQ(id_assigner.NumFreeIds(), kNumberOfIds);
}

TYPED_TEST_P(ObjectIdAssignerTest, UseAllFreeThenGetValidId) {
  TypeParam id_assigner;
  using Id = typename TypeParam::IdType;

  std::vector<Id> ids(id_assigner.NumFreeIds());
  while (id_assigner.NumFreeIds() > 0) {
    Id id = id_assigner.GetFreeId();
    EXPECT_NE(id, TypeParam::kInvalidId);
    ids.push_back(id);
  }
  EXPECT_EQ(id_assigner.GetFreeId(), TypeParam::kInvalidId);
  EXPECT_EQ(id_assigner.NumFreeIds(), 0);

  Id released_id = ids.back();
  ids.pop_back();
  id_assigner.ReleaseId(released_id);
  Id id = id_assigner.GetFreeId();
  EXPECT_EQ(id, released_id);
}

REGISTER_TYPED_TEST_SUITE_P(ObjectIdAssignerTest, AllAssignments,
                            UseAllFreeThenGetValidId);

INSTANTIATE_TYPED_TEST_SUITE_P(ObjectIdAssignerTestSuite, ObjectIdAssignerTest,
                               IdAssignerTypes);

}  // namespace
}  // namespace collision_checking
