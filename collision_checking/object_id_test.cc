#include "experimental/users/buschmann/collision_checking/object_id.h"

#include "experimental/users/buschmann/collision_checking/test_utils.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"

namespace collision_checking {
namespace {
using testing::StatusCodeIs;

TEST(ObjectIdSet, Contains) {
  ObjectIdSet set;
  EXPECT_TRUE(set.Empty());
  EXPECT_FALSE(set.Contains(ObjectId(5)));
  set.Insert(ObjectId(5));
  EXPECT_TRUE(set.Contains(ObjectId(5)));
  EXPECT_FALSE(set.Contains(ObjectId(3)));
  set.Insert(ObjectIdSet(0b11));
  EXPECT_TRUE(set.Contains(ObjectId(0)));
  EXPECT_TRUE(set.Contains(ObjectId(1)));
}

TEST(ObjectIdSet, Clear) {
  ObjectIdSet set{0xF};
  EXPECT_FALSE(set.Empty());
  set.Clear();
  EXPECT_TRUE(set.Empty());
}

TEST(ObjectIdSet, Overlaps) {
  ObjectIdSet set;
  EXPECT_FALSE(set.Overlaps(ObjectIdSet(0b11)));
  EXPECT_FALSE(set.Overlaps(ObjectIdSet(0b00)));

  set.Insert(ObjectIdSet(0b11));
  EXPECT_TRUE(set.Overlaps(ObjectIdSet(0b11)));
  EXPECT_TRUE(set.Overlaps(ObjectIdSet(0b10)));
  EXPECT_TRUE(set.Overlaps(ObjectIdSet(0b01)));
  EXPECT_FALSE(set.Overlaps(ObjectIdSet(0b100)));
}

TEST(ObjectIdSet, Remove) {
  ObjectIdSet set;
  EXPECT_TRUE(set.Empty());
  set.Remove(ObjectIdSet(0b11));
  EXPECT_TRUE(set.Empty());

  set.Insert(ObjectIdSet(0b11));
  set.Remove(ObjectIdSet(0b100));
  EXPECT_EQ(set, ObjectIdSet(0b11));
  set.Remove(ObjectIdSet(0b10));
  EXPECT_EQ(set, ObjectIdSet(0b01));
  set.Remove(ObjectIdSet(0b01));
  EXPECT_EQ(set, ObjectIdSet(0b00));
}

TEST(ObjectIdSet, Operators) {
  EXPECT_EQ(ObjectIdSet(0b10) | ObjectIdSet(0b01), ObjectIdSet(0b11));

  EXPECT_EQ(ObjectIdSet(0b10) & ObjectIdSet(0b01), ObjectIdSet(0b00));
  EXPECT_EQ(ObjectIdSet(0b11) & ObjectIdSet(0b01), ObjectIdSet(0b01));

  EXPECT_EQ(ObjectIdSet(0b11) ^ ObjectIdSet(0b01), ObjectIdSet(0b10));
  EXPECT_EQ(ObjectIdSet(0b10) ^ ObjectIdSet(0b01), ObjectIdSet(0b11));
}

TEST(ObjectIdSet, GetObjectIdFromSet) {
  for (ObjectId::ValueType i = 0; i < CHAR_BIT * sizeof(ObjectIdSet::SetType);
       ++i) {
    ObjectId id{i};
    ObjectIdSet set(id);
     CC_ASSERT_OK_AND_ASSIGN(ObjectId roundtrip, GetObjectIdFromSet(set));
    EXPECT_EQ(id, roundtrip);

    roundtrip = ObjectId{0};
    EXPECT_TRUE(GetObjectIdFromSet(set, roundtrip));
    EXPECT_EQ(id, roundtrip);
  }

  EXPECT_THAT(GetObjectIdFromSet(ObjectIdSet{3}),
              StatusCodeIs(absl::StatusCode::kFailedPrecondition));

  ObjectId id;
  EXPECT_FALSE(GetObjectIdFromSet(ObjectIdSet{3}, id));
}

TEST(ObjectIdSet, Count) {
  ObjectIdSet set;
  EXPECT_EQ(set.Count(), 0);
  set.Insert(ObjectId(1));
  EXPECT_EQ(set.Count(), 1);
  set.Insert(ObjectId(1));
  EXPECT_EQ(set.Count(), 1);
  set.Insert(ObjectId(5));
  EXPECT_EQ(set.Count(), 2);
}

TEST(ObjectFlags, ShouldCheckPair) {
  ObjectFlags flags_a, flags_b;
  // flag_a and flag_b's ids are in each others inclusion set.
  flags_a.id_set = ObjectIdSet(ObjectId(0));
  flags_a.inclusion_set = ObjectIdSet(0b1111);
  flags_b.id_set = ObjectIdSet(ObjectId(1));
  flags_b.inclusion_set = ObjectIdSet(0b1111);
  EXPECT_TRUE(ShouldCheckPair(flags_a, flags_b));
  // flag_a's id is in flag_b's inclusion set, but not vice versa.
  flags_a.id_set = ObjectIdSet(ObjectId(0));
  flags_a.inclusion_set = ObjectIdSet(0b100);
  flags_b.id_set = ObjectIdSet(ObjectId(1));
  flags_b.inclusion_set = ObjectIdSet(0b1111);
  EXPECT_FALSE(ShouldCheckPair(flags_a, flags_b));

  // flag_b's id is in flag_a's inclusion set, but not vice versa.
  flags_b.id_set = ObjectIdSet(ObjectId(0));
  flags_b.inclusion_set = ObjectIdSet(0b100);
  flags_a.id_set = ObjectIdSet(ObjectId(1));
  flags_a.inclusion_set = ObjectIdSet(0b1111);
  EXPECT_FALSE(ShouldCheckPair(flags_a, flags_b));

  // Neither flag's id is in the others inclusion set
  flags_b.id_set = ObjectIdSet(ObjectId(0));
  flags_b.inclusion_set = ObjectIdSet(0b100);
  flags_a.id_set = ObjectIdSet(ObjectId(1));
  flags_a.inclusion_set = ObjectIdSet(0b1000);
  EXPECT_FALSE(ShouldCheckPair(flags_a, flags_b));
}

}  // namespace
}  // namespace collision_checking
