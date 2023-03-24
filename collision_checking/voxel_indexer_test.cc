#include "collision_checking/voxel_indexer.h"

#include <limits>

#include "collision_checking/eigenmath.h"
#include "collision_checking/geometry.h"
#include "collision_checking/test_utils.h"
#include "third_party/absl/flags/flag.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

// Set to true to run exhaustive tests, which are not run by default on TAP.
ABSL_FLAG(bool, run_exhaustive_tests, false,
          "If true, exhaustive tests are run that are disabled by default.");

namespace collision_checking {
namespace {
using IntType = std::uint8_t;
using ::testing::ElementsAre;
using ::blue::eigenmath::testing::IsApprox;

typedef ::testing::Types<float, double> FPTypes;

template <typename T>
class VoxelMapIndexerDeathTest : public ::testing::Test {};
TYPED_TEST_SUITE_P(VoxelMapIndexerDeathTest);

TYPED_TEST_P(VoxelMapIndexerDeathTest, ConstructorPanics) {
  using FloatType = TypeParam;
  using VoxelIndexer = VoxelIndexer<FloatType, IntType>;

  // Negative cell width.
  EXPECT_DEATH(
      (VoxelIndexer(AlignedBox<FloatType>{.low = Vector3<FloatType>(0, 0, 0),
                                          .high = Vector3<FloatType>(1, 1, 1)},
                    FloatType{-1.0})),
      "cell_width must be positive");

  // Too many cells.
  EXPECT_DEATH(
      (VoxelIndexer(
          AlignedBox<FloatType>{.low = Vector3<FloatType>(0, 0, 0),
                                .high = Vector3<FloatType>(256, 10, 20)},
          FloatType{1.0})),
      "Too many cells");
}

REGISTER_TYPED_TEST_SUITE_P(VoxelMapIndexerDeathTest, ConstructorPanics);

INSTANTIATE_TYPED_TEST_SUITE_P(VoxelMapIndexerDeathTestSuite,
                               VoxelMapIndexerDeathTest, FPTypes);

template <typename T>
class VoxelMapIndexer : public ::testing::Test {};
TYPED_TEST_SUITE_P(VoxelMapIndexer);

TYPED_TEST_P(VoxelMapIndexer, ToPointToCellRoundTrip) {
  using FloatType = TypeParam;
  using Point = Vector3<FloatType>;
  using Index = Vector3<IntType>;

  constexpr uint32_t kMaxIndex = std::numeric_limits<IntType>::max();
  constexpr FloatType kCellWidth{1};
  const AlignedBox<FloatType> box{.low = Point(0.1, 0.1, 0.1),
                                  .high = Point(255.1, 255.1, 255.1)};
  VoxelIndexer<FloatType, IntType> indexer(box, kCellWidth);

  const int increment =
      absl::GetFlag<bool>(FLAGS_run_exhaustive_tests) ? 1 : 10;
  for (uint32_t i = 0; i <= kMaxIndex; i += increment) {
    for (uint32_t j = 0; j <= kMaxIndex; j += increment) {
      for (uint32_t k = 0; k <= kMaxIndex; k += increment) {
        const Index cell(i, j, k);
        const Point point = indexer.ToPoint(cell);
        const Index cell_roundtrip = indexer.ToCell(point);
        ASSERT_THAT(cell_roundtrip, ::testing::ElementsAreArray(cell));
      }
    }
  }
}

TYPED_TEST_P(VoxelMapIndexer, ToPoint) {
  using FloatType = TypeParam;
  using Point = Vector3<FloatType>;
  using Index = Vector3<IntType>;

  constexpr FloatType kCellWidth{0.1};
  const AlignedBox<FloatType> box{.low = Point(0.1, 0.2, 0.3),
                                  .high = Point(1, 1, 1)};

  VoxelIndexer<FloatType, IntType> indexer(box, kCellWidth);
  EXPECT_THAT(indexer.ToPoint(Index(0, 0, 0)), IsApprox(box.low));
  EXPECT_THAT(indexer.ToPoint(Index(2, 1, 5)), IsApprox(Point(0.3, 0.3, 0.8)));
  EXPECT_THAT(indexer.ToPoint(Index(9, 8, 7)), IsApprox(Point(1, 1, 1)));

  // Out-of-range indices are *not* clamped.
  EXPECT_THAT(indexer.ToPoint(Index(10, 9, 8)), IsApprox(Point(1.1, 1.1, 1.1)));
}

TYPED_TEST_P(VoxelMapIndexer, ToCell) {
  using FloatType = TypeParam;
  using Point = Vector3<FloatType>;

  constexpr FloatType kCellWidth{0.1};
  const AlignedBox<FloatType> box{.low = Point(0.1, 0.2, 0.3),
                                  .high = Point(1, 1, 1)};

  VoxelIndexer<FloatType, IntType> indexer(box, kCellWidth);
  EXPECT_THAT(indexer.ToCell(box.low), ElementsAre(0, 0, 0));
  EXPECT_THAT(indexer.ToCell(Point(0.31, 0.31, 0.81)), ElementsAre(2, 1, 5));
  EXPECT_THAT(indexer.ToCell(box.high), ElementsAre(9, 8, 7));

  // Out of range values are clamped.
  EXPECT_THAT(indexer.ToCell(box.low.array() - 0.1), ElementsAre(0, 0, 0));
}

TYPED_TEST_P(VoxelMapIndexer, MinCellWidth) {
  using FloatType = TypeParam;
  using Point = Vector3<FloatType>;

  EXPECT_EQ(
      (VoxelIndexer<FloatType, IntType>::MinCellWidth(Point(255, 255, 255))),
      1);
  EXPECT_EQ(
      (VoxelIndexer<FloatType, IntType>::MinCellWidth(Point(128, 255, 128))),
      1);
}

REGISTER_TYPED_TEST_SUITE_P(VoxelMapIndexer, ToPointToCellRoundTrip, ToPoint,
                            ToCell, MinCellWidth);

INSTANTIATE_TYPED_TEST_SUITE_P(VoxelMapIndexerTestSuite, VoxelMapIndexer,
                               FPTypes);

}  // namespace
}  // namespace collision_checking
