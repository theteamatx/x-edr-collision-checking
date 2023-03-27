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

#include "collision_checking/voxel_code.h"

#include <vector>

#include "collision_checking/eigenmath.h"
#include "absl/flags/flag.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

// Set to true to run exhaustive tests, which are not run by default on TAP.
ABSL_FLAG(bool, run_exhaustive_tests, false,
          "If true, exhaustive tests are run that are disabled by default.");

namespace collision_checking {
namespace {
using ::testing::ElementsAre;
using CodeType = VoxelCodeTraits::CodeType;
using IntCoordType = VoxelCodeTraits::IntCoordType;
using Coords = Vector3<IntCoordType>;

// Exhaustively test that round encode/decode roundtrip works.
TEST(VoxelCodeTest, EncodeDecodeRoundTrip) {
  const int increment =
      absl::GetFlag<bool>(FLAGS_run_exhaustive_tests) ? 1 : 10;
  for (CodeType i = 0; i <= VoxelCodeTraits::kMaxCoord; i += increment) {
    for (CodeType j = 0; j <= VoxelCodeTraits::kMaxCoord; j += increment) {
      for (CodeType k = 0; k <= VoxelCodeTraits::kMaxCoord; k += increment) {
        const Coords coords(i, j, k);
        const CodeType code = VoxelIndexToCode(coords);
        const Coords coords_roundtrip = CodeToVoxelIndex(code);
        EXPECT_THAT(coords_roundtrip, ::testing::ElementsAreArray(coords));
      }
    }
  }
}

TEST(VoxelCodeTest, VoxelIndexToCode) {
  // Test a few encodings.
  EXPECT_THAT(VoxelIndexToCode(Coords(0, 0, 0)), 0);
  EXPECT_THAT(VoxelIndexToCode(Coords(0b1, 0b1, 0b1)), 0b111);
  EXPECT_THAT(VoxelIndexToCode(Coords(0b1110, 0b0101, 0b0111)), 0b001111101110);
  EXPECT_THAT(VoxelIndexToCode(Coords(0xFF, 0xFF, 0xFF)), 0xFFFFFF);
}

TEST(VoxelCodeTest, CodeToVoxelIndex) {
  // Test a few decodings.
  EXPECT_THAT(CodeToVoxelIndex(0), ElementsAre(0, 0, 0));
  EXPECT_THAT(CodeToVoxelIndex(0b111), ElementsAre(0b1, 0b1, 0b1));
  EXPECT_THAT(CodeToVoxelIndex(0b001111101110),
              ElementsAre(0b1110, 0b0101, 0b0111));
  EXPECT_THAT(CodeToVoxelIndex(0xFFFFFF), ElementsAre(0xFF, 0xFF, 0xFF));
}

TEST(VoxelCodeTest, CodeBracketIncludesAllIndicesInBoundingBox) {
  // Verify that the code for the lower bounding box corner and the code for
  // the upper bounding box corner bracket the codes for all cells in the
  // bounding box.
  struct IntBox {
    Coords low;
    Coords high;
  };

  // A list of more or less arbitrarily chosen bounding boxes.
  std::vector<IntBox> boxes = {
      {.low = Coords(10, 20, 5), .high = Coords(20, 50, 51)},
      {.low = Coords(0, 0, 0), .high = Coords(20, 50, 51)},
      {.low = Coords(0, 0, 0), .high = Coords(255, 255, 255)},
      {.low = Coords(100, 100, 100), .high = Coords(110, 101, 118)},
      {.low = Coords(250, 253, 200), .high = Coords(255, 255, 255)},
  };

  for (const auto& box : boxes) {
    const CodeType low_code = VoxelIndexToCode(box.low);
    const CodeType high_code = VoxelIndexToCode(box.high);
    for (int i = box.low[0]; i <= box.high[0]; ++i) {
      for (int j = box.low[1]; j <= box.high[1]; ++j) {
        for (int k = box.low[2]; k <= box.high[2]; ++k) {
          const CodeType code = VoxelIndexToCode(Coords(i, j, k));
          ASSERT_GE(code, low_code);
          ASSERT_LE(code, high_code);
        }
      }
    }
  }
}

}  // namespace
}  // namespace collision_checking
