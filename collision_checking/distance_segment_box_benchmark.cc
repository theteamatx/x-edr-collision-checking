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

#include "collision_checking/distance_segment_box.h"

#include "eigenmath/rotation_utils.h"
#include "benchmark/benchmark.h"

namespace collision_checking {
namespace {
template <typename Scalar>
void BM_DistanceSquaredTest(benchmark::State& state) {
  using Vector3 = Vector3<Scalar>;
  Box<Scalar> box;
  box.center << -1, -2, -3;
  eigenmath::RotationFromRPY<Scalar>(0.1, 0.2, 0.3,
                                             &box.box_rotation_world);
  box.half_lengths << 2, 3, 4;
  Segment<Scalar> segment;

  segment.center = box.center;
  segment.direction = Vector3(1, 1, 1).normalized();
  segment.half_length = 0.4;

  // Use state.range(0) to choose test cases:
  switch (state.range(0)) {
    case 0: {
      // Segment penetrates box, not parallel to edge.
      segment.center += 0.5 * box.half_lengths;
    } break;
    case 1: {
      // Segment penetrates box, not parallel to box.
      segment.center += 0.5 * box.half_lengths;
      segment.direction = Vector3::UnitY();
    } break;
    case 2: {
      // Segment outside box, not parallel to box.
      segment.center += 10 * box.half_lengths;
    } break;
    case 3: {
      // Segment outside box, parallel to box.
      segment.center += 10 * box.half_lengths;
      segment.direction = Vector3::UnitY();
    } break;
  }

  for (auto _ : state) {
    benchmark::DoNotOptimize(DistanceSquared(segment, box));
  }
}

BENCHMARK(BM_DistanceSquaredTest<double>)->Arg(0)->Arg(1)->Arg(2)->Arg(3);
BENCHMARK(BM_DistanceSquaredTest<float>)->Arg(0)->Arg(1)->Arg(2)->Arg(3);

}  // namespace
}  // namespace collision_checking

// Run the benchmark
BENCHMARK_MAIN();
