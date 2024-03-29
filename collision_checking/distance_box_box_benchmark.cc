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

#include "benchmark/benchmark.h"
#include "collision_checking/distance_box_box.h"
#include "eigenmath/rotation_utils.h"

namespace collision_checking {
namespace {
void BM_DistanceSquaredTest(benchmark::State& state) {
  using Scalar = double;
  Box<Scalar> box_a;
  Box<Scalar> box_b;
  box_a.center << -1, -2, -3;
  eigenmath::RotationFromRPY<Scalar>(0.1, 0.2, 0.3, &box_a.box_rotation_world);
  box_a.half_lengths << 2, 3, 4;
  box_b.center = box_a.center;
  box_b.half_lengths << 0.5, 1.5, 0.4;
  box_b.box_rotation_world = box_a.box_rotation_world;
  // Use state.range(0) to choose test cases:
  switch (state.range(0)) {
    case 0: {
      // Box B in box A, not parallel.
      eigenmath::RotationFromRPY<Scalar>(-0.1, -0.2, 0.35,
                                         &box_b.box_rotation_world);
    } break;
    case 1: {
      // Box B in Box A, parallel.
    } break;
    case 2: {
      // Box B outside box A, fully aligned.
      box_b.center[0] += 10;
      box_b.center[1] += 11;
      box_b.center[2] += 12;
    } break;
    case 3: {
      // Box B outside box A, not aligned.
      eigenmath::RotationFromRPY<Scalar>(-0.1, -0.2, 0.35,
                                         &box_b.box_rotation_world);
      box_b.center[0] += 10;
      box_b.center[1] += 11;
      box_b.center[2] += 12;
    } break;
    case 4: {
      // Box B half inside box A.
      eigenmath::RotationFromRPY<Scalar>(-0.1, -0.2, 0.35,
                                         &box_b.box_rotation_world);
      box_b.center += box_a.half_lengths * 0.6;
    } break;
  }

  for (auto _ : state) {
    DistanceSquared(box_a, box_b);
  }
}

BENCHMARK(BM_DistanceSquaredTest)->Arg(0)->Arg(1)->Arg(2)->Arg(3)->Arg(4);

}  // namespace
}  // namespace collision_checking

// Run the benchmark
BENCHMARK_MAIN();
