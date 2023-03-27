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

#include "collision_checking/distance_point_box.h"

#include "eigenmath/rotation_utils.h"
#include "benchmark/benchmark.h"

namespace collision_checking {
namespace {
void BM_DistanceSquaredTest(benchmark::State& state) {
  using Scalar = double;
  Box<Scalar> box;
  box.center << -1, -2, -3;
  eigenmath::RotationFromRPY<Scalar>(0.1, 0.2, 0.3,
                                             &box.box_rotation_world);
  box.half_lengths << 2, 3, 4;
  // Point location relative to box has no influence on run times.
  Point<Scalar> point = {box.center + 1.5 * box.half_lengths};

  for (auto _ : state) {
    benchmark::DoNotOptimize(DistanceSquared(point, box));
  }
}

BENCHMARK(BM_DistanceSquaredTest);

}  // namespace
}  // namespace collision_checking

// Run the benchmark
BENCHMARK_MAIN();
