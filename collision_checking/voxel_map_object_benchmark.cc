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
#include "collision_checking/voxel_map_object.h"
#include "eigenmath/distribution.h"
#include "eigenmath/interpolation.h"
#include "eigenmath/sampling.h"

namespace collision_checking {
namespace {
// A dummy function for benchmarking to run a computation over spheres in a
// query range.
template <typename Scalar>
Scalar AccumulateRadiiInBox(const VoxelMapObject<Scalar>& object,
                            const AlignedBox<Scalar>& box) {
  Scalar r{0};
  for (const auto& it : object.GetInBoxVoxelRange(box)) {
    r += it.sphere.radius;
  }
  return r;
}

void BM_InBoxVoxelRange(benchmark::State& state) {
  using Scalar = double;
  AlignedBox<Scalar> query_box;
  query_box.high = Vector3<Scalar>(1, 2, 3);
  query_box.low = Vector3<Scalar>(0, -1, 2);

  AlignedBox<Scalar> map_box;
  map_box.high = Vector3<Scalar>(5, 5, 5);
  map_box.low = Vector3<Scalar>(-5, -5, -5);

  const int outside_count = state.range(0);
  const int inside_count = state.range(1);

  // Generate the spheres.
  VoxelMapObject<Scalar> object;
  object.ResizeBuffers(outside_count + inside_count);
  // Seed the generator so runs are deterministic.
  eigenmath::TestGenerator generator(eigenmath::kGeneratorTestSeed);
  eigenmath::UniformDistributionVector<Scalar, 3> center_dist;
  // Add elements inside the query box.
  for (int i = 0; i < inside_count; ++i) {
    object.AddSphere(
        {{eigenmath::InterpolateLinearInBox(center_dist(generator),
                                            query_box.low, query_box.high)},
         Scalar{0}},
        i);
  }
  object.UpdateQueryStructures();

  // Add elements outside the query box.
  for (int i = inside_count; i < object.size();) {
    const Sphere<Scalar> sphere{
        {eigenmath::InterpolateLinearInBox(center_dist(generator), map_box.low,
                                           map_box.high)},
        Scalar{0}};

    // If the quasi-random sphere is outside the query box, keep it.
    if (!DoOverlap(query_box, sphere)) {
      object.AddSphere(sphere, i);
      ++i;
    }
  }

  for (auto _ : state) {
    AccumulateRadiiInBox(object, query_box);
  }
}

// Benchmark family:
//  * first argument: spheres inside the query box.
//  * second argument: spheres outside the query box.
BENCHMARK(BM_InBoxVoxelRange)
    ->ArgPair(0, 0)
    ->ArgPair(0, 1000)
    ->ArgPair(0, 10000)
    ->ArgPair(0, 100000)
    ->ArgPair(1000, 0)
    ->ArgPair(1000, 1000)
    ->ArgPair(1000, 10000)
    ->ArgPair(1000, 100000)
    ->ArgPair(10000, 0)
    ->ArgPair(10000, 1000)
    ->ArgPair(10000, 10000)
    ->ArgPair(10000, 100000);

}  // namespace
}  // namespace collision_checking

// Run the benchmark
BENCHMARK_MAIN();
