#include "collision_checking/voxel_map_object.h"
#include "googlex/proxy/eigenmath/distribution.h"
#include "googlex/proxy/eigenmath/interpolation.h"
#include "googlex/proxy/eigenmath/sampling.h"
#include "third_party/benchmark/include/benchmark/benchmark.h"

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
  ::blue::eigenmath::ProxyTestGenerator generator(
      ::blue::eigenmath::kGeneratorTestSeed);
  ::blue::eigenmath::UniformDistributionVector<Scalar, 3> center_dist;
  // Add elements inside the query box.
  for (int i = 0; i < inside_count; ++i) {
    object.AddSphere(
        {{::blue::eigenmath::InterpolateLinearInBox(
             center_dist(generator), query_box.low, query_box.high)},
         Scalar{0}},
        i);
  }
  object.UpdateQueryStructures();

  // Add elements outside the query box.
  for (int i = inside_count; i < object.size();) {
    const Sphere<Scalar> sphere{
        {::blue::eigenmath::InterpolateLinearInBox(center_dist(generator),
                                                   map_box.low, map_box.high)},
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
