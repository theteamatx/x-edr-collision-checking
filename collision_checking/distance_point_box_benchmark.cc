#include "collision_checking/distance_point_box.h"

#include "googlex/proxy/eigenmath/rotation_utils.h"
#include "third_party/benchmark/include/benchmark/benchmark.h"
namespace collision_checking {
namespace {
void BM_DistanceSquaredTest(benchmark::State& state) {
  using Scalar = double;
  Box<Scalar> box;
  box.center << -1, -2, -3;
  ::blue::eigenmath::RotationFromRPY<Scalar>(0.1, 0.2, 0.3,
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
