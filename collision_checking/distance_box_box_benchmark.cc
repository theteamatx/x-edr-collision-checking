#include "experimental/users/buschmann/collision_checking/distance_box_box.h"

#include "googlex/proxy/eigenmath/rotation_utils.h"
#include "third_party/benchmark/include/benchmark/benchmark.h"

namespace collision_checking {
namespace {
void BM_DistanceSquaredTest(benchmark::State& state) {
  using Scalar = double;
  Box<Scalar> box_a;
  Box<Scalar> box_b;
  box_a.center << -1, -2, -3;
  ::blue::eigenmath::RotationFromRPY<Scalar>(0.1, 0.2, 0.3,
                                             &box_a.box_rotation_world);
  box_a.half_lengths << 2, 3, 4;
  box_b.center = box_a.center;
  box_b.half_lengths << 0.5, 1.5, 0.4;
  box_b.box_rotation_world = box_a.box_rotation_world;
  // Use state.range(0) to choose test cases:
  switch (state.range(0)) {
    case 0: {
      // Box B in box A, not parallel.
      ::blue::eigenmath::RotationFromRPY<Scalar>(-0.1, -0.2, 0.35,
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
      ::blue::eigenmath::RotationFromRPY<Scalar>(-0.1, -0.2, 0.35,
                                                  &box_b.box_rotation_world);
      box_b.center[0] += 10;
      box_b.center[1] += 11;
      box_b.center[2] += 12;
    } break;
    case 4: {
      // Box B half inside box A.
      ::blue::eigenmath::RotationFromRPY<Scalar>(-0.1, -0.2, 0.35,
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
