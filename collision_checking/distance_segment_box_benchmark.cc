#include "experimental/users/buschmann/collision_checking/distance_segment_box.h"

#include "googlex/proxy/eigenmath/rotation_utils.h"
#include "third_party/benchmark/include/benchmark/benchmark.h"

namespace collision_checking {
namespace {
template <typename Scalar>
void BM_DistanceSquaredTest(benchmark::State& state) {
  using Vector3 = Vector3<Scalar>;
  Box<Scalar> box;
  box.center << -1, -2, -3;
  ::blue::eigenmath::RotationFromRPY<Scalar>(0.1, 0.2, 0.3,
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
