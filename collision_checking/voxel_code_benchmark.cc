#include "experimental/users/buschmann/collision_checking/voxel_code.h"

#include "third_party/benchmark/include/benchmark/benchmark.h"

namespace collision_checking {
namespace {
void BM_VoxelIndexToCode(benchmark::State& state) {
  VoxelCodeTraits::CodeType code;
  Vector3<VoxelCodeTraits::IntCoordType> coords;
  coords.setZero();

  for (auto _ : state) {
    benchmark::DoNotOptimize(coords);
    benchmark::DoNotOptimize(code = VoxelIndexToCode(coords));
    benchmark::DoNotOptimize(code);
  }
}

BENCHMARK(BM_VoxelIndexToCode);

void BM_CodeToVoxelIndex(benchmark::State& state) {
  VoxelCodeTraits::CodeType code;
  Vector3<VoxelCodeTraits::IntCoordType> coords;
  coords.setZero();

  for (auto _ : state) {
    benchmark::DoNotOptimize(code);
    benchmark::DoNotOptimize(coords = CodeToVoxelIndex(code));
    benchmark::DoNotOptimize(coords);
  }
}

BENCHMARK(BM_CodeToVoxelIndex);
}  // namespace
}  // namespace collision_checking

// Run the benchmark
BENCHMARK_MAIN();

// BM_VoxelIndexToCode       3.01 ns         3.01 ns    463178940
// BM_CodeToVoxelIndex       6.02 ns         6.02 ns    233443952
