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

#include "collision_checking/test_utils.h"

#include <dlfcn.h>  // for dlsym()

// #include "absl/log/absl_log.h"
#include "collision_checking/logging.h"

namespace {
std::atomic<size_t> g_allocs = 0;
std::atomic<size_t> g_frees = 0;
}  // namespace

namespace collision_checking {
namespace testing {

void MallocCounterInitAndClear() {
  g_allocs = 0;
  g_frees = 0;
}

int MallocCounterGetAllocations() { return g_allocs; }

int MallocCounterGetFrees() { return g_frees; }

bool MallocCounterIsAvailable() { return true; }

}  // namespace testing
}  // namespace collision_checking

extern "C" {

// Wrappers for malloc and friends.
void* malloc(size_t size) {
  typedef void*(MallocFunction)(size_t);
  static MallocFunction* real_malloc =
      (MallocFunction*)::dlsym(RTLD_NEXT, "malloc");
  CC_CHECK_NE(real_malloc, nullptr, "Could not find symbol for malloc.");
  ++g_allocs;
  return real_malloc(size);
}

namespace {
// This memory is for when dlsym calls calloc when setting up the interposer
// for calloc. It appears to use 32 bytes of memory, which is increased here
// in case that changes.
std::array<char, 64> calloc_dlsym_init_mem = {};
void* dlsym_bootstrap_calloc(size_t nmemb, size_t size) {
  CC_CHECK_LT(nmemb * size, calloc_dlsym_init_mem.size(),
              "Increase calloc_dlsym_init_mem size.");
  return static_cast<void*>(calloc_dlsym_init_mem.data());
}
}  // namespace

void* calloc(size_t nmemb, size_t size) {
  typedef void*(CallocFunction)(size_t, size_t);
  static CallocFunction* real_calloc = nullptr;
  if (real_calloc == nullptr) {
    // dlsym calls calloc, so on the first call, make calloc point to a
    // something returning stack memory while we're in the first dlsym() call.
    real_calloc = dlsym_bootstrap_calloc;
    real_calloc = (CallocFunction*)::dlsym(RTLD_NEXT, "calloc");
    // real_calloc now points to the libc implementation.
  }

  CC_CHECK_NE(real_calloc, nullptr, "Could not find symbol for calloc.");
  ++g_allocs;
  return real_calloc(nmemb, size);
}

void free(void* ptr) {
  // The standard says that free(null) does nothing, so calling it even from a
  // hard real-time context is OK and we can just return here.
  if (ptr == nullptr) {
    return;
  }

  // If this is the stack memory used for dlsym when setting up the interposeser
  // for calloc, don't pass it to free.
  if (ptr == calloc_dlsym_init_mem.data()) {
    return;
  }

  typedef void*(FreeFunction)(void*);
  static FreeFunction* real_free = (FreeFunction*)::dlsym(RTLD_NEXT, "free");
  CC_CHECK_NE(real_free, nullptr, "Could not find symbol for free.");
  ++g_frees;
  real_free(ptr);
}

void* realloc(void* ptr, size_t size) {
  typedef void*(ReallocFunction)(void*, size_t);
  static ReallocFunction* real_realloc =
      (ReallocFunction*)::dlsym(RTLD_NEXT, "realloc");
  CC_CHECK_NE(real_realloc, nullptr, "Could not find symbol for realloc.");
  ++g_allocs;
  return real_realloc(ptr, size);
}

void* memalign(size_t boundary, size_t size) {
  typedef void*(MemalignFunction)(size_t, size_t);
  static MemalignFunction* real_memalign =
      (MemalignFunction*)::dlsym(RTLD_NEXT, "memalign");
  CC_CHECK_NE(real_memalign, nullptr, "Could not find symbol for memalign.");
  ++g_allocs;
  return real_memalign(boundary, size);
}

void* valloc(size_t size) {
  typedef void*(VallocFunction)(size_t);
  static VallocFunction* real_valloc =
      (VallocFunction*)::dlsym(RTLD_NEXT, "valloc");
  CC_CHECK_NE(real_valloc, nullptr, "Could not find symbol for valloc.");
  ++g_allocs;
  return real_valloc(size);
}

void* aligned_alloc(size_t alignment, size_t size) {
  typedef void*(AlignedAllocFunction)(size_t, size_t);
  static AlignedAllocFunction* real_aligned_alloc =
      (AlignedAllocFunction*)::dlsym(RTLD_NEXT, "aligned_alloc");
  CC_CHECK_NE(real_aligned_alloc, nullptr,
              "Could not find symbol for aligned_alloc.");
  ++g_allocs;
  return real_aligned_alloc(alignment, size);
}

void* pvalloc(size_t size) {
  typedef void*(PvallocFunction)(size_t);
  static PvallocFunction* real_pvalloc =
      (PvallocFunction*)::dlsym(RTLD_NEXT, "pvalloc");
  CC_CHECK_NE(real_pvalloc, nullptr, "Could not find symbol for pvalloc.");
  ++g_allocs;
  return real_pvalloc(size);
}

int posix_memalign(void** memptr, size_t alignment, size_t size) {
  typedef int(PosixMemalignFunction)(void**, size_t, size_t);
  static PosixMemalignFunction* real_posix_memalign =
      (PosixMemalignFunction*)::dlsym(RTLD_NEXT, "posix_memalign");
  CC_CHECK_NE(real_posix_memalign, nullptr,
              "Could not find symbol for posix_memalign.");
  ++g_allocs;
  return real_posix_memalign(memptr, alignment, size);
}
}  // extern "C"

namespace collision_checking {
namespace testing {
SolveQuadraticResult SolveBoxQPBruteForce(
    const eigenmath::MatrixXd& cost_matrix,
    const eigenmath::VectorXd& cost_vector,
    const eigenmath::VectorXd& lower_bound,
    const eigenmath::VectorXd& upper_bound) {
  ABSL_CHECK_EQ(cost_matrix.rows(), cost_matrix.cols());
  ABSL_CHECK_EQ(cost_matrix.rows(), cost_vector.size());
  ABSL_CHECK_EQ(cost_matrix.rows(), lower_bound.size());
  ABSL_CHECK_EQ(cost_matrix.rows(), upper_bound.size());

  // Acceptable constraint violation. Chosen rather loosly.
  constexpr double kConstraintEpsilon = 1e-8;
  const auto satisfies_constraints = [&](const eigenmath::VectorXd& x) {
    return (x.array() >= lower_bound.array() - kConstraintEpsilon).all() &&
           (x.array() <= upper_bound.array() + kConstraintEpsilon).all();
  };
  const auto cost = [&](const auto x) -> double {
    return (0.5 * cost_matrix * x + cost_vector).dot(x);
  };
  const int size = lower_bound.size();
  // Always use double precision in the solver.
  // augmented_matrix = [cost_matrix const_matrix^t;const_matrix,0]
  // augmented_matrix = [cost_vector, const_vector]
  eigenmath::MatrixXd const_matrix(size, size);
  eigenmath::VectorXd const_vector(size);
  eigenmath::MatrixXd augmented_matrix(2 * size, 2 * size);
  eigenmath::VectorXd augmented_vector(2 * size);
  eigenmath::VectorXd augmented_solution(2 * size);
  augmented_matrix.setZero();
  augmented_matrix.block(0, 0, size, size) = cost_matrix;
  augmented_vector.setZero();
  augmented_vector.segment(0, size) = cost_vector;

  enum ConstraintState {
    kInactive = 0,
    kLowerBound = 1,
    kUpperBound = 2,
    kNumStates = 3
  };
  std::vector<int> constraint_state(size, kInactive);
  // The total number of constraint states.
  const int kNumConstraintStates = std::pow<double>(kNumStates, size);
  // Given an index that runs from zero to kNumConstraintStates, sets vec such
  // that the vec[i] is the ConstraintState of the ith optimization variable.
  const auto set_constraint_state = [&](const int loop, std::vector<int>& vec) {
    int ii = loop;
    static const int initial_pow = kNumConstraintStates / kNumStates;
    int pow = initial_pow;
    for (int k = 0; k < vec.size(); ++k) {
      int e = ii / pow;
      vec[k] = e;
      ii -= e * pow;
      pow /= kNumStates;
    }
  };

  SolveQuadraticResult result{
      .minimum = std::numeric_limits<double>::infinity(),
      .solution = eigenmath::VectorXd::Constant(
          size, std::numeric_limits<double>::quiet_NaN())};
  // Loop over all constraint permutations and pick the minimum among the valid
  // solutions.
  for (int loop = 0; loop < kNumConstraintStates; ++loop) {
    // Update constraints matrix.
    // Inactive constraints simply get zero rows in the constraint matrix and
    // vector, such that the solver will compute a multiplier == 0.
    const_matrix.setZero();
    const_vector.setZero();
    set_constraint_state(loop, constraint_state);
    for (int i = 0; i < constraint_state.size(); ++i) {
      switch (constraint_state[i]) {
        case kInactive:
          break;
        case kLowerBound:
          const_matrix(i, i) = -1.0;
          const_vector(i) = lower_bound[i];
          break;
        case kUpperBound:
          const_matrix(i, i) = 1.0;
          const_vector(i) = -upper_bound[i];
          break;
      }
    }
    // Update augmented system.
    augmented_matrix.block(0, size, size, size) = const_matrix.transpose();
    augmented_matrix.block(size, 0, size, size) = const_matrix;
    augmented_vector.segment(size, size) = const_vector;
    augmented_solution =
        augmented_matrix.fullPivHouseholderQr().solve(-augmented_vector);
    const double cost_value = cost(augmented_solution.head(size));
    const bool solution_is_valid =
        satisfies_constraints(augmented_solution.head(size));
    if (solution_is_valid && cost_value < result.minimum) {
      result.solution = augmented_solution.head(size);
      result.minimum = cost_value;
    }
  }
  return result;
}
}  // namespace testing
}  // namespace collision_checking
