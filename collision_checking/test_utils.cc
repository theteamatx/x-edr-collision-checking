#include "collision_checking/test_utils.h"

#include "third_party/absl/base/call_once.h"
#include "third_party/absl/base/internal/malloc_hook.h"
#include "third_party/absl/log/absl_log.h"

namespace collision_checking {
namespace testing {

namespace {

#if __has_feature(address_sanitizer)
#define ENABLE_MALLOC_COUNTER false
#elif __has_feature(hwaddress_sanitizer)
#define ENABLE_MALLOC_COUNTER false
#elif __has_feature(memory_sanitizer)
#define ENABLE_MALLOC_COUNTER false
#elif __has_feature(thread_sanitizer)
#define ENABLE_MALLOC_COUNTER false
#else
#define ENABLE_MALLOC_COUNTER true
#endif

constexpr bool kEnableMallocCounter = ENABLE_MALLOC_COUNTER;

absl::once_flag g_once_flag;
std::atomic<size_t> g_allocs = 0;
std::atomic<size_t> g_frees = 0;

void NewHook(const void *ptr, size_t size) { g_allocs++; }

void DeleteHook(const void *ptr) {
  if (ptr == nullptr) {
    return;
  }
  g_frees++;
}
}  // namespace

void MallocCounterInitAndClear() {
  if constexpr (kEnableMallocCounter) {
    // Install hooks on the first invocation.
    absl::call_once(g_once_flag, []() {
      absl::base_internal::MallocHook::AddNewHook(&NewHook);
      absl::base_internal::MallocHook::AddDeleteHook(&DeleteHook);
    });
  } else {
    absl::call_once(g_once_flag, []() {
      ABSL_LOG(INFO) << "Malloc checker disabled in this mode.";
    });
  }

  // Reset counters.
  g_allocs = 0;
  g_frees = 0;
}

int MallocCounterGetAllocations() { return g_allocs; }

int MallocCounterGetFrees() { return g_frees; }

bool MallocCounterIsAvailable() { return kEnableMallocCounter; }

}  // namespace testing
}  // namespace collision_checking
