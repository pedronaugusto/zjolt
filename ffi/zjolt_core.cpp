//===----------------------------------------------------------------------===//
// zjolt — initialisation, the allocator seam, the diagnostic hooks, and the
// job system.
//===----------------------------------------------------------------------===//

// zjolt_internal.h must come first: it opens with <Jolt/Jolt.h>, and every Jolt
// header assumes that one has already been seen. This ordering is the same in
// every ffi translation unit.
#include "zjolt_internal.h"

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/RegisterTypes.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>

namespace {

//===----------------------------------------------------------------------===//
// Per-thread error detail
//
// Jolt reports shape construction and deserialisation failures as strings. A
// flat result enum would throw them away, and threading an out-parameter
// through every constructor would make the common success path pay for the
// rare failure. A thread-local buffer keeps the detail reachable without
// either cost, and being per-thread means a cook running on a worker cannot
// clobber the message another thread is about to read.
//===----------------------------------------------------------------------===//

constexpr size_t kErrorCapacity = 256;
thread_local char g_error[kErrorCapacity] = {0};

void CopyMessage(const char *message) {
  if (message == nullptr) {
    g_error[0] = '\0';
    return;
  }
  size_t i = 0;
  for (; i + 1 < kErrorCapacity && message[i] != '\0'; ++i)
    g_error[i] = message[i];
  g_error[i] = '\0';
}

//===----------------------------------------------------------------------===//
// Allocator seam
//
// Jolt routes every allocation through five global function pointers. The host
// table is held here and the five thunks below add the `user` argument Jolt's
// signatures have no room for.
//===----------------------------------------------------------------------===//

ZJoltAllocator g_allocator = {};

void *HostAllocate(size_t size) {
  return g_allocator.allocate(g_allocator.user, size);
}
void *HostReallocate(void *block, size_t old_size, size_t new_size) {
  return g_allocator.reallocate(g_allocator.user, block, old_size, new_size);
}
void HostFree(void *block) { g_allocator.free(g_allocator.user, block); }
void *HostAlignedAllocate(size_t size, size_t alignment) {
  return g_allocator.aligned_allocate(g_allocator.user, size, alignment);
}
void HostAlignedFree(void *block) {
  g_allocator.aligned_free(g_allocator.user, block);
}

//===----------------------------------------------------------------------===//
// Diagnostic hooks
//===----------------------------------------------------------------------===//

ZJoltTraceFn g_trace = nullptr;
ZJoltAssertFailedFn g_assert_failed = nullptr;
void *g_hooks_user = nullptr;

JPH::TraceFunction g_saved_trace = nullptr;
#ifdef JPH_ENABLE_ASSERTS
JPH::AssertFailedFunction g_saved_assert_failed = nullptr;
#endif

/// Jolt's trace hook is varargs. Formatting here rather than in the ABI is
/// what lets the C callback be an ordinary two-argument function that any
/// language can implement.
///
/// This is installed unconditionally, even when the host supplies no callback,
/// and that is not tidiness. Jolt's OWN default is `DummyTrace`, whose entire
/// body is `JPH_ASSERT(false)` — so in a build with asserts on, the first time
/// Jolt has anything to say it kills the process instead of saying it. Falling
/// back to stderr turns that into a line of text.
void TraceThunk(const char *format, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';

  if (g_trace != nullptr) {
    g_trace(g_hooks_user, buffer);
    return;
  }
  fputs(buffer, stderr);
  fputc('\n', stderr);
}

#ifdef JPH_ENABLE_ASSERTS
bool AssertFailedThunk(const char *expression, const char *message,
                       const char *file, JPH::uint line) {
  if (g_assert_failed == nullptr) return true;
  return g_assert_failed(g_hooks_user, expression, message, file,
                         static_cast<uint32_t>(line));
}
#endif

bool g_initialized = false;

/// Physics systems, job systems and characters currently alive. Atomic because
/// nothing stops a host creating them from more than one thread.
std::atomic<int32_t> g_live_handles{0};

//===----------------------------------------------------------------------===//
// Job system
//===----------------------------------------------------------------------===//

void DestroyThreadPool(JPH::JobSystem *impl) {
  zjolt::Delete(static_cast<JPH::JobSystemThreadPool *>(impl));
}

void DestroySingleThreaded(JPH::JobSystem *impl) {
  zjolt::Delete(static_cast<JPH::JobSystemSingleThreaded *>(impl));
}

}  // namespace

namespace zjolt {

ZJoltResult SetError(ZJoltResult result, const char *message) {
  CopyMessage(message);
  return result;
}

void ClearError() { g_error[0] = '\0'; }

bool IsInitialized() { return g_initialized; }

void HandleCreated() { g_live_handles.fetch_add(1, std::memory_order_relaxed); }

void HandleDestroyed() { g_live_handles.fetch_sub(1, std::memory_order_relaxed); }

}  // namespace zjolt

extern "C" {

//===----------------------------------------------------------------------===//
// Version and configuration
//===----------------------------------------------------------------------===//

uint32_t zjoltVersion(void) {
  return (static_cast<uint32_t>(ZJOLT_VERSION_MAJOR) << 16) |
         (static_cast<uint32_t>(ZJOLT_VERSION_MINOR) << 8) |
         static_cast<uint32_t>(ZJOLT_VERSION_PATCH);
}

uint32_t zjoltJoltVersion(void) {
  // Comes from Jolt's own Core.h, so a re-vendor moves it automatically and
  // this cannot go stale the way a hand-typed constant would.
  return (static_cast<uint32_t>(JPH_VERSION_MAJOR) << 16) |
         (static_cast<uint32_t>(JPH_VERSION_MINOR) << 8) |
         static_cast<uint32_t>(JPH_VERSION_PATCH);
}

uint32_t zjoltConfigId(void) { return static_cast<uint32_t>(ZJOLT_CONFIG_ID); }

const char *zjoltResultName(ZJoltResult result) {
  switch (result) {
    case ZJOLT_RESULT_OK:
      return "ok";
    case ZJOLT_RESULT_NOT_INITIALIZED:
      return "not initialized";
    case ZJOLT_RESULT_ALREADY_INITIALIZED:
      return "already initialized";
    case ZJOLT_RESULT_CONFIG_MISMATCH:
      return "config mismatch";
    case ZJOLT_RESULT_OUT_OF_MEMORY:
      return "out of memory";
    case ZJOLT_RESULT_INVALID_ARGUMENT:
      return "invalid argument";
    case ZJOLT_RESULT_BUFFER_TOO_SMALL:
      return "buffer too small";
    case ZJOLT_RESULT_SHAPE_INVALID:
      return "shape invalid";
    case ZJOLT_RESULT_BAD_FORMAT:
      return "bad format";
    case ZJOLT_RESULT_BODY_NOT_FOUND:
      return "body not found";
    case ZJOLT_RESULT_UNSUPPORTED:
      return "unsupported in this build";
  }
  return "unknown result";
}

const char *zjoltLastError(void) { return g_error; }

size_t zjoltDefaultAllocateAlignment(void) {
  return static_cast<size_t>(JPH_DEFAULT_ALLOCATE_ALIGNMENT);
}

//===----------------------------------------------------------------------===//
// Initialisation
//===----------------------------------------------------------------------===//

ZJoltResult zjoltInitWithConfig(const ZJoltInitDesc *desc, uint32_t config_id) {
  zjolt::ClearError();

  if (g_initialized) {
    return zjolt::SetError(ZJOLT_RESULT_ALREADY_INITIALIZED,
                           "zjoltInit called twice without zjoltDeinit");
  }

  // Checked before anything is installed, so a mismatched consumer leaves the
  // process exactly as it found it. This is the guard that turns a
  // header/library skew — which would otherwise misread every position handed
  // across the boundary — into a start-up error.
  if (config_id != static_cast<uint32_t>(ZJOLT_CONFIG_ID)) {
    return zjolt::SetError(
        ZJOLT_RESULT_CONFIG_MISMATCH,
        "zjolt.h was compiled with different layout-affecting settings than "
        "the library (double precision, or object layer bits)");
  }

  if (desc != nullptr && desc->allocator != nullptr) {
    const ZJoltAllocator &host = *desc->allocator;
    if (host.allocate == nullptr || host.reallocate == nullptr ||
        host.free == nullptr || host.aligned_allocate == nullptr ||
        host.aligned_free == nullptr) {
      return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                             "every ZJoltAllocator function pointer is "
                             "required; none may be null");
    }
    g_allocator = host;
    JPH::Allocate = HostAllocate;
    JPH::Reallocate = HostReallocate;
    JPH::Free = HostFree;
    JPH::AlignedAllocate = HostAlignedAllocate;
    JPH::AlignedFree = HostAlignedFree;
  } else {
    g_allocator = ZJoltAllocator{};
    JPH::RegisterDefaultAllocator();
  }

  g_hooks_user = desc != nullptr ? desc->hooks_user : nullptr;

  g_saved_trace = JPH::Trace;
  g_trace = desc != nullptr ? desc->trace : nullptr;
  // Always, not only when a hook was supplied — see TraceThunk.
  JPH::Trace = TraceThunk;

#ifdef JPH_ENABLE_ASSERTS
  g_saved_assert_failed = JPH::AssertFailed;
  g_assert_failed = desc != nullptr ? desc->assert_failed : nullptr;
  if (g_assert_failed != nullptr) JPH::AssertFailed = AssertFailedThunk;
#else
  // Recorded so a host can see its hook was accepted but will never fire.
  g_assert_failed = desc != nullptr ? desc->assert_failed : nullptr;
#endif

  // Jolt's own `new Factory()` is a placement-new on an unchecked allocation
  // (JPH_OVERRIDE_NEW_DELETE), so a failing allocator would construct at
  // address zero. Allocating it here instead is what makes the out-of-memory
  // path at start-up an error rather than a null dereference.
  JPH::Factory *factory = zjolt::New<JPH::Factory>();
  if (factory == nullptr) {
    JPH::RegisterDefaultAllocator();
    return zjolt::SetError(ZJOLT_RESULT_OUT_OF_MEMORY,
                           "could not allocate Jolt's type factory");
  }
  JPH::Factory::sInstance = factory;

  // Compares the caller's JPH_VERSION_ID against the library's and aborts on a
  // mismatch. It cannot fire here: build.zig applies one macro set to the Jolt
  // translation units and this one alike, so the two are the same by
  // construction. See UPSTREAM.md.
  JPH::RegisterTypes();

  g_initialized = true;
  return ZJOLT_RESULT_OK;
}

uint32_t zjoltLiveHandleCount(void) {
  const int32_t count = g_live_handles.load(std::memory_order_relaxed);
  return count > 0 ? static_cast<uint32_t>(count) : 0u;
}

void zjoltDeinit(void) {
  if (!g_initialized) return;

  // Refusing is not pedantry. The last thing this function does is give Jolt
  // its own allocator back; a handle destroyed after that is freed through an
  // allocator it was never allocated from, and nothing goes wrong until much
  // later somewhere else. Leaving the library up instead means the eventual
  // destroy is still correct, and the trace says what to fix.
  const int32_t live = g_live_handles.load(std::memory_order_relaxed);
  if (live > 0) {
    JPH::Trace(
        "zjoltDeinit: %d handle(s) still alive (physics systems, job systems "
        "or characters). Nothing was torn down, because restoring Jolt's "
        "allocator now would make destroying them corrupt the heap. Destroy "
        "them and call zjoltDeinit again.",
        static_cast<int>(live));
    return;
  }

  JPH::UnregisterTypes();
  zjolt::Delete(JPH::Factory::sInstance);
  JPH::Factory::sInstance = nullptr;

  if (g_saved_trace != nullptr) JPH::Trace = g_saved_trace;
  g_saved_trace = nullptr;
#ifdef JPH_ENABLE_ASSERTS
  if (g_saved_assert_failed != nullptr) JPH::AssertFailed = g_saved_assert_failed;
  g_saved_assert_failed = nullptr;
#endif
  g_trace = nullptr;
  g_assert_failed = nullptr;
  g_hooks_user = nullptr;

  // Restored last: everything freed above had to go through the allocator it
  // was allocated from.
  JPH::RegisterDefaultAllocator();
  g_allocator = ZJoltAllocator{};

  g_initialized = false;
}

bool zjoltIsInitialized(void) { return g_initialized; }

//===----------------------------------------------------------------------===//
// Job system
//===----------------------------------------------------------------------===//

ZJoltResult zjoltJobSystemCreateThreadPool(uint32_t max_jobs,
                                           uint32_t max_barriers,
                                           int32_t num_threads,
                                           ZJoltJobSystem **out) {
  zjolt::ClearError();
  if (out == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  *out = nullptr;
  if (!zjolt::IsInitialized()) return ZJOLT_RESULT_NOT_INITIALIZED;
  if (max_jobs == 0 || max_barriers == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "max_jobs and max_barriers must be positive");
  }

  ZJoltJobSystem *handle = zjolt::New<ZJoltJobSystem>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  JPH::JobSystemThreadPool *pool = zjolt::New<JPH::JobSystemThreadPool>(
      static_cast<JPH::uint>(max_jobs), static_cast<JPH::uint>(max_barriers),
      static_cast<int>(num_threads));
  if (pool == nullptr) {
    zjolt::Delete(handle);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }

  handle->impl = pool;
  handle->destroy = DestroyThreadPool;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltJobSystemCreateSingleThreaded(uint32_t max_jobs,
                                               ZJoltJobSystem **out) {
  zjolt::ClearError();
  if (out == nullptr) return ZJOLT_RESULT_INVALID_ARGUMENT;
  *out = nullptr;
  if (!zjolt::IsInitialized()) return ZJOLT_RESULT_NOT_INITIALIZED;
  if (max_jobs == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "max_jobs must be positive");
  }

  ZJoltJobSystem *handle = zjolt::New<ZJoltJobSystem>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  JPH::JobSystemSingleThreaded *single =
      zjolt::New<JPH::JobSystemSingleThreaded>(static_cast<JPH::uint>(max_jobs));
  if (single == nullptr) {
    zjolt::Delete(handle);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }

  handle->impl = single;
  handle->destroy = DestroySingleThreaded;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltJobSystemDestroy(ZJoltJobSystem *job_system) {
  if (job_system == nullptr) return;
  job_system->destroy(job_system->impl);
  zjolt::Delete(job_system);
  zjolt::HandleDestroyed();
}

uint32_t zjoltJobSystemGetMaxConcurrency(const ZJoltJobSystem *job_system) {
  if (job_system == nullptr) return 0;
  return static_cast<uint32_t>(job_system->impl->GetMaxConcurrency());
}

}  // extern "C"
