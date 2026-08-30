//===----------------------------------------------------------------------===//
// zjolt — initialisation, the allocator seam, the diagnostic hooks, and the
// job system.
//===----------------------------------------------------------------------===//

// zjolt_internal.h must come first: it opens with <Jolt/Jolt.h>, and every Jolt
// header assumes that one has already been seen. This ordering is the same in
// every ffi translation unit.
#include "zjolt_internal.h"

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/FPFlushDenormals.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/RegisterTypes.h>

#ifdef JPH_EXTERNAL_PROFILE
#include <Jolt/Core/Profiler.h>
#endif  // JPH_EXTERNAL_PROFILE

#include <atomic>
#include <cstdarg>
#include <cstdio>

namespace {

//===----------------------------------------------------------------------===//
// Per-thread error detail
//
// Jolt reports shape construction/deserialisation failures as strings; a flat result enum would throw them away, and an out-parameter on every constructor would make the common success path pay for the rare failure. Thread-local so a cook running on a worker cannot clobber the message another thread is about to read.
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
// Jolt routes every allocation through five global function pointers; the host table is held here, and the five thunks below add the `user` argument Jolt's signatures have no room for.
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

/// Jolt's trace hook is varargs; formatting here lets the C callback be an
/// ordinary two-argument function any language can implement.
///
/// Installed unconditionally, even with no host callback: Jolt's OWN
/// default, `DummyTrace`, is `JPH_ASSERT(false)`, so with asserts on the
/// first trace would kill the process. Falling back to stderr instead.
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

//===----------------------------------------------------------------------===//
// Host job system
//
// JobSystem::Job is `protected`, since Jolt expects only a JobSystem override to touch one. HostJobSystem below IS such a subclass; the free functions later (zjoltJobRun and friends) are not, and borrow the access through JobTypeAccess purely to name the type — every member they call through it (Execute, AddRef, Release) is itself public on Job.
//===----------------------------------------------------------------------===//

/// Runs Jolt's step on a host's own task graph instead of spawning threads
/// of its own. JobSystemWithBarrier — the base JobSystemThreadPool also
/// builds on — supplies every barrier virtual (CreateBarrier, WaitForJobs,
/// Barrier::AddJob, Job::SetBarrier, OnJobFinished, ...), so this only
/// implements the five answering "how many jobs at once" and "where does
/// this one run": GetMaxConcurrency, CreateJob, QueueJob, QueueJobs, FreeJob.
class HostJobSystem final : public JPH::JobSystemWithBarrier {
 public:
  HostJobSystem(const ZJoltHostJobSystem &host, JPH::uint max_barriers)
      : JPH::JobSystemWithBarrier(max_barriers), host_(host) {}

  int GetMaxConcurrency() const override {
    return static_cast<int>(host_.get_max_concurrency(host_.user));
  }

  JobHandle CreateJob(const char *inJobName, JPH::ColorArg inColor,
                      const JobFunction &inJobFunction,
                      JPH::uint32 inNumDependencies) override {
    // Job's own constructor and destructor are public; only the TYPE is
    // protected, and this class has access to it as a JobSystem subclass.
    // JPH_OVERRIDE_NEW_DELETE on Job routes this through JPH::Allocate, so a
    // host allocator installed at zjoltInit sees this too.
    Job *job = new Job(inJobName, inColor, this, inJobFunction, inNumDependencies);
    JobHandle handle(job);
    // Mirrors JobSystemThreadPool::CreateJob: a job with no dependencies is
    // ready the instant it exists, and nothing else will ever queue it.
    if (inNumDependencies == 0) QueueJob(job);
    return handle;
  }

 protected:
  void QueueJob(Job *inJob) override {
    // Jolt's own contract (JobSystem.h): the job is guaranteed alive for the
    // duration of this call only. The reference taken here is what the host
    // gives back through zjoltJobRelease once it is done with it — exactly
    // the same bookkeeping JobSystemThreadPool::QueueJobInternal does before
    // handing a job to its own queue.
    inJob->AddRef();
    host_.queue_job(host_.user, reinterpret_cast<ZJoltJob *>(inJob));
  }

  void QueueJobs(Job **inJobs, JPH::uint inNumJobs) override {
    for (JPH::uint i = 0; i < inNumJobs; ++i) inJobs[i]->AddRef();
    // A ZJoltJob* is a bare reinterpret_cast tag over Job* (see the note on
    // ZJoltShape in zjolt_internal.h for why that conversion is sound), so
    // the whole array converts at once rather than element by element.
    host_.queue_jobs(host_.user, reinterpret_cast<ZJoltJob *const *>(inJobs),
                     static_cast<uint32_t>(inNumJobs));
  }

  void FreeJob(Job *inJob) override { delete inJob; }

 private:
  ZJoltHostJobSystem host_;
};

void DestroyHost(JPH::JobSystem *impl) {
  zjolt::Delete(static_cast<HostJobSystem *>(impl));
}

/// See the section comment above: this borrows a JobSystem subclass's access
/// to its own protected nested Job type, purely to name it from ordinary
/// code. Never instantiated — JobSystem is abstract, and nothing here needs
/// an instance, only the name.
class JobTypeAccess : private JPH::JobSystem {
 public:
  using Job = JPH::JobSystem::Job;
};
using HostJob = JobTypeAccess::Job;

inline HostJob *ToJoltJob(ZJoltJob *job) {
  return reinterpret_cast<HostJob *>(job);
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

uint32_t zjoltCpuFeatures(void) {
  uint32_t features = 0;
#ifdef JPH_USE_SSE
  features |= ZJOLT_CPU_FEATURE_SSE;
#endif
#ifdef JPH_USE_SSE4_1
  features |= ZJOLT_CPU_FEATURE_SSE4_1;
#endif
#ifdef JPH_USE_SSE4_2
  features |= ZJOLT_CPU_FEATURE_SSE4_2;
#endif
#ifdef JPH_USE_AVX
  features |= ZJOLT_CPU_FEATURE_AVX;
#endif
#ifdef JPH_USE_AVX2
  features |= ZJOLT_CPU_FEATURE_AVX2;
#endif
#ifdef JPH_USE_AVX512
  features |= ZJOLT_CPU_FEATURE_AVX512;
#endif
#ifdef JPH_USE_F16C
  features |= ZJOLT_CPU_FEATURE_F16C;
#endif
#ifdef JPH_USE_LZCNT
  features |= ZJOLT_CPU_FEATURE_LZCNT;
#endif
#ifdef JPH_USE_TZCNT
  features |= ZJOLT_CPU_FEATURE_TZCNT;
#endif
#ifdef JPH_USE_FMADD
  features |= ZJOLT_CPU_FEATURE_FMADD;
#endif
#ifdef JPH_USE_NEON
  features |= ZJOLT_CPU_FEATURE_NEON;
#endif
  return features;
}

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
    case ZJOLT_RESULT_IO_ERROR:
      return "stream I/O error";
    case ZJOLT_RESULT_STATE_INCOMPLETE:
      return "state incomplete";
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
        "zjoltDeinit: %d handle(s) still alive. Nothing was torn down, because "
        "restoring Jolt's allocator now would make destroying them corrupt the "
        "heap. Destroy them and call zjoltDeinit again. The kinds counted here "
        "are physics systems, job systems, characters, character contact "
        "listeners, character-vs-character collisions, debug renderers, "
        "compute systems, hair, vehicle constraints, and ragdolls.",
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
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
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
  handle->kind = ZJoltJobSystemKind::ThreadPool;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltJobSystemCreateThreadPoolWithHooks(
    uint32_t max_jobs, uint32_t max_barriers, int32_t num_threads,
    ZJoltThreadHookFn thread_init, ZJoltThreadHookFn thread_exit, void *user,
    ZJoltJobSystem **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (max_jobs == 0 || max_barriers == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "max_jobs and max_barriers must be positive");
  }

  ZJoltJobSystem *handle = zjolt::New<ZJoltJobSystem>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  // Default-constructed and left un-started: Init below is what launches the
  // worker threads, and SetThreadInitFunction/SetThreadExitFunction only take
  // effect on threads that start AFTER they are set — see the declaration's
  // comment in zjolt_core.h for why this cannot be a setter on an already-
  // created pool.
  JPH::JobSystemThreadPool *pool = zjolt::New<JPH::JobSystemThreadPool>();
  if (pool == nullptr) {
    zjolt::Delete(handle);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }

  if (thread_init != nullptr) {
    pool->SetThreadInitFunction(
        [thread_init, user](int index) { thread_init(user, static_cast<int32_t>(index)); });
  }
  if (thread_exit != nullptr) {
    pool->SetThreadExitFunction(
        [thread_exit, user](int index) { thread_exit(user, static_cast<int32_t>(index)); });
  }
  pool->Init(static_cast<JPH::uint>(max_jobs), static_cast<JPH::uint>(max_barriers),
            static_cast<int>(num_threads));

  handle->impl = pool;
  handle->destroy = DestroyThreadPool;
  handle->kind = ZJoltJobSystemKind::ThreadPool;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltJobSystemSetNumThreads(ZJoltJobSystem *job_system,
                                        int32_t num_threads) {
  ZJOLT_ENTER();
  if (!zjolt::Present(job_system)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (job_system->kind != ZJoltJobSystemKind::ThreadPool) {
    return zjolt::SetError(
        ZJOLT_RESULT_INVALID_ARGUMENT,
        "zjoltJobSystemSetNumThreads only applies to a job system created by "
        "zjoltJobSystemCreateThreadPool or zjoltJobSystemCreateThreadPoolWithHooks");
  }
  static_cast<JPH::JobSystemThreadPool *>(job_system->impl)
      ->SetNumThreads(static_cast<int>(num_threads));
  return ZJOLT_RESULT_OK;
}

ZJoltResult zjoltJobSystemCreateHost(const ZJoltHostJobSystem *host,
                                     uint32_t max_barriers,
                                     ZJoltJobSystem **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(host, out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
  if (host->get_max_concurrency == nullptr || host->queue_job == nullptr ||
      host->queue_jobs == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "a host job system needs get_max_concurrency, "
                           "queue_job and queue_jobs");
  }
  if (max_barriers == 0) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "max_barriers must be positive");
  }

  ZJoltJobSystem *handle = zjolt::New<ZJoltJobSystem>();
  if (handle == nullptr) return ZJOLT_RESULT_OUT_OF_MEMORY;

  HostJobSystem *impl =
      zjolt::New<HostJobSystem>(*host, static_cast<JPH::uint>(max_barriers));
  if (impl == nullptr) {
    zjolt::Delete(handle);
    return ZJOLT_RESULT_OUT_OF_MEMORY;
  }

  handle->impl = impl;
  handle->destroy = DestroyHost;
  handle->kind = ZJoltJobSystemKind::Host;
  zjolt::HandleCreated();
  *out = handle;
  return ZJOLT_RESULT_OK;
}

void zjoltJobRun(ZJoltJob *job) {
  if (job == nullptr) return;
  ToJoltJob(job)->Execute();
}

void zjoltJobAddRef(ZJoltJob *job) {
  if (job == nullptr) return;
  ToJoltJob(job)->AddRef();
}

void zjoltJobRelease(ZJoltJob *job) {
  if (job == nullptr) return;
  ToJoltJob(job)->Release();
}

ZJoltResult zjoltJobSystemCreateSingleThreaded(uint32_t max_jobs,
                                               ZJoltJobSystem **out) {
  ZJOLT_ENTER(out);
  if (!zjolt::Present(out)) return ZJOLT_RESULT_INVALID_ARGUMENT;
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
  handle->kind = ZJoltJobSystemKind::SingleThreaded;
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

//===----------------------------------------------------------------------===//
// Factory
//===----------------------------------------------------------------------===//

namespace {

void FillRttiInfo(const JPH::RTTI *rtti, ZJoltRttiInfo *out) {
  if (rtti == nullptr) {
    *out = ZJoltRttiInfo{};
    return;
  }
  out->name = rtti->GetName();
  out->hash = rtti->GetHash();
  out->size = static_cast<int32_t>(rtti->GetSize());
  out->is_abstract = rtti->IsAbstract();
}

}  // namespace

void zjoltFactoryFind(const char *name, ZJoltRttiInfo *out) {
  if (out == nullptr) return;
  *out = ZJoltRttiInfo{};
  if (name == nullptr || JPH::Factory::sInstance == nullptr) return;
  FillRttiInfo(JPH::Factory::sInstance->Find(name), out);
}

void zjoltFactoryFindByHash(uint32_t hash, ZJoltRttiInfo *out) {
  if (out == nullptr) return;
  *out = ZJoltRttiInfo{};
  if (JPH::Factory::sInstance == nullptr) return;
  FillRttiInfo(JPH::Factory::sInstance->Find(static_cast<JPH::uint32>(hash)), out);
}

ZJoltResult zjoltFactoryGetAllClasses(ZJoltRttiInfo *out, uint32_t capacity,
                                      uint32_t *out_count) {
  ZJOLT_ENTER(out_count);
  if (!zjolt::Present(out_count)) return ZJOLT_RESULT_INVALID_ARGUMENT;

  if (JPH::Factory::sInstance == nullptr) {
    *out_count = 0;
    return ZJOLT_RESULT_OK;
  }

  const JPH::Array<const JPH::RTTI *> classes =
      JPH::Factory::sInstance->GetAllClasses();
  const uint32_t count = static_cast<uint32_t>(classes.size());
  *out_count = count;
  if (out == nullptr) return ZJOLT_RESULT_OK;
  if (capacity < count) return ZJOLT_RESULT_BUFFER_TOO_SMALL;
  for (uint32_t i = 0; i < count; ++i) FillRttiInfo(classes[i], &out[i]);
  return ZJOLT_RESULT_OK;
}

//===----------------------------------------------------------------------===//
// Floating-point control word
//
// FPFlushDenormals.h stubs the guard to an empty, trivially
// constructible/destructible class on a target with no control word (WASM,
// RISC-V, PPC, LoongArch), so this needs no platform guard of its own.
//===----------------------------------------------------------------------===//

static_assert(sizeof(JPH::FPFlushDenormals) <= sizeof(ZJoltFPControlWordState),
              "ZJoltFPControlWordState must be at least as large as JPH::FPFlushDenormals");
static_assert(alignof(ZJoltFPControlWordState) >= alignof(JPH::FPFlushDenormals),
              "ZJoltFPControlWordState must be at least as aligned as JPH::FPFlushDenormals");

void zjoltFPFlushDenormalsEnter(ZJoltFPControlWordState *out) {
  if (out == nullptr) return;
  new (out) JPH::FPFlushDenormals();
}

void zjoltFPFlushDenormalsLeave(ZJoltFPControlWordState *state) {
  if (state == nullptr) return;
  reinterpret_cast<JPH::FPFlushDenormals *>(state)->~FPFlushDenormals();
}

//===----------------------------------------------------------------------===//
// External profiler bridge
//
// JPH::ExternalProfileMeasurement is declared by Jolt only under
// JPH_EXTERNAL_PROFILE, with its constructor/destructor left undefined for a
// statically-linked build (Profiler.h's own comment) — zjolt is always
// statically linked into this library, so these definitions are what makes
// the type link at all when the macro is on.
//===----------------------------------------------------------------------===//

#ifdef JPH_EXTERNAL_PROFILE

namespace {
ZJoltExternalProfilerStartFn g_external_profiler_start = nullptr;
ZJoltExternalProfilerEndFn g_external_profiler_end = nullptr;
void *g_external_profiler_user = nullptr;
}  // namespace

JPH_NAMESPACE_BEGIN

ExternalProfileMeasurement::ExternalProfileMeasurement(const char *inName, uint32 inColor) {
  std::memset(mUserData, 0, sizeof(mUserData));
  if (g_external_profiler_start != nullptr) {
    g_external_profiler_start(g_external_profiler_user, inName, inColor, mUserData);
  }
}

ExternalProfileMeasurement::~ExternalProfileMeasurement() {
  if (g_external_profiler_end != nullptr) {
    g_external_profiler_end(g_external_profiler_user, mUserData);
  }
}

JPH_NAMESPACE_END

#endif  // JPH_EXTERNAL_PROFILE

ZJoltResult zjoltExternalProfilerSetHooks(ZJoltExternalProfilerStartFn start,
                                          ZJoltExternalProfilerEndFn end,
                                          void *user) {
  ZJOLT_ENTER();
#ifdef JPH_EXTERNAL_PROFILE
  if (start == nullptr || end == nullptr) {
    return zjolt::SetError(ZJOLT_RESULT_INVALID_ARGUMENT,
                           "zjoltExternalProfilerSetHooks requires both hooks");
  }
  g_external_profiler_start = start;
  g_external_profiler_end = end;
  g_external_profiler_user = user;
  return ZJOLT_RESULT_OK;
#else
  (void)start;
  (void)end;
  (void)user;
  return ZJOLT_RESULT_UNSUPPORTED;
#endif  // JPH_EXTERNAL_PROFILE
}

void zjoltExternalProfilerClearHooks(void) {
#ifdef JPH_EXTERNAL_PROFILE
  g_external_profiler_start = nullptr;
  g_external_profiler_end = nullptr;
  g_external_profiler_user = nullptr;
#endif  // JPH_EXTERNAL_PROFILE
}

}  // extern "C"
