//===----------------------------------------------------------------------===//
// zjolt — a test-only window onto the two globals Jolt keeps its diagnostic
// hooks in.
//
// The C ABI does not report which trace or assert-failure function is
// currently installed, and it should not: those are Jolt's globals, not
// zjolt's state, and a host has no call to read them. A test does. "A call
// that reported failure left the process exactly as it found it" is a claim
// about those two pointers and about nothing else the ABI can be asked, so
// without this the claim cannot be checked at all — the only path that
// reaches the trace hook through the ABI needs a library that came up.
//
// Compiled into the Zig test binary only. Not part of the library, not an
// installed header, and nothing outside src/*_test.zig may call it.
//===----------------------------------------------------------------------===//

#include <Jolt/Jolt.h>

#include <Jolt/Core/IssueReporting.h>

extern "C" {

/// `JPH::Trace` as it stands right now, as an opaque address: a test compares
/// it against the same reading taken earlier, never against a function it can
/// name — zjolt's own thunks have internal linkage.
const void *zjoltTestTraceHook(void) {
  return reinterpret_cast<const void *>(JPH::Trace);
}

/// `JPH::AssertFailed`, same rules. Null in a build without asserts, where
/// Jolt declares no such global and zjolt installs nothing.
const void *zjoltTestAssertFailedHook(void) {
#ifdef JPH_ENABLE_ASSERTS
  return reinterpret_cast<const void *>(JPH::AssertFailed);
#else
  return nullptr;
#endif
}

}  // extern "C"
