//===----------------------------------------------------------------------===//
// zjolt — a C ABI over Jolt Physics.
//
// This is the umbrella header: include it and you have the whole surface. The
// parts it pulls in are split by concern rather than by convenience, and each
// one stands on its own given zjolt_core.h — but there is no reason for a
// consumer to reach for them individually, and one include is the contract.
//
// The conventions every part follows, and the guarantees the boundary makes,
// are documented at the top of zjolt_core.h. Read that first.
//===----------------------------------------------------------------------===//

#ifndef ZJOLT_H_
#define ZJOLT_H_

#include "zjolt_core.h"

#include "zjolt_batch.h"
#include "zjolt_body.h"
#include "zjolt_broadphase.h"
#include "zjolt_character.h"
#include "zjolt_group.h"
#include "zjolt_material.h"
#include "zjolt_query.h"
#include "zjolt_shape.h"
#include "zjolt_state.h"
#include "zjolt_system.h"
#include "zjolt_vehicle.h"

#endif  // ZJOLT_H_
