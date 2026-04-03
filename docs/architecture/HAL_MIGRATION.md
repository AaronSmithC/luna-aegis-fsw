/**
 * @file HAL_MIGRATION.md
 * @brief Luna-Aegis HAL Architecture — Per-App Migration Guide
 *
 * This document describes the per-app HAL pattern and how to
 * migrate apps from the monolithic la_hal.h to per-app headers.
 */

# HAL Architecture — Per-App Pattern

## Overview

Each cFS app that touches hardware owns its own HAL interface:

```
fsw/apps/<app>/hal/
  <app>_hal.h       ← Header: types + function prototypes
  <app>_hal_sim.c   ← Sim implementation (desktop)
  <app>_hal_hw.c    ← HW implementation (flight, future)
```

Apps that are pure mission-layer (no hardware access) have NO hal/
directory.  They read only from SB telemetry packets published by
other apps.

## App → HAL Mapping

| App     | HAL Header    | HW Interfaces                      |
|---------|---------------|-------------------------------------|
| PROP    | prop_hal.h    | Engine controller, cryo tanks       |
| IMU_MGR | imu_hal.h     | FOG/RLG IMU, MEMS backup IMU       |
| ALT_MGR | alt_hal.h     | Lidar altimeter, radar altimeter    |
| ACS     | acs_hal.h     | Engine gimbal, RCS valve bank       |
| EPS     | eps_hal.h     | Battery management unit             |
| TCS     | tcs_hal.h     | Heater zones, temperature sensors   |
| COMM    | comm_hal.h    | HGA/UHF antenna pointing            |
| LG      | lg_hal.h      | Landing gear actuators, load cells  |
| DOCK    | dock_hal.h    | Docking collar, seals, latches      |
| LUNET   | lunet_hal.h   | Beacon receiver (read-only)         |

## Apps with NO HAL (Pure Mission Layer)

These apps read ONLY from the Software Bus.  They never include
any *_hal.h header:

  MM, GDN, TRN, NAV, LSS, HS, LC, SC, SCH, HK, CI, TO, DS, FM

## Migration Steps (per app)

1. Create `fsw/apps/<app>/hal/` directory
2. Extract relevant types and functions from monolithic la_hal.h
   into `<app>_hal.h`, using app-prefixed names (PropHAL_, ImuHAL_...)
3. Extract relevant simulation code from la_hal_sim.c into
   `<app>_hal_sim.c` with its own static state struct
4. Update app source: `#include "../hal/<app>_hal.h"` instead of
   `#include "../../hal/inc/la_hal.h"`
5. Update CMakeLists.txt to compile `<app>_hal_sim.c` alongside
   the app source
6. Verify: app compiles with ONLY its own HAL — no reference to
   the monolithic header

## Naming Convention

Per-app HAL types use the pattern `<App>HAL_<Type>_t`:
  - PropHAL_EngineTlm_t    (not HAL_Engine_Tlm_t)
  - ImuHAL_Raw_t           (not HAL_IMU_Raw_t)
  - LgHAL_State_t          (not HAL_LG_State_t)

Per-app HAL functions use `<App>HAL_<Subsys>_<Action>`:
  - PropHAL_Engine_Read()   (not HAL_Engine_Read())
  - ImuHAL_ReadPrimary()    (not HAL_IMU_ReadPrimary())

This prevents name collisions and makes it obvious in any
source file which HAL boundary is being crossed.

## Swapping Sim → Hardware

Replace the build target:
  - Sim:  compile <app>_hal_sim.c
  - HW:   compile <app>_hal_hw.c

Nothing else changes.  The app source, the SB messages, the
gate predicates, the ground console — all unchanged.

## Relationship to Sequence Gates

Sequence gates (la_seq_gate.h) live in fsw/common/ and read
ONLY from app-level telemetry structs on the SB.  They have
ZERO knowledge of any HAL.  The layering is:

  HAL (per-app) → App → [publishes TLM on SB] → Gate reads TLM

If you swap a HAL sim for real hardware, the gate predicates
don't change — they only see the published packet values.
