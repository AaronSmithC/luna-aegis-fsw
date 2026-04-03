/**
 * @file la_mm_gates.h
 * @brief Mission Manager Gate Predicates
 *
 * Each predicate reads from a cached telemetry packet that MM
 * already subscribes to on the Software Bus.  The predicates
 * know ONLY about packet field values — never HAL types, HAL
 * enums, or HAL return codes.
 *
 * The magic numbers here (e.g., EngineState == 3) correspond
 * to the values that PROP *publishes* in its StatusPkt.  If
 * PROP changes its telemetry encoding, these change too — but
 * HAL changes are invisible to this file.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 */

#ifndef LA_MM_GATES_H
#define LA_MM_GATES_H

#include "../../msg_defs/la_msg_structs.h"
#include <stdbool.h>

/* ── PROP predicates ─────────────────────────────────────── */

/** Engine reports RUNNING (EngineState == 3 in PROP StatusPkt) */
static inline bool MMGate_EngineRunning(const void *ctx)
{
    const LA_PROP_StatusPkt_t *p = (const LA_PROP_StatusPkt_t *)ctx;
    return p->EngineState == 3;  /* Value PROP publishes for RUNNING */
}

/** Engine reports OFF or SHUTDOWN (safe to transition away) */
static inline bool MMGate_EngineOff(const void *ctx)
{
    const LA_PROP_StatusPkt_t *p = (const LA_PROP_StatusPkt_t *)ctx;
    return p->EngineState == 0 || p->EngineState == 4;
    /* 0 = OFF, 4 = SHUTDOWN */
}

/** Propellant above abort reserve (50 kg combined) */
static inline bool MMGate_PropAboveReserve(const void *ctx)
{
    const LA_PROP_StatusPkt_t *p = (const LA_PROP_StatusPkt_t *)ctx;
    return (p->OxMass_kg + p->FuelMass_kg) > 50.0;
}

/* ── LG predicates ───────────────────────────────────────── */

/** Landing gear reports DEPLOYED (DeployState == 2) */
static inline bool MMGate_GearDeployed(const void *ctx)
{
    const LA_LG_StatusPkt_t *p = (const LA_LG_StatusPkt_t *)ctx;
    return p->DeployState == 2;  /* DEPLOYED */
}

/** Landing gear reports STOWED (DeployState == 0) */
static inline bool MMGate_GearStowed(const void *ctx)
{
    const LA_LG_StatusPkt_t *p = (const LA_LG_StatusPkt_t *)ctx;
    return p->DeployState == 0;  /* STOWED */
}

/** Touchdown detected on all 4 legs */
static inline bool MMGate_TouchdownAll(const void *ctx)
{
    const LA_LG_StatusPkt_t *p = (const LA_LG_StatusPkt_t *)ctx;
    return p->TouchdownDetect == 0x0F;  /* All 4 legs */
}

/* ── NAV predicates ──────────────────────────────────────── */

/** Vertical velocity below threshold (near-hover) */
static inline bool MMGate_VelBelowThreshold(const void *ctx)
{
    const LA_NAV_StatePkt_t *p = (const LA_NAV_StatePkt_t *)ctx;
    double vz = p->Vel_MCMF_mps.z;
    return (vz > -2.0 && vz < 2.0);
}

/** Nav solution healthy (uncertainty < 50 m) */
static inline bool MMGate_NavHealthy(const void *ctx)
{
    const LA_NAV_StatePkt_t *p = (const LA_NAV_StatePkt_t *)ctx;
    return p->PosUncert_m < 50.0;
}

/* ── EPS predicates ──────────────────────────────────────── */

/** Battery SOC above critical minimum (15%) */
static inline bool MMGate_BattAboveCritical(const void *ctx)
{
    const LA_EPS_StatusPkt_t *p = (const LA_EPS_StatusPkt_t *)ctx;
    return p->BattSOC_pct > 15.0;
}

/* ── DOCK predicates ─────────────────────────────────────── */

/** Dock collar retracted (safe to fly) */
static inline bool MMGate_DockRetracted(const void *ctx)
{
    const LA_DOCK_StatusPkt_t *p = (const LA_DOCK_StatusPkt_t *)ctx;
    return p->CollarState == 0;  /* RETRACTED */
}

#endif /* LA_MM_GATES_H */
