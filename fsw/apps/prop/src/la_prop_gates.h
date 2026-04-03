/**
 * @file la_prop_gates.h
 * @brief PROP App — Internal Sequence Gate Predicates
 *
 * Enforces the engine start sequence:
 *   IDLE → ARMED → CHILL → IGNITION → RUNNING
 *
 * Predicates read from PROP's own internal state and its
 * per-app HAL telemetry cache.  No monolithic HAL dependency.
 *
 * Example of how any app can adopt the gate pattern for its
 * own internal state machine — not just MM.
 */

#ifndef LA_PROP_GATES_H
#define LA_PROP_GATES_H

#include "../../common/la_seq_gate.h"
#include "../hal/prop_hal.h"
#include <stdbool.h>

/* ── PROP sequence state IDs ─────────────────────────────── */

#define PROP_ST_IDLE      0
#define PROP_ST_ARMED     1
#define PROP_ST_CHILL     2
#define PROP_ST_IGNITION  3
#define PROP_ST_RUNNING   4
#define PROP_ST_SHUTDOWN  5

/* ── Context struct for PROP gate checks ─────────────────── */

typedef struct {
    PropHAL_EngineTlm_t  engine_tlm;    /* Latest from PropHAL     */
    PropHAL_CryoTlm_t    cryo_tlm;      /* Latest from PropHAL     */
    double                ox_mass_kg;    /* PROP's tracked mass     */
    double                fuel_mass_kg;
} PropGate_Ctx_t;

/* ── Predicates ──────────────────────────────────────────── */

/** Propellant available for start (> 100 kg combined) */
static inline bool PropGate_HasPropellant(const void *ctx)
{
    const PropGate_Ctx_t *c = (const PropGate_Ctx_t *)ctx;
    return (c->ox_mass_kg + c->fuel_mass_kg) > 100.0;
}

/** Tank pressures nominal for engine start */
static inline bool PropGate_TankPressNominal(const void *ctx)
{
    const PropGate_Ctx_t *c = (const PropGate_Ctx_t *)ctx;
    return c->cryo_tlm.ox_press_kPa > 100.0 &&
           c->cryo_tlm.fuel_press_kPa > 50.0;
}

/** Chamber pressure confirms ignition (> 500 kPa) */
static inline bool PropGate_ChamberPressOK(const void *ctx)
{
    const PropGate_Ctx_t *c = (const PropGate_Ctx_t *)ctx;
    return c->engine_tlm.chamber_press_kPa > 500.0;
}

/** Chamber pressure confirms full thrust (> 2500 kPa) */
static inline bool PropGate_FullThrust(const void *ctx)
{
    const PropGate_Ctx_t *c = (const PropGate_Ctx_t *)ctx;
    return c->engine_tlm.chamber_press_kPa > 2500.0;
}

/**
 * Build PROP's internal gate table.
 */
static inline void PROP_InitGateTable(SeqGate_Table_t *table,
                                      const PropGate_Ctx_t *ctx)
{
    int idx;
    SeqGate_Init(table);

    /* IDLE → ARMED: need propellant and tank pressure */
    idx = SeqGate_AddEntry(table, PROP_ST_IDLE, PROP_ST_ARMED, 0);
    SeqGate_AddPrereq(table, idx, "HAS_PROPELLANT",
                      PropGate_HasPropellant, ctx);
    SeqGate_AddPrereq(table, idx, "TANK_PRESS_NOM",
                      PropGate_TankPressNominal, ctx);

    /* ARMED → CHILL: same checks (re-verify) */
    idx = SeqGate_AddEntry(table, PROP_ST_ARMED, PROP_ST_CHILL, 0);
    SeqGate_AddPrereq(table, idx, "HAS_PROPELLANT",
                      PropGate_HasPropellant, ctx);

    /* CHILL → IGNITION: no extra gate — timer-driven */
    /* (ungated = always passes) */

    /* IGNITION → RUNNING: chamber pressure must confirm */
    idx = SeqGate_AddEntry(table, PROP_ST_IGNITION,
                           PROP_ST_RUNNING, 10); /* 10 cycle timeout */
    SeqGate_AddPrereq(table, idx, "CHAMBER_PRESS_OK",
                      PropGate_ChamberPressOK, ctx);
}

#endif /* LA_PROP_GATES_H */
