/**
 * @file la_mm_gate_table.h
 * @brief Mission Manager — Gate Table Configuration
 *
 * Defines which prerequisites must be met BEFORE each MM phase
 * transition is committed.  Only checks pre-conditions — never
 * checks state that the transition itself will command.
 *
 * Example: PREFLIGHT → POWERED_ASC requires battery and dock
 * retracted, but NOT engine running (the transition commands
 * engine start).
 *
 * IMPORTANT: The `ctx` pointers point to MM's cached telemetry
 * structs (LastProp, LastLG, LastNav, etc.).  These are updated
 * every time MM reads from the SB pipe.  The gate predicates
 * read the latest values at check time — no extra messages.
 *
 * ABORT is never gated — enforced in MM_TransitionTo, not here.
 */

#ifndef LA_MM_GATE_TABLE_H
#define LA_MM_GATE_TABLE_H

#include "../../common/la_seq_gate.h"
#include "la_mm_gates.h"

/**
 * Populate the gate table with all guarded transitions.
 *
 * @param table   Gate table to populate
 * @param prop    Pointer to MM's cached PROP status packet
 * @param lg      Pointer to MM's cached LG status packet
 * @param nav     Pointer to MM's cached NAV state packet
 * @param eps     Pointer to MM's cached EPS status packet
 * @param dock    Pointer to MM's cached DOCK status packet
 */
static inline void MM_InitGateTable(
    SeqGate_Table_t             *table,
    const LA_PROP_StatusPkt_t   *prop,
    const LA_LG_StatusPkt_t     *lg,
    const LA_NAV_StatePkt_t     *nav,
    const LA_EPS_StatusPkt_t    *eps,
    const LA_DOCK_StatusPkt_t   *dock)
{
    int idx;
    SeqGate_Init(table);

    /* ════════════════════════════════════════════════════════
     * COMMON TRANSITIONS (both profiles)
     * ════════════════════════════════════════════════════════ */

    /* ── PREFLIGHT → POWERED_ASC ─────────────────────────── */
    /* Battery must be above critical, dock collar retracted.
     * Engine start is commanded BY the transition — not a prereq. */
    idx = SeqGate_AddEntry(table, LA_PHASE_PREFLIGHT,
                           LA_PHASE_POWERED_ASC, 0);
    SeqGate_AddPrereq(table, idx, "BATT_ABOVE_CRIT",
                      MMGate_BattAboveCritical, eps);
    SeqGate_AddPrereq(table, idx, "DOCK_RETRACTED",
                      MMGate_DockRetracted, dock);

    /* ── POWERED_ASC → COAST ─────────────────────────────── */
    /* Prop above reserve (engine shutdown commanded by COAST entry) */
    idx = SeqGate_AddEntry(table, LA_PHASE_POWERED_ASC,
                           LA_PHASE_COAST, 0);
    SeqGate_AddPrereq(table, idx, "PROP_ABOVE_RESERVE",
                      MMGate_PropAboveReserve, prop);

    /* ── COAST → POWERED_DES ─────────────────────────────── */
    /* Nav healthy (engine re-start commanded by POWERED_DES entry) */
    idx = SeqGate_AddEntry(table, LA_PHASE_COAST,
                           LA_PHASE_POWERED_DES, 0);
    SeqGate_AddPrereq(table, idx, "NAV_HEALTHY",
                      MMGate_NavHealthy, nav);

    /* ── POWERED_DES → HOVER ─────────────────────────────── */
    /* Velocity near zero (gear deploy commanded by HOVER entry) */
    idx = SeqGate_AddEntry(table, LA_PHASE_POWERED_DES,
                           LA_PHASE_HOVER, 0);
    SeqGate_AddPrereq(table, idx, "VEL_BELOW_THRESH",
                      MMGate_VelBelowThreshold, nav);

    /* ════════════════════════════════════════════════════════
     * SURFACE PROFILE
     * ════════════════════════════════════════════════════════ */

    /* ── HOVER → TERMINAL ────────────────────────────────── */
    /* Gear must be deployed (was commanded on HOVER entry),
     * nav solution healthy for terminal descent                */
    idx = SeqGate_AddEntry(table, LA_PHASE_HOVER,
                           LA_PHASE_TERMINAL, 0);
    SeqGate_AddPrereq(table, idx, "GEAR_DEPLOYED",
                      MMGate_GearDeployed, lg);
    SeqGate_AddPrereq(table, idx, "NAV_HEALTHY",
                      MMGate_NavHealthy, nav);

    /* ── TERMINAL → LANDED ───────────────────────────────── */
    /* Touchdown on all 4 legs (engine shutdown commanded by
     * LANDED entry)                                            */
    idx = SeqGate_AddEntry(table, LA_PHASE_TERMINAL,
                           LA_PHASE_LANDED, 0);
    SeqGate_AddPrereq(table, idx, "TOUCHDOWN_ALL",
                      MMGate_TouchdownAll, lg);

    /* ════════════════════════════════════════════════════════
     * ORBITAL DOCKING PROFILE
     * ════════════════════════════════════════════════════════ */

    /* ── HOVER → RENDEZVOUS ──────────────────────────────── */
    /* Gear must be stowed for orbital approach, nav healthy    */
    idx = SeqGate_AddEntry(table, LA_PHASE_HOVER,
                           LA_PHASE_RENDEZVOUS, 0);
    SeqGate_AddPrereq(table, idx, "GEAR_STOWED",
                      MMGate_GearStowed, lg);
    SeqGate_AddPrereq(table, idx, "NAV_HEALTHY",
                      MMGate_NavHealthy, nav);

    /* ── RENDEZVOUS → DOCKING ────────────────────────────── */
    /* Nav healthy for final approach (PROP shutdown + DOCK mate
     * commanded by DOCKING entry)                              */
    idx = SeqGate_AddEntry(table, LA_PHASE_RENDEZVOUS,
                           LA_PHASE_DOCKING, 0);
    SeqGate_AddPrereq(table, idx, "NAV_HEALTHY",
                      MMGate_NavHealthy, nav);

    /* ── DOCKING → DOCKED ────────────────────────────────── */
    /* No gate — transition is triggered by hard dock + seal
     * confirmation in MM_UpdateFSM.  Those checks ARE the gate. */

    /* ── UNDOCKING → POWERED_ASC ─────────────────────────── */
    /* Dock collar must be retracted before ignition, battery OK */
    idx = SeqGate_AddEntry(table, LA_PHASE_UNDOCKING,
                           LA_PHASE_POWERED_ASC, 0);
    SeqGate_AddPrereq(table, idx, "DOCK_RETRACTED",
                      MMGate_DockRetracted, dock);
    SeqGate_AddPrereq(table, idx, "BATT_ABOVE_CRIT",
                      MMGate_BattAboveCritical, eps);

    /* ── Note: ABORT is always allowed from any phase ────────
     * No gate entry = ungated = always passes.  Enforced in
     * MM_TransitionTo which skips gate checks for ABORT.       */
}

#endif /* LA_MM_GATE_TABLE_H */
