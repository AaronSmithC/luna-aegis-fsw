/**
 * @file la_seq_gate.h
 * @brief Luna-Aegis Sequence Gate — Phase Transition Enforcement
 *
 * Provides a reusable, table-driven mechanism to enforce
 * prerequisite conditions before allowing state transitions.
 * Gates read ONLY from app-level telemetry packets already
 * cached in memory — never from HAL.
 *
 * ARCHITECTURE BOUNDARY:
 *   HAL → Apps → [publish TLM on SB] → Gate reads TLM structs
 *   Gate is purely mission-layer.  Swapping HAL sim for HW
 *   changes nothing in this file.
 *
 * USAGE:
 *   1. Define a SeqGate_Table_t with entries for each guarded
 *      transition.
 *   2. Before committing a transition, call SeqGate_Check().
 *   3. If it returns SEQGATE_PASS, proceed.  Otherwise, the
 *      gate tells you which prerequisite failed.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#ifndef LA_SEQ_GATE_H
#define LA_SEQ_GATE_H

#include <stdint.h>
#include <stdbool.h>

/* ── Result Codes ────────────────────────────────────────── */

typedef enum {
    SEQGATE_PASS          = 0,    /* All prerequisites met           */
    SEQGATE_FAIL_PREREQ   = 1,    /* A named prerequisite failed     */
    SEQGATE_FAIL_TIMEOUT  = 2,    /* Prerequisite not met in time    */
    SEQGATE_NO_ENTRY      = 3,    /* No gate defined for this combo  */
} SeqGate_Result_t;

/* ── Prerequisite Check Function Pointer ─────────────────── */

/**
 * User-supplied predicate.  Returns true if the prerequisite
 * is currently satisfied.  The opaque `ctx` pointer lets the
 * caller pass in cached telemetry structs without the gate
 * knowing their type.
 *
 * Example:
 *   bool check_engine_running(const void *ctx) {
 *       const LA_PROP_StatusPkt_t *p = ctx;
 *       return p->EngineState == 3;  // HAL_ENGINE_RUNNING
 *   }
 */
typedef bool (*SeqGate_Predicate_t)(const void *ctx);

/* ── Gate Entry ──────────────────────────────────────────── */

#define SEQGATE_MAX_PREREQS  4    /* Max prerequisites per gate      */
#define SEQGATE_NAME_LEN    32    /* Max name length for diagnostics  */

typedef struct {
    char                  name[SEQGATE_NAME_LEN];
    SeqGate_Predicate_t   check;
    const void           *ctx;    /* Points to cached TLM struct     */
} SeqGate_Prereq_t;

typedef struct {
    uint8_t             from_state;       /* Source state ID          */
    uint8_t             to_state;         /* Target state ID          */
    SeqGate_Prereq_t   prereqs[SEQGATE_MAX_PREREQS];
    uint8_t             prereq_count;
    uint16_t            timeout_cycles;   /* 0 = no timeout           */
} SeqGate_Entry_t;

/* ── Gate Table ──────────────────────────────────────────── */

#define SEQGATE_MAX_ENTRIES  16

typedef struct {
    SeqGate_Entry_t  entries[SEQGATE_MAX_ENTRIES];
    uint8_t          entry_count;
} SeqGate_Table_t;

/* ── Diagnostic Output ───────────────────────────────────── */

typedef struct {
    SeqGate_Result_t  result;
    uint8_t           failed_prereq_idx;  /* Which prereq failed     */
    char              failed_name[SEQGATE_NAME_LEN];
} SeqGate_Diag_t;

/* ══════════════════════════════════════════════════════════
 * API
 * ══════════════════════════════════════════════════════════ */

/**
 * Initialize a gate table to empty.
 */
static inline void SeqGate_Init(SeqGate_Table_t *table)
{
    table->entry_count = 0;
}

/**
 * Add a gate entry.  Returns index or -1 if table full.
 */
static inline int SeqGate_AddEntry(SeqGate_Table_t *table,
                                   uint8_t from, uint8_t to,
                                   uint16_t timeout_cycles)
{
    if (table->entry_count >= SEQGATE_MAX_ENTRIES) return -1;
    int idx = table->entry_count++;
    table->entries[idx].from_state    = from;
    table->entries[idx].to_state      = to;
    table->entries[idx].prereq_count  = 0;
    table->entries[idx].timeout_cycles = timeout_cycles;
    return idx;
}

/**
 * Add a prerequisite to an existing gate entry.
 * Returns 0 on success, -1 if entry full.
 */
static inline int SeqGate_AddPrereq(SeqGate_Table_t *table,
                                    int entry_idx,
                                    const char *name,
                                    SeqGate_Predicate_t check,
                                    const void *ctx)
{
    if (entry_idx < 0 || entry_idx >= table->entry_count) return -1;
    SeqGate_Entry_t *e = &table->entries[entry_idx];
    if (e->prereq_count >= SEQGATE_MAX_PREREQS) return -1;

    SeqGate_Prereq_t *p = &e->prereqs[e->prereq_count++];
    /* Safe string copy */
    int i;
    for (i = 0; i < SEQGATE_NAME_LEN - 1 && name[i]; i++)
        p->name[i] = name[i];
    p->name[i] = '\0';

    p->check = check;
    p->ctx   = ctx;
    return 0;
}

/**
 * Check whether a transition from `from_state` to `to_state`
 * is permitted.  Evaluates all prerequisites for the matching
 * gate entry.
 *
 * If no entry matches the (from, to) pair, returns SEQGATE_PASS
 * by default (ungated transitions are allowed).  Set `strict`
 * to true to return SEQGATE_NO_ENTRY instead.
 */
static inline SeqGate_Result_t SeqGate_Check(
    const SeqGate_Table_t *table,
    uint8_t from_state,
    uint8_t to_state,
    bool strict,
    SeqGate_Diag_t *diag)
{
    /* Find matching entry */
    const SeqGate_Entry_t *entry = NULL;
    for (int i = 0; i < table->entry_count; i++) {
        if (table->entries[i].from_state == from_state &&
            table->entries[i].to_state   == to_state) {
            entry = &table->entries[i];
            break;
        }
    }

    if (!entry) {
        if (diag) {
            diag->result = strict ? SEQGATE_NO_ENTRY : SEQGATE_PASS;
            diag->failed_prereq_idx = 0;
            diag->failed_name[0] = '\0';
        }
        return strict ? SEQGATE_NO_ENTRY : SEQGATE_PASS;
    }

    /* Evaluate all prerequisites */
    for (int i = 0; i < entry->prereq_count; i++) {
        const SeqGate_Prereq_t *p = &entry->prereqs[i];
        if (!p->check(p->ctx)) {
            if (diag) {
                diag->result = SEQGATE_FAIL_PREREQ;
                diag->failed_prereq_idx = (uint8_t)i;
                int j;
                for (j = 0; j < SEQGATE_NAME_LEN - 1 && p->name[j]; j++)
                    diag->failed_name[j] = p->name[j];
                diag->failed_name[j] = '\0';
            }
            return SEQGATE_FAIL_PREREQ;
        }
    }

    if (diag) {
        diag->result = SEQGATE_PASS;
        diag->failed_prereq_idx = 0;
        diag->failed_name[0] = '\0';
    }
    return SEQGATE_PASS;
}

/**
 * Convenience: Check and report via EVS-style string.
 * Returns true if transition is allowed.
 */
static inline bool SeqGate_Permit(
    const SeqGate_Table_t *table,
    uint8_t from_state,
    uint8_t to_state,
    char *reason_buf,
    int reason_buf_len)
{
    SeqGate_Diag_t diag;
    SeqGate_Result_t r = SeqGate_Check(table, from_state, to_state,
                                       false, &diag);
    if (r == SEQGATE_PASS) {
        if (reason_buf && reason_buf_len > 0) reason_buf[0] = '\0';
        return true;
    }

    if (reason_buf && reason_buf_len > 0) {
        /* Build diagnostic string */
        int pos = 0;
        const char *prefix = "GATE BLOCKED: ";
        for (int i = 0; prefix[i] && pos < reason_buf_len - 1; i++)
            reason_buf[pos++] = prefix[i];
        for (int i = 0; diag.failed_name[i] && pos < reason_buf_len - 1; i++)
            reason_buf[pos++] = diag.failed_name[i];
        reason_buf[pos] = '\0';
    }
    return false;
}

#endif /* LA_SEQ_GATE_H */
