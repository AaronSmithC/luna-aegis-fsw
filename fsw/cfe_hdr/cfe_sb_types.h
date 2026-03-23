/**
 * @file cfe_sb_types.h
 * @brief cFE Software Bus type definitions (stub for luna-aegis-fsw)
 *
 * Minimal CCSDS / cFE SB type definitions for compilation outside
 * the full cFE tree.  These mirror the cFE 7.x API surface used
 * by Luna-Aegis mission apps.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#ifndef CFE_SB_TYPES_H
#define CFE_SB_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ── CCSDS Primary Header ─────────────────────────────────── */

typedef struct {
    uint16_t StreamId;    /* APID + packet type + sec hdr flag  */
    uint16_t Sequence;    /* Sequence flags + count             */
    uint16_t Length;      /* Packet data length - 1             */
} CCSDS_PrimaryHdr_t;

/* ── CCSDS Command Secondary Header ──────────────────────── */

typedef struct {
    uint8_t FunctionCode;
    uint8_t Checksum;
} CCSDS_CmdSecHdr_t;

/* ── CCSDS Telemetry Secondary Header ────────────────────── */

typedef struct {
    uint32_t Seconds;
    uint16_t Subseconds;
} CCSDS_TlmSecHdr_t;

/* ── cFE Software Bus Message Types ──────────────────────── */

typedef struct {
    CCSDS_PrimaryHdr_t  Hdr;
} CFE_MSG_Message_t;

typedef struct {
    CFE_MSG_Message_t   Msg;
    CCSDS_CmdSecHdr_t   Sec;
} CFE_MSG_CommandHeader_t;

typedef struct {
    CCSDS_PrimaryHdr_t  Hdr;
    CCSDS_TlmSecHdr_t   Sec;
} CCSDS_TelemetrySecondaryHeader_t;

typedef struct {
    CFE_MSG_Message_t                Msg;
    CCSDS_TelemetrySecondaryHeader_t Sec;
} CFE_MSG_TelemetryHeader_t;

/* ── Software Bus Identifiers ────────────────────────────── */

typedef uint32_t CFE_SB_MsgId_t;
typedef uint32_t CFE_SB_PipeId_t;

/* MsgId conversion macros (real cFE may use opaque types) */
#define CFE_SB_ValueToMsgId(v)    ((CFE_SB_MsgId_t)(v))
#define CFE_SB_MsgIdToValue(id)   ((uint32_t)(id))

/* ── Function Code Type ──────────────────────────────────── */

typedef uint16_t CFE_MSG_FcnCode_t;

/* ── Status Codes ────────────────────────────────────────── */

typedef int32_t CFE_Status_t;

#define CFE_SUCCESS           ((CFE_Status_t) 0)
#define CFE_STATUS_WRONG_MSG  ((CFE_Status_t)-1)
#define CFE_SB_TIMEOUT        ((CFE_Status_t)-2)

/* ── SB API Stubs ────────────────────────────────────────── */

#define CFE_SB_PEND_FOREVER   (-1)
#define CFE_SB_POLL           (0)

/* Buffer type — in real cFE this is a struct with .Msg member */
typedef struct {
    CFE_MSG_Message_t Msg;
} CFE_SB_Buffer_t;

/* MSG pointer macro */
#define CFE_MSG_PTR(header)   (&(header).Msg)

#endif /* CFE_SB_TYPES_H */
