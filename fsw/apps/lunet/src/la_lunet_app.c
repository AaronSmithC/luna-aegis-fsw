/**
 * @file la_lunet_app.c
 * @brief Luna-Aegis LUNET Interface Application
 *
 * Interfaces with LUNET surface infrastructure nodes for:
 * - Beacon reception (range/bearing for nav)
 * - Post-landing refuel/recharge sequencing
 * - Field diagnostics via rover interface or node terminal
 *
 * Refuel Sequence (Rev D — ECL-2026-005):
 *   On LANDED at a refuel-capable destination, LUNET drives a
 *   multi-step sequence: node handshake → cartridge mate →
 *   LOX/LH2 transfer (via PROP REFUEL cmd) → battery recharge
 *   (via EPS SOC monitoring).  Sequence state published in
 *   BeaconPkt.RefuelReady field.
 *
 * Subscribes: SCH Wakeup (1 Hz), COMM_LinkPkt, LUNET_CMD,
 *             MM_PhasePkt, PROP_StatusPkt, EPS_StatusPkt
 * Publishes:  LUNET_BeaconPkt (1 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include "../../common/la_destinations.h"
#include <string.h>
#include <math.h>

#define LUNET_PIPE_DEPTH  32
#define LUNET_PIPE_NAME   "LUNET_PIPE"

/* Default beacon: Shackleton Base (node 1) */
#define LUNET_DEFAULT_NODE_ID  1

/* Refuel sequence states — published in RefuelReady field */
#define REFUEL_IDLE            0
#define REFUEL_HANDSHAKE       1
#define REFUEL_CARTRIDGE_MATE  2
#define REFUEL_PROP_TRANSFER   3
#define REFUEL_RECHARGE        4
#define REFUEL_COMPLETE        5

/* Refuel sequence timing (seconds) */
#define REFUEL_HANDSHAKE_TIME     3
#define REFUEL_MATE_TIME          5

/* Propellant full thresholds */
#define REFUEL_PROP_FULL_KG    2745.0  /* Within 5 kg of 2750 kg capacity */
#define REFUEL_BATT_FULL_PCT     95.0

/* PROP refuel command FC */
#define PROP_REFUEL_CC         6

/* LUNET command function codes */
#define LUNET_NOOP_CC          0
#define LUNET_RESET_CC         1
#define LUNET_REFUEL_START_CC  2   /* Manual refuel trigger */
#define LUNET_AUTO_REFUEL_CC   3   /* Toggle auto-refuel on/off */

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    LA_COMM_LinkPkt_t    LastComm;
    LA_MM_PhasePkt_t     LastPhase;
    LA_PROP_StatusPkt_t  LastProp;
    LA_EPS_StatusPkt_t   LastEPS;
    LA_NAV_StatePkt_t    LastNav;
    bool                 CommValid;
    bool                 PhaseValid;
    bool                 PropValid;
    bool                 EPSValid;
    bool                 NavValid;
    uint8_t              ActiveNodeId;  /* Current LUNET node to simulate */

    /* Refuel sequence */
    uint8_t    RefuelState;
    uint32_t   RefuelTimer_s;
    bool       RefuelCapable;       /* Destination has refuel */
    uint8_t    DestNodeId;          /* LUNET node at destination */
    bool       AutoRefuel;          /* true = start on landing, false = on demand */

    LA_LUNET_BeaconPkt_t BeaconPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} LUNET_AppData_t;

static LUNET_AppData_t LUNET;

/* ── Forward Declarations ────────────────────────────────── */

static void LUNET_Process(void);
static void LUNET_UpdateRefuel(void);
static void LUNET_SendSubsystemCmd(uint16_t mid_raw, uint16_t fc);

/* ── Initialization ──────────────────────────────────────── */

static CFE_Status_t LUNET_Init(void)
{
    memset(&LUNET, 0, sizeof(LUNET));
    LUNET.RunStatus    = CFE_ES_RunStatus_APP_RUN;
    LUNET.RefuelState  = REFUEL_IDLE;
    LUNET.ActiveNodeId = LUNET_DEFAULT_NODE_ID;
    LUNET.AutoRefuel   = true;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&LUNET.CmdPipe, LUNET_PIPE_DEPTH, LUNET_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LUNET_CMD_MID),         LUNET.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_COMM_LINK_TLM_MID),     LUNET.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),      LUNET.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_PROP_STATUS_TLM_MID),   LUNET.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_EPS_STATUS_TLM_MID),    LUNET.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_NAV_STATE_TLM_MID),    LUNET.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x0C)),  LUNET.CmdPipe);

    CFE_MSG_Init(&LUNET.BeaconPkt.TlmHdr.Msg,
                 CFE_SB_ValueToMsgId(LA_LUNET_BEACON_TLM_MID),
                 sizeof(LA_LUNET_BeaconPkt_t));

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "LUNET: Interface initialized — default node %u",
                      LUNET_DEFAULT_NODE_ID);
    return CFE_SUCCESS;
}

/* ── Beacon Simulation ───────────────────────────────────── */

static void LUNET_Process(void)
{
    /* Update active node from NAV destination */
    if (LUNET.NavValid && LUNET.LastNav.DestId != 0xFF) {
        const LA_Destination_t *dest = LA_Dest_Lookup(LUNET.LastNav.DestId);
        if (dest && dest->lunet_node_id != 0) {
            LUNET.ActiveNodeId = dest->lunet_node_id;
        }
    }

    /* Look up the active node's destination to get coordinates */
    const LA_Destination_t *node_dest = NULL;
    for (int i = 0; i < LA_DEST_COUNT; i++) {
        if (LA_DEST_TABLE[i].lunet_node_id == LUNET.ActiveNodeId) {
            node_dest = &LA_DEST_TABLE[i];
            break;
        }
    }

    /* Simulate beacon position from destination coordinates.
     * Convert lat/lon to a simplified MCMF position for range calc. */
    double bx = 0.0, by = 0.0, bz = 0.0;
    if (node_dest) {
        double DEG2RAD = 3.14159265358979323846 / 180.0;
        double r = LA_MOON_RADIUS_M + node_dest->alt_m;
        double lat = node_dest->lat_deg * DEG2RAD;
        double lon = node_dest->lon_deg * DEG2RAD;
        bx = r * cos(lat) * cos(lon);
        by = r * cos(lat) * sin(lon);
        bz = r * sin(lat);
    }

    LUNET.BeaconPkt.BeaconPos_MCMF_m.x = bx;
    LUNET.BeaconPkt.BeaconPos_MCMF_m.y = by;
    LUNET.BeaconPkt.BeaconPos_MCMF_m.z = bz;

    /* Range from vehicle (NAV position) to beacon */
    double dx = bx - (LUNET.NavValid ? LUNET.LastNav.Pos_MCMF_m.x : 0.0);
    double dy = by - (LUNET.NavValid ? LUNET.LastNav.Pos_MCMF_m.y : 0.0);
    double dz = bz - (LUNET.NavValid ? LUNET.LastNav.Pos_MCMF_m.z : 0.0);
    double range = sqrt(dx*dx + dy*dy + dz*dz);

    LUNET.BeaconPkt.BeaconRange_m     = range;
    LUNET.BeaconPkt.BeaconBearing_rad = atan2(dy, dx);
    LUNET.BeaconPkt.BeaconID          = LUNET.ActiveNodeId;

    /* Signal quality depends on comms link */
    uint8_t sq = 200;
    if (LUNET.CommValid && LUNET.LastComm.LinkState == 0) {
        sq = 50;
    }
    LUNET.BeaconPkt.SignalQuality = sq;

    /* Refuel state published in RefuelReady field */
    LUNET.BeaconPkt.RefuelReady = LUNET.RefuelState;

    /* Run refuel sequence */
    LUNET_UpdateRefuel();

    CFE_SB_TransmitMsg(&LUNET.BeaconPkt.TlmHdr.Msg, true);
}

/* ── Refuel / Recharge Sequence ──────────────────────────── */

static void LUNET_UpdateRefuel(void)
{
    switch (LUNET.RefuelState) {

    case REFUEL_IDLE:
        /* Check if we're at a refuel-capable node (surface or orbital) */
        if (LUNET.PhaseValid &&
            (LUNET.LastPhase.CurrentPhase == LA_PHASE_LANDED ||
             LUNET.LastPhase.CurrentPhase == LA_PHASE_DOCKED))
        {
            for (int i = 0; i < LA_DEST_COUNT; i++) {
                if (LA_DEST_TABLE[i].lunet_node_id == LUNET.ActiveNodeId &&
                    LA_DEST_TABLE[i].has_refuel)
                {
                    LUNET.RefuelCapable = true;
                    LUNET.DestNodeId    = LA_DEST_TABLE[i].lunet_node_id;
                    break;
                }
            }
            /* Auto-refuel: start immediately; manual: wait for command */
            if (LUNET.RefuelCapable && LUNET.AutoRefuel) {
                LUNET.RefuelState   = REFUEL_HANDSHAKE;
                LUNET.RefuelTimer_s = 0;
                CFE_EVS_SendEvent(20, CFE_EVS_EventType_INFORMATION,
                    "LUNET: Auto-refuel START — node %u",
                    LUNET.DestNodeId);
            }
        }
        break;

    case REFUEL_HANDSHAKE:
        /* Simulated node handshake */
        LUNET.RefuelTimer_s++;
        if (LUNET.RefuelTimer_s >= REFUEL_HANDSHAKE_TIME) {
            LUNET.RefuelState   = REFUEL_CARTRIDGE_MATE;
            LUNET.RefuelTimer_s = 0;
            CFE_EVS_SendEvent(21, CFE_EVS_EventType_INFORMATION,
                "LUNET: Node handshake complete — mating cartridge");
        }
        break;

    case REFUEL_CARTRIDGE_MATE:
        /* Simulated ISRU cartridge mate */
        LUNET.RefuelTimer_s++;
        if (LUNET.RefuelTimer_s >= REFUEL_MATE_TIME) {
            LUNET.RefuelState   = REFUEL_PROP_TRANSFER;
            LUNET.RefuelTimer_s = 0;
            /* Command PROP to begin accepting propellant */
            LUNET_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_REFUEL_CC);
            CFE_EVS_SendEvent(22, CFE_EVS_EventType_INFORMATION,
                "LUNET: Cartridge mated — LOX/LH2 transfer starting");
        }
        break;

    case REFUEL_PROP_TRANSFER:
        /* Monitor PROP status until tanks full */
        if (LUNET.PropValid) {
            double total = LUNET.LastProp.OxMass_kg
                         + LUNET.LastProp.FuelMass_kg;
            if (total >= REFUEL_PROP_FULL_KG) {
                LUNET.RefuelState   = REFUEL_RECHARGE;
                LUNET.RefuelTimer_s = 0;
                CFE_EVS_SendEvent(23, CFE_EVS_EventType_INFORMATION,
                    "LUNET: Propellant transfer complete (%.0f kg) — "
                    "battery recharge starting", total);
            }
        }
        LUNET.RefuelTimer_s++;
        /* Timeout: if transfer takes > 120s, something is wrong */
        if (LUNET.RefuelTimer_s > 120) {
            CFE_EVS_SendEvent(30, CFE_EVS_EventType_ERROR,
                "LUNET: Propellant transfer timeout — aborting refuel");
            LUNET.RefuelState = REFUEL_IDLE;
        }
        break;

    case REFUEL_RECHARGE:
        /* Monitor EPS until battery charged */
        if (LUNET.EPSValid && LUNET.LastEPS.BattSOC_pct >= REFUEL_BATT_FULL_PCT) {
            LUNET.RefuelState   = REFUEL_COMPLETE;
            LUNET.RefuelTimer_s = 0;
            CFE_EVS_SendEvent(24, CFE_EVS_EventType_INFORMATION,
                "LUNET: Battery recharge complete (%.1f%%) — "
                "vehicle READY", LUNET.LastEPS.BattSOC_pct);
        }
        LUNET.RefuelTimer_s++;
        if (LUNET.RefuelTimer_s > 120) {
            /* Battery not charging — complete anyway, not critical */
            LUNET.RefuelState   = REFUEL_COMPLETE;
            LUNET.RefuelTimer_s = 0;
            CFE_EVS_SendEvent(31, CFE_EVS_EventType_INFORMATION,
                "LUNET: Recharge timeout — completing with SOC %.1f%%",
                LUNET.EPSValid ? LUNET.LastEPS.BattSOC_pct : 0.0);
        }
        break;

    case REFUEL_COMPLETE:
        /* Reset when vehicle departs (leaves LANDED or DOCKED) */
        if (LUNET.PhaseValid &&
            LUNET.LastPhase.CurrentPhase != LA_PHASE_LANDED &&
            LUNET.LastPhase.CurrentPhase != LA_PHASE_DOCKED)
        {
            LUNET.RefuelState   = REFUEL_IDLE;
            LUNET.RefuelCapable = false;
            CFE_EVS_SendEvent(25, CFE_EVS_EventType_INFORMATION,
                "LUNET: Refuel sequence reset — vehicle departing");
        }
        break;
    }
}

/* ── Subsystem Command Helper ────────────────────────────── */

static void LUNET_SendSubsystemCmd(uint16_t mid_raw, uint16_t fc)
{
    CFE_MSG_CommandHeader_t cmd;
    CFE_MSG_Init(&cmd.Msg, CFE_SB_ValueToMsgId(mid_raw), sizeof(cmd));
    CFE_MSG_SetFcnCode(&cmd.Msg, fc);
    CFE_SB_TransmitMsg(&cmd.Msg, true);
}

/* ── Main Loop ───────────────────────────────────────────── */

void LUNET_AppMain(void)
{
    CFE_Status_t     s;
    CFE_SB_Buffer_t *SBBufPtr;
    CFE_SB_MsgId_t   MsgId;

    if (LUNET_Init() != CFE_SUCCESS) {
        LUNET.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&LUNET.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, LUNET.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;

        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

        switch (CFE_SB_MsgIdToValue(MsgId)) {

        case LA_SCH_WAKEUP_MID(0x0C):
            LUNET_Process();
            LUNET.CycleCount++;
            break;

        case LA_COMM_LINK_TLM_MID:
            memcpy(&LUNET.LastComm, SBBufPtr, sizeof(LA_COMM_LinkPkt_t));
            LUNET.CommValid = true;
            break;

        case LA_MM_PHASE_TLM_MID:
            memcpy(&LUNET.LastPhase, SBBufPtr, sizeof(LA_MM_PhasePkt_t));
            LUNET.PhaseValid = true;
            break;

        case LA_PROP_STATUS_TLM_MID:
            memcpy(&LUNET.LastProp, SBBufPtr, sizeof(LA_PROP_StatusPkt_t));
            LUNET.PropValid = true;
            break;

        case LA_EPS_STATUS_TLM_MID:
            memcpy(&LUNET.LastEPS, SBBufPtr, sizeof(LA_EPS_StatusPkt_t));
            LUNET.EPSValid = true;
            break;

        case LA_NAV_STATE_TLM_MID:
            memcpy(&LUNET.LastNav, SBBufPtr, sizeof(LA_NAV_StatePkt_t));
            LUNET.NavValid = true;
            break;

        case LA_LUNET_CMD_MID: {
            CFE_MSG_FcnCode_t fc;
            CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &fc);
            switch (fc) {
            case LUNET_NOOP_CC:
                LUNET.CmdCount++;
                break;
            case LUNET_RESET_CC:
                LUNET.CmdCount = 0; LUNET.ErrCount = 0;
                break;
            case LUNET_REFUEL_START_CC:
                if (LUNET.RefuelState == REFUEL_IDLE && LUNET.RefuelCapable) {
                    LUNET.RefuelState   = REFUEL_HANDSHAKE;
                    LUNET.RefuelTimer_s = 0;
                    LUNET.CmdCount++;
                    CFE_EVS_SendEvent(20, CFE_EVS_EventType_INFORMATION,
                        "LUNET: Manual refuel START — node %u", LUNET.DestNodeId);
                } else {
                    LUNET.ErrCount++;
                    CFE_EVS_SendEvent(32, CFE_EVS_EventType_ERROR,
                        "LUNET: REFUEL rejected — %s",
                        !LUNET.RefuelCapable ? "no refuel node" : "already in progress");
                }
                break;
            case LUNET_AUTO_REFUEL_CC:
                LUNET.AutoRefuel = !LUNET.AutoRefuel;
                LUNET.CmdCount++;
                CFE_EVS_SendEvent(33, CFE_EVS_EventType_INFORMATION,
                    "LUNET: Auto-refuel %s", LUNET.AutoRefuel ? "ON" : "OFF");
                break;
            default:
                LUNET.ErrCount++;
                break;
            }
            break;
        }

        default:
            break;
        }
    }

    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
