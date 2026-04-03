/**
 * @file la_mm_app.c
 * @brief Luna-Aegis Mission Manager (MM) Application — Rev C
 *
 * Flight phase finite state machine.  Two mission profiles:
 *
 *   SURFACE:  PREFLIGHT → POWERED_ASC → COAST → POWERED_DES →
 *             HOVER → TERMINAL → LANDED
 *
 *   ORBITAL:  PREFLIGHT → POWERED_ASC → COAST → POWERED_DES →
 *             HOVER → RENDEZVOUS → DOCKING → DOCKED
 *
 *   DEPARTURE (from DOCKED):
 *             DOCKED → UNDOCKING → POWERED_ASC → ...
 *
 * ABORT reachable from any active phase (both profiles, ungated).
 *
 * Subscribes: NAV_StatePkt, PROP_StatusPkt, LG_StatusPkt,
 *             DOCK_StatusPkt, EPS_StatusPkt, HS_AlertPkt, MM_CMD
 * Publishes:  MM_PhasePkt (1 Hz), MM_AbortCmd (async)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include "la_mm_gate_table.h"
#include <string.h>
#include <math.h>

/* ── App Data ────────────────────────────────────────────── */

#define MM_PIPE_DEPTH     48
#define MM_PIPE_NAME      "MM_PIPE"

#define MM_PROP_ABORT_THRESH_KG    50.0

/* Command function codes */
#define MM_NOOP_CC          0
#define MM_RESET_CC         1
#define MM_START_MISSION_CC 2
#define MM_ABORT_CC         3
#define MM_MANUAL_CC        4
#define MM_AUTO_CC          5
#define MM_SET_DOCK_CC      6
#define MM_SET_SURFACE_CC   7

/* Mission types */
#define MM_MISSION_SURFACE  0
#define MM_MISSION_DOCK     1

/* Subsystem command function codes */
#define PROP_ARM_CC       2
#define PROP_START_CC     4
#define PROP_SHUTDOWN_CC  5
#define LG_DEPLOY_CC      2
#define LG_RETRACT_CC     3
#define DOCK_MATE_CC      2
#define DOCK_DEMATE_CC    3

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    LA_FlightPhase_t CurrentPhase;
    LA_FlightPhase_t PreviousPhase;
    uint8_t          MissionMode;
    uint8_t          MissionType;
    uint8_t          AbortReason;
    double           MET_s;
    uint32_t         PhaseTimer_s;
    double           BurnTime_s;

    LA_NAV_StatePkt_t   LastNav;
    LA_PROP_StatusPkt_t LastProp;
    LA_LG_StatusPkt_t   LastLG;
    LA_DOCK_StatusPkt_t LastDock;
    LA_EPS_StatusPkt_t  LastEPS;
    LA_HS_AlertPkt_t    LastHS;
    bool                NavValid;
    bool                PropValid;
    bool                LGValid;
    bool                DockValid;
    bool                EPSValid;

    LA_MM_PhasePkt_t   PhasePkt;
    SeqGate_Table_t    GateTable;

    uint16_t         CmdCount;
    uint16_t         ErrCount;
} MM_AppData_t;

static MM_AppData_t MM;

static void MM_ProcessCommand(const CFE_MSG_Message_t *MsgPtr);
static void MM_UpdateFSM(void);
static void MM_SendPhasePkt(void);
static void MM_TransitionTo(LA_FlightPhase_t new_phase);
static void MM_SendSubsystemCmd(uint16_t mid_raw, uint16_t fc);

static const char *phase_names[] = {
    "PREFLIGHT", "POWERED_ASC", "COAST", "POWERED_DES",
    "HOVER", "TERMINAL", "LANDED", "ABORT", "SAFED",
    "RENDEZVOUS", "DOCKING", "DOCKED", "UNDOCKING"
};

/* ── Initialization ──────────────────────────────────────── */

static CFE_Status_t MM_Init(void)
{
    memset(&MM, 0, sizeof(MM));
    MM.RunStatus    = CFE_ES_RunStatus_APP_RUN;
    MM.CurrentPhase = LA_PHASE_PREFLIGHT;
    MM.MissionMode  = 0;
    MM.MissionType  = MM_MISSION_SURFACE;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&MM.CmdPipe, MM_PIPE_DEPTH, MM_PIPE_NAME);

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_CMD_MID),           MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_ABORT_CMD_MID),     MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_NAV_STATE_TLM_MID),    MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_PROP_STATUS_TLM_MID),  MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LG_STATUS_TLM_MID),    MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_DOCK_STATUS_TLM_MID),  MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_EPS_STATUS_TLM_MID),   MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_HS_ALERT_TLM_MID),     MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x08)), MM.CmdPipe);

    CFE_MSG_Init(&MM.PhasePkt.TlmHdr.Msg,
                 CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),
                 sizeof(LA_MM_PhasePkt_t));

    /* Initialize sequence gate table with transition prerequisites */
    MM_InitGateTable(&MM.GateTable,
                     &MM.LastProp, &MM.LastLG, &MM.LastNav,
                     &MM.LastEPS, &MM.LastDock);

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "MM: Mission Manager initialized — phase PREFLIGHT, %u gates",
                      MM.GateTable.entry_count);
    return CFE_SUCCESS;
}

/* ── Main Loop ───────────────────────────────────────────── */

void MM_AppMain(void)
{
    CFE_Status_t      status;
    CFE_SB_Buffer_t  *SBBufPtr;
    CFE_SB_MsgId_t    MsgId;

    if (MM_Init() != CFE_SUCCESS) {
        MM.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&MM.RunStatus) == true) {
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, MM.CmdPipe,
                                      CFE_SB_PEND_FOREVER);
        if (status != CFE_SUCCESS) continue;

        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

        switch (CFE_SB_MsgIdToValue(MsgId)) {

        case LA_SCH_WAKEUP_MID(0x08):
            MM.MET_s += 1.0;
            MM.PhaseTimer_s++;
            MM_UpdateFSM();
            MM_SendPhasePkt();
            break;

        case LA_NAV_STATE_TLM_MID:
            memcpy(&MM.LastNav, SBBufPtr, sizeof(LA_NAV_StatePkt_t));
            MM.NavValid = true;
            break;

        case LA_PROP_STATUS_TLM_MID:
            memcpy(&MM.LastProp, SBBufPtr, sizeof(LA_PROP_StatusPkt_t));
            MM.PropValid = true;
            break;

        case LA_LG_STATUS_TLM_MID:
            memcpy(&MM.LastLG, SBBufPtr, sizeof(LA_LG_StatusPkt_t));
            MM.LGValid = true;
            break;

        case LA_DOCK_STATUS_TLM_MID:
            memcpy(&MM.LastDock, SBBufPtr, sizeof(LA_DOCK_StatusPkt_t));
            MM.DockValid = true;
            break;

        case LA_EPS_STATUS_TLM_MID:
            memcpy(&MM.LastEPS, SBBufPtr, sizeof(LA_EPS_StatusPkt_t));
            MM.EPSValid = true;
            break;

        case LA_HS_ALERT_TLM_MID:
            memcpy(&MM.LastHS, SBBufPtr, sizeof(LA_HS_AlertPkt_t));
            break;

        case LA_MM_CMD_MID:
            MM_ProcessCommand(&SBBufPtr->Msg);
            break;

        case LA_MM_ABORT_CMD_MID:
            MM.AbortReason = 3;
            MM_TransitionTo(LA_PHASE_ABORT);
            break;

        default:
            break;
        }
    }

    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}

/* ══════════════════════════════════════════════════════════
 * FLIGHT PHASE FSM
 * ══════════════════════════════════════════════════════════ */

static void MM_UpdateFSM(void)
{
    if (MM.MissionMode == 1) return;

    /* ── Auto-abort: any active flight phase ── */
    bool in_flight = (MM.CurrentPhase >= LA_PHASE_POWERED_ASC &&
                      MM.CurrentPhase <= LA_PHASE_TERMINAL) ||
                     MM.CurrentPhase == LA_PHASE_RENDEZVOUS  ||
                     MM.CurrentPhase == LA_PHASE_DOCKING     ||
                     MM.CurrentPhase == LA_PHASE_UNDOCKING;

    if (in_flight) {
        if (MM.PropValid) {
            double total_prop = MM.LastProp.OxMass_kg
                              + MM.LastProp.FuelMass_kg;
            if (total_prop < MM_PROP_ABORT_THRESH_KG) {
                MM.AbortReason = 1;
                MM_TransitionTo(LA_PHASE_ABORT);
                return;
            }
        }
        if (MM.NavValid && MM.LastNav.NavHealth == 2) {
            MM.AbortReason = 2;
            MM_TransitionTo(LA_PHASE_ABORT);
            return;
        }
    }

    /* ── Phase transitions ── */
    switch (MM.CurrentPhase) {

    case LA_PHASE_PREFLIGHT:
        break;

    /* ── COMMON (both profiles) ── */

    case LA_PHASE_POWERED_ASC:
        if (MM.PropValid && MM.LastProp.EngineState == 3)
            MM.BurnTime_s += 1.0;
        if (MM.BurnTime_s >= 15.0) {
            MM.BurnTime_s = 0.0;
            MM_TransitionTo(LA_PHASE_COAST);
        }
        if (MM.PhaseTimer_s > 25 && MM.BurnTime_s < 1.0) {
            MM.AbortReason = 1;
            MM_TransitionTo(LA_PHASE_ABORT);
        }
        break;

    case LA_PHASE_COAST:
        if (MM.PhaseTimer_s > 20)
            MM_TransitionTo(LA_PHASE_POWERED_DES);
        break;

    case LA_PHASE_POWERED_DES:
        if (MM.PropValid && MM.LastProp.EngineState == 3)
            MM.BurnTime_s += 1.0;
        if (MM.BurnTime_s >= 15.0) {
            MM.BurnTime_s = 0.0;
            MM_TransitionTo(LA_PHASE_HOVER);
        }
        if (MM.PhaseTimer_s > 30) {
            MM.BurnTime_s = 0.0;
            MM_TransitionTo(LA_PHASE_HOVER);
        }
        break;

    case LA_PHASE_HOVER:
        if (MM.MissionType == MM_MISSION_DOCK) {
            if (MM.PhaseTimer_s > 8)
                MM_TransitionTo(LA_PHASE_RENDEZVOUS);
        } else {
            if (MM.PhaseTimer_s > 8)
                MM_TransitionTo(LA_PHASE_TERMINAL);
        }
        break;

    /* ── SURFACE ── */

    case LA_PHASE_TERMINAL:
        if (MM.PhaseTimer_s > 6)
            MM_TransitionTo(LA_PHASE_LANDED);
        break;

    case LA_PHASE_LANDED:
        break;

    /* ── ORBITAL DOCKING ── */

    case LA_PHASE_RENDEZVOUS:
        if (MM.PhaseTimer_s > 10)
            MM_TransitionTo(LA_PHASE_DOCKING);
        break;

    case LA_PHASE_DOCKING:
        if (MM.DockValid &&
            MM.LastDock.CollarState == 3 &&
            MM.LastDock.SealPressOK == 1)
        {
            MM_TransitionTo(LA_PHASE_DOCKED);
        }
        if (MM.PhaseTimer_s > 60) {
            MM.AbortReason = 4;
            MM_TransitionTo(LA_PHASE_ABORT);
        }
        break;

    case LA_PHASE_DOCKED:
        break;

    /* ── DEPARTURE ── */

    case LA_PHASE_UNDOCKING:
        if (MM.DockValid && MM.LastDock.CollarState == 0) {
            CFE_EVS_SendEvent(64, CFE_EVS_EventType_INFORMATION,
                "MM: Collar retracted — cleared for departure");
            MM_TransitionTo(LA_PHASE_POWERED_ASC);
        }
        if (MM.PhaseTimer_s > 30) {
            MM.AbortReason = 4;
            CFE_EVS_SendEvent(70, CFE_EVS_EventType_CRITICAL,
                "MM: UNDOCKING timeout — collar did not retract");
            MM_TransitionTo(LA_PHASE_ABORT);
        }
        break;

    /* ── ABORT / SAFED ── */

    case LA_PHASE_ABORT:
        if (MM.PhaseTimer_s > 10)
            MM_TransitionTo(LA_PHASE_SAFED);
        break;

    case LA_PHASE_SAFED:
        break;

    default:
        break;
    }
}

/* ── Subsystem Command Helper ────────────────────────────── */

static void MM_SendSubsystemCmd(uint16_t mid_raw, uint16_t fc)
{
    CFE_MSG_CommandHeader_t cmd;
    CFE_MSG_Init(&cmd.Msg, CFE_SB_ValueToMsgId(mid_raw), sizeof(cmd));
    CFE_MSG_SetFcnCode(&cmd.Msg, fc);
    CFE_SB_TransmitMsg(&cmd.Msg, true);
}

/* ── Phase Transition + Subsystem Orchestration ──────────── */

static void MM_TransitionTo(LA_FlightPhase_t new_phase)
{
    if (new_phase == MM.CurrentPhase) return;

    /* ABORT and SAFED are never gated — safety-critical escape paths */
    if (new_phase != LA_PHASE_ABORT && new_phase != LA_PHASE_SAFED) {
        SeqGate_Diag_t diag;
        SeqGate_Result_t gate = SeqGate_Check(
            &MM.GateTable,
            (uint8_t)MM.CurrentPhase,
            (uint8_t)new_phase,
            false,   /* non-strict: ungated transitions pass */
            &diag);

        if (gate != SEQGATE_PASS) {
            CFE_EVS_SendEvent(80, CFE_EVS_EventType_ERROR,
                "MM: GATE BLOCKED %s -> %s — prereq failed: %s",
                phase_names[MM.CurrentPhase],
                phase_names[new_phase],
                diag.failed_name);
            return;
        }
    }

    MM.PreviousPhase = MM.CurrentPhase;
    MM.CurrentPhase  = new_phase;
    MM.PhaseTimer_s  = 0;

    uint16_t evt_type = (new_phase == LA_PHASE_ABORT)
                        ? CFE_EVS_EventType_CRITICAL
                        : CFE_EVS_EventType_INFORMATION;

    CFE_EVS_SendEvent(10 + (uint16_t)new_phase, evt_type,
                      "MM: Phase transition %s -> %s",
                      phase_names[MM.PreviousPhase],
                      phase_names[MM.CurrentPhase]);

    switch (new_phase) {

    case LA_PHASE_POWERED_ASC:
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_ARM_CC);
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_START_CC);
        MM_SendSubsystemCmd(LA_LG_CMD_MID, LG_RETRACT_CC);
        CFE_EVS_SendEvent(50, CFE_EVS_EventType_INFORMATION,
            "MM: PROP arm+start, LG retract [%s]",
            MM.MissionType == MM_MISSION_DOCK ? "DOCKING" : "SURFACE");
        break;

    case LA_PHASE_COAST:
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_SHUTDOWN_CC);
        CFE_EVS_SendEvent(51, CFE_EVS_EventType_INFORMATION,
            "MM: PROP shutdown for coast");
        break;

    case LA_PHASE_POWERED_DES:
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_ARM_CC);
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_START_CC);
        CFE_EVS_SendEvent(52, CFE_EVS_EventType_INFORMATION,
            "MM: PROP arm+start for %s",
            MM.MissionType == MM_MISSION_DOCK ? "approach [DOCKING]"
                                              : "descent [SURFACE]");
        break;

    case LA_PHASE_HOVER:
        if (MM.MissionType == MM_MISSION_SURFACE) {
            MM_SendSubsystemCmd(LA_LG_CMD_MID, LG_DEPLOY_CC);
            CFE_EVS_SendEvent(53, CFE_EVS_EventType_INFORMATION,
                "MM: LG deploy for surface landing");
        } else {
            CFE_EVS_SendEvent(53, CFE_EVS_EventType_INFORMATION,
                "MM: Station-keeping — LG stowed for rendezvous");
        }
        break;

    case LA_PHASE_TERMINAL:
        CFE_EVS_SendEvent(54, CFE_EVS_EventType_INFORMATION,
            "MM: Terminal descent");
        break;

    case LA_PHASE_LANDED:
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_SHUTDOWN_CC);
        CFE_EVS_SendEvent(55, CFE_EVS_EventType_INFORMATION,
            "MM: TOUCHDOWN — PROP shutdown");
        break;

    case LA_PHASE_RENDEZVOUS:
        CFE_EVS_SendEvent(60, CFE_EVS_EventType_INFORMATION,
            "MM: RENDEZVOUS — closing on target");
        break;

    case LA_PHASE_DOCKING:
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_SHUTDOWN_CC);
        MM_SendSubsystemCmd(LA_DOCK_CMD_MID, DOCK_MATE_CC);
        CFE_EVS_SendEvent(61, CFE_EVS_EventType_INFORMATION,
            "MM: DOCKING — PROP shutdown, DOCK mate initiated");
        break;

    case LA_PHASE_DOCKED:
        CFE_EVS_SendEvent(62, CFE_EVS_EventType_INFORMATION,
            "MM: DOCKED — hard dock confirmed, seal nominal");
        break;

    case LA_PHASE_UNDOCKING:
        MM_SendSubsystemCmd(LA_DOCK_CMD_MID, DOCK_DEMATE_CC);
        CFE_EVS_SendEvent(63, CFE_EVS_EventType_INFORMATION,
            "MM: UNDOCKING — demate commanded, awaiting retract");
        break;

    case LA_PHASE_ABORT:
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_SHUTDOWN_CC);
        if (MM.MissionType == MM_MISSION_DOCK) {
            if (MM.PreviousPhase == LA_PHASE_DOCKING ||
                MM.PreviousPhase == LA_PHASE_DOCKED  ||
                MM.PreviousPhase == LA_PHASE_UNDOCKING)
            {
                MM_SendSubsystemCmd(LA_DOCK_CMD_MID, DOCK_DEMATE_CC);
            }
            CFE_EVS_SendEvent(56, CFE_EVS_EventType_CRITICAL,
                "MM: ABORT — PROP shutdown, DOCK demate");
        } else {
            MM_SendSubsystemCmd(LA_LG_CMD_MID, LG_DEPLOY_CC);
            CFE_EVS_SendEvent(56, CFE_EVS_EventType_CRITICAL,
                "MM: ABORT — PROP shutdown, LG deploy");
        }
        break;

    default:
        break;
    }
}

/* ── Command Processing ──────────────────────────────────── */

static void MM_ProcessCommand(const CFE_MSG_Message_t *MsgPtr)
{
    CFE_MSG_FcnCode_t fc;
    CFE_MSG_GetFcnCode(MsgPtr, &fc);

    switch (fc) {
    case MM_NOOP_CC:
        MM.CmdCount++;
        CFE_EVS_SendEvent(20, CFE_EVS_EventType_INFORMATION,
                          "MM: NOOP received — v3.0");
        break;

    case MM_RESET_CC:
        MM.CmdCount = 0;
        MM.ErrCount = 0;
        break;

    case MM_START_MISSION_CC:
        if (MM.CurrentPhase == LA_PHASE_DOCKED) {
            /* Departing from dock — undock first, then auto-ignite */
            MM.MET_s = 0.0;
            MM.BurnTime_s = 0.0;
            MM.AbortReason = 0;
            MM.MissionMode = 0;
            MM_TransitionTo(LA_PHASE_UNDOCKING);
            MM.CmdCount++;
            CFE_EVS_SendEvent(27, CFE_EVS_EventType_INFORMATION,
                "MM: Mission START from DOCKED — undocking");
        }
        else if (MM.CurrentPhase == LA_PHASE_PREFLIGHT ||
                 MM.CurrentPhase == LA_PHASE_LANDED    ||
                 MM.CurrentPhase == LA_PHASE_SAFED)
        {
            MM.MET_s = 0.0;
            MM.BurnTime_s = 0.0;
            MM.AbortReason = 0;
            MM.MissionMode = 0;
            MM_TransitionTo(LA_PHASE_POWERED_ASC);
            MM.CmdCount++;
            CFE_EVS_SendEvent(27, CFE_EVS_EventType_INFORMATION,
                "MM: Mission START [%s]",
                MM.MissionType == MM_MISSION_DOCK ? "DOCKING" : "SURFACE");
        }
        else {
            MM.ErrCount++;
            CFE_EVS_SendEvent(21, CFE_EVS_EventType_ERROR,
                "MM: START rejected — phase %s",
                phase_names[MM.CurrentPhase]);
        }
        break;

    case MM_ABORT_CC:
        MM.AbortReason = 3;
        MM_TransitionTo(LA_PHASE_ABORT);
        MM.CmdCount++;
        break;

    case MM_MANUAL_CC:
        MM.MissionMode = 1;
        MM.CmdCount++;
        CFE_EVS_SendEvent(22, CFE_EVS_EventType_INFORMATION,
                          "MM: Switched to MANUAL mode");
        break;

    case MM_AUTO_CC:
        MM.MissionMode = 0;
        MM.CmdCount++;
        CFE_EVS_SendEvent(23, CFE_EVS_EventType_INFORMATION,
                          "MM: Switched to AUTO mode");
        break;

    case MM_SET_DOCK_CC:
        if (MM.CurrentPhase == LA_PHASE_PREFLIGHT ||
            MM.CurrentPhase == LA_PHASE_LANDED    ||
            MM.CurrentPhase == LA_PHASE_DOCKED    ||
            MM.CurrentPhase == LA_PHASE_SAFED)
        {
            MM.MissionType = MM_MISSION_DOCK;
            MM.CmdCount++;
            CFE_EVS_SendEvent(25, CFE_EVS_EventType_INFORMATION,
                "MM: Profile set to ORBITAL DOCKING");
        } else {
            MM.ErrCount++;
            CFE_EVS_SendEvent(28, CFE_EVS_EventType_ERROR,
                "MM: SET_DOCK rejected — not in idle phase");
        }
        break;

    case MM_SET_SURFACE_CC:
        if (MM.CurrentPhase == LA_PHASE_PREFLIGHT ||
            MM.CurrentPhase == LA_PHASE_LANDED    ||
            MM.CurrentPhase == LA_PHASE_DOCKED    ||
            MM.CurrentPhase == LA_PHASE_SAFED)
        {
            MM.MissionType = MM_MISSION_SURFACE;
            MM.CmdCount++;
            CFE_EVS_SendEvent(26, CFE_EVS_EventType_INFORMATION,
                "MM: Profile set to SURFACE LANDING");
        } else {
            MM.ErrCount++;
            CFE_EVS_SendEvent(29, CFE_EVS_EventType_ERROR,
                "MM: SET_SURFACE rejected — not in idle phase");
        }
        break;

    default:
        MM.ErrCount++;
        CFE_EVS_SendEvent(24, CFE_EVS_EventType_ERROR,
                          "MM: Invalid function code %u", fc);
        break;
    }
}

/* ── Telemetry ───────────────────────────────────────────── */

static void MM_SendPhasePkt(void)
{
    MM.PhasePkt.CurrentPhase    = (uint8_t)MM.CurrentPhase;
    MM.PhasePkt.PreviousPhase   = (uint8_t)MM.PreviousPhase;
    MM.PhasePkt.AbortReason     = MM.AbortReason;
    MM.PhasePkt.MissionMode     = MM.MissionMode;
    MM.PhasePkt.MissionType     = MM.MissionType;
    MM.PhasePkt.MET_s           = MM.MET_s;
    MM.PhasePkt.PhaseTimer_s    = MM.PhaseTimer_s;

    if (MM.PropValid) {
        double total_prop = MM.LastProp.OxMass_kg
                          + MM.LastProp.FuelMass_kg;
        MM.PhasePkt.PropRemaining_kg = total_prop;
        double m_wet = 5250.0 + total_prop;
        double m_dry = 5250.0;
        if (m_wet > m_dry) {
            MM.PhasePkt.DvRemaining_mps = 440.0 * 9.80665
                                          * log(m_wet / m_dry);
        } else {
            MM.PhasePkt.DvRemaining_mps = 0.0;
        }
    }

    CFE_SB_TransmitMsg(&MM.PhasePkt.TlmHdr.Msg, true);
}
