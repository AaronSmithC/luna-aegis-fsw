/**
 * @file la_mm_app.c
 * @brief Luna-Aegis Mission Manager (MM) Application
 *
 * Flight phase finite state machine.  Controls the overall
 * mission sequence:
 *   PREFLIGHT → POWERED_ASC → COAST → POWERED_DES →
 *   HOVER → TERMINAL → LANDED
 * with ABORT reachable from any powered phase.
 *
 * Subscribes: NAV_StatePkt, PROP_StatusPkt, HS_AlertPkt, MM_CMD
 * Publishes:  MM_PhasePkt (1 Hz), MM_AbortCmd (async)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include <string.h>
#include <math.h>

/* ── App Data ────────────────────────────────────────────── */

#define MM_PIPE_DEPTH     32
#define MM_PIPE_NAME      "MM_PIPE"

/* Phase transition thresholds */
#define MM_ASC_DV_THRESHOLD_MPS    850.0   /* Coast insertion Δv      */
#define MM_COAST_ALT_THRESH_M      50000.0 /* Begin descent burn      */
#define MM_HOVER_ALT_THRESH_M      100.0   /* Transition to hover     */
#define MM_TERMINAL_ALT_THRESH_M   10.0    /* Terminal descent         */
#define MM_TOUCHDOWN_VEL_THRESH    0.5     /* Landed detection (m/s)  */
#define MM_PROP_ABORT_THRESH_KG    50.0    /* Min prop for abort      */

/* Command function codes */
#define MM_NOOP_CC          0
#define MM_RESET_CC         1
#define MM_START_MISSION_CC 2
#define MM_ABORT_CC         3
#define MM_MANUAL_CC        4
#define MM_AUTO_CC          5
#define MM_SET_DOCK_CC      6  /* Set orbital/docking mission profile  */
#define MM_SET_SURFACE_CC   7  /* Set surface landing mission profile  */

/* Mission types */
#define MM_MISSION_SURFACE  0  /* Landing profile: HOVER → TERMINAL → LANDED */
#define MM_MISSION_DOCK     1  /* Docking profile: HOVER → TERMINAL → LANDED (dock) */

typedef struct {
    /* cFS infrastructure */
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    /* Mission state */
    LA_FlightPhase_t CurrentPhase;
    LA_FlightPhase_t PreviousPhase;
    uint8_t          MissionMode;     /* 0=auto, 1=manual, 2=abort-seq */
    uint8_t          MissionType;     /* 0=surface landing, 1=orbital docking */
    uint8_t          AbortReason;
    double           MET_s;           /* Mission elapsed time          */
    uint32_t         PhaseTimer_s;
    double           BurnTime_s;      /* Accumulated engine-on time in current burn */

    /* Cached inputs */
    LA_NAV_StatePkt_t  LastNav;
    LA_PROP_StatusPkt_t LastProp;
    LA_HS_AlertPkt_t   LastHS;
    bool               NavValid;
    bool               PropValid;

    /* Output packet */
    LA_MM_PhasePkt_t   PhasePkt;

    /* Telemetry counters */
    uint16_t         CmdCount;
    uint16_t         ErrCount;
} MM_AppData_t;

static MM_AppData_t MM;

/* ── Forward Declarations ────────────────────────────────── */

static void MM_ProcessCommand(const CFE_MSG_Message_t *MsgPtr);
static void MM_UpdateFSM(void);
static void MM_SendPhasePkt(void);
static void MM_TransitionTo(LA_FlightPhase_t new_phase);

/* ── Initialization ──────────────────────────────────────── */

static CFE_Status_t MM_Init(void)
{
    memset(&MM, 0, sizeof(MM));
    MM.RunStatus = CFE_ES_RunStatus_APP_RUN;
    MM.CurrentPhase = LA_PHASE_PREFLIGHT;
    MM.MissionMode  = 0; /* auto */

    /* Register with EVS */
    CFE_EVS_Register(NULL, 0, 0);

    /* Create SB pipe */
    CFE_SB_CreatePipe(&MM.CmdPipe, MM_PIPE_DEPTH, MM_PIPE_NAME);

    /* Subscribe to inputs */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_CMD_MID),           MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_ABORT_CMD_MID),     MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_NAV_STATE_TLM_MID),    MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_PROP_STATUS_TLM_MID),  MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_HS_ALERT_TLM_MID),     MM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x08)), MM.CmdPipe); /* 1 Hz wakeup */

    /* Initialize output packet */
    CFE_MSG_Init(&MM.PhasePkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),
                 sizeof(LA_MM_PhasePkt_t));

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "MM: Mission Manager initialized — phase PREFLIGHT");

    return CFE_SUCCESS;
}

/* ── Main Loop ───────────────────────────────────────────── */

void MM_AppMain(void)
{
    CFE_Status_t      status;
    CFE_SB_Buffer_t *SBBufPtr;
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

        /* ── Wakeup: run FSM and send TLM ── */
        case LA_SCH_WAKEUP_MID(0x08):
            MM.MET_s      += 1.0;
            MM.PhaseTimer_s++;
            MM_UpdateFSM();
            MM_SendPhasePkt();
            break;

        /* ── Nav state update ── */
        case LA_NAV_STATE_TLM_MID:
            memcpy(&MM.LastNav, SBBufPtr, sizeof(LA_NAV_StatePkt_t));
            MM.NavValid = true;
            break;

        /* ── Prop status update ── */
        case LA_PROP_STATUS_TLM_MID:
            memcpy(&MM.LastProp, SBBufPtr, sizeof(LA_PROP_StatusPkt_t));
            MM.PropValid = true;
            break;

        /* ── Health alert ── */
        case LA_HS_ALERT_TLM_MID:
            memcpy(&MM.LastHS, SBBufPtr, sizeof(LA_HS_AlertPkt_t));
            break;

        /* ── Ground / crew commands ── */
        case LA_MM_CMD_MID:
            MM_ProcessCommand(&SBBufPtr->Msg);
            break;

        /* ── Abort command (high priority) ── */
        case LA_MM_ABORT_CMD_MID:
            MM.AbortReason = 3; /* commanded */
            MM_TransitionTo(LA_PHASE_ABORT);
            break;

        default:
            break;
        }
    }

    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}

/* ── Flight Phase FSM ────────────────────────────────────── */

static void MM_UpdateFSM(void)
{
    if (MM.MissionMode == 1) return; /* Manual mode: no auto transitions */

    /* Auto-abort checks (any powered phase) */
    if (MM.CurrentPhase >= LA_PHASE_POWERED_ASC &&
        MM.CurrentPhase <= LA_PHASE_TERMINAL)
    {
        /* Prop depletion abort */
        if (MM.PropValid) {
            double total_prop = MM.LastProp.OxMass_kg
                              + MM.LastProp.FuelMass_kg;
            if (total_prop < MM_PROP_ABORT_THRESH_KG) {
                MM.AbortReason = 1; /* propulsion */
                MM_TransitionTo(LA_PHASE_ABORT);
                return;
            }
        }

        /* Nav loss abort */
        if (MM.NavValid && MM.LastNav.NavHealth == 2) {
            MM.AbortReason = 2; /* nav lost */
            MM_TransitionTo(LA_PHASE_ABORT);
            return;
        }
    }

    /* Phase-specific transitions */
    switch (MM.CurrentPhase) {

    case LA_PHASE_PREFLIGHT:
        /* Waiting for START_MISSION command — no auto transition */
        break;

    case LA_PHASE_POWERED_ASC:
        /* Wait for engine to be running before counting burn time.
         * Ascent burn: ~10 seconds at full thrust to reach coast insertion.
         * Min 5s with engine running before coast transition. */
        if (MM.PropValid && MM.LastProp.EngineState == 3 /* HAL_ENGINE_RUNNING */) {
            MM.BurnTime_s += 1.0;
        }
        if (MM.BurnTime_s >= 15.0) {
            MM.BurnTime_s = 0.0;
            MM_TransitionTo(LA_PHASE_COAST);
        }
        /* Timeout abort if engine doesn't start within 15s */
        if (MM.PhaseTimer_s > 25 && MM.BurnTime_s < 1.0) {
            MM.AbortReason = 1; /* propulsion */
            MM_TransitionTo(LA_PHASE_ABORT);
        }
        break;

    case LA_PHASE_COAST:
        /* Ballistic coast: 20 seconds */
        if (MM.PhaseTimer_s > 20) {
            MM_TransitionTo(LA_PHASE_POWERED_DES);
        }
        break;

    case LA_PHASE_POWERED_DES:
        /* Descent burn: wait for engine running, then ~8 seconds */
        if (MM.PropValid && MM.LastProp.EngineState == 3 /* HAL_ENGINE_RUNNING */) {
            MM.BurnTime_s += 1.0;
        }
        if (MM.BurnTime_s >= 15.0) {
            MM.BurnTime_s = 0.0;
            MM_TransitionTo(LA_PHASE_HOVER);
        }
        /* Timeout: proceed to hover even without engine (will use RCS) */
        if (MM.PhaseTimer_s > 30) {
            MM.BurnTime_s = 0.0;
            MM_TransitionTo(LA_PHASE_HOVER);
        }
        break;

    case LA_PHASE_HOVER:
        if (MM.MissionType == MM_MISSION_DOCK) {
            /* Docking: hover = station-keeping near target, then approach */
            if (MM.PhaseTimer_s > 10) {
                CFE_EVS_SendEvent(30, CFE_EVS_EventType_INFORMATION,
                                  "MM: Approach phase — initiating docking sequence");
                MM_TransitionTo(LA_PHASE_TERMINAL);
            }
        } else {
            /* Surface: hover at ~50 m, 8 seconds, then terminal descent */
            if (MM.PhaseTimer_s > 8) {
                MM_TransitionTo(LA_PHASE_TERMINAL);
            }
        }
        break;

    case LA_PHASE_TERMINAL:
        if (MM.MissionType == MM_MISSION_DOCK) {
            /* Docking approach: 10 seconds then transition to LANDED (docked) */
            if (MM.PhaseTimer_s > 10) {
                CFE_EVS_SendEvent(31, CFE_EVS_EventType_INFORMATION,
                                  "MM: Docking — commanding DOCK MATE");
                /* Send DOCK MATE command */
                CFE_MSG_CommandHeader_t dock_cmd;
                CFE_MSG_Init(&dock_cmd.Msg,
                             CFE_SB_ValueToMsgId(LA_DOCK_CMD_MID),
                             sizeof(dock_cmd));
                CFE_MSG_SetFcnCode(&dock_cmd.Msg, 2); /* MATE */
                CFE_SB_TransmitMsg(&dock_cmd.Msg, true);
                MM_TransitionTo(LA_PHASE_LANDED);
            }
        } else {
            /* Surface terminal descent: 6 seconds */
            if (MM.PhaseTimer_s > 6) {
                MM_TransitionTo(LA_PHASE_LANDED);
            }
        }
        break;

    case LA_PHASE_LANDED:
        /* Terminal state — docked or landed, awaiting safing or new mission */
        if (MM.MissionType == MM_MISSION_DOCK && MM.PhaseTimer_s == 1) {
            CFE_EVS_SendEvent(32, CFE_EVS_EventType_INFORMATION,
                              "MM: DOCKED at Aegis Station — systems nominal");
        }
        break;

    case LA_PHASE_ABORT:
        /* Abort sequence: engine shutdown, safing */
        if (MM.PhaseTimer_s > 10) {
            MM_TransitionTo(LA_PHASE_SAFED);
        }
        break;

    case LA_PHASE_SAFED:
        /* Terminal safe state */
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

/* PROP command function codes (must match la_prop_app.c) */
#define PROP_ARM_CC       2
#define PROP_START_CC     4
#define PROP_SHUTDOWN_CC  5

/* LG command function codes (must match la_lg_app.c) */
#define LG_DEPLOY_CC      2
#define LG_RETRACT_CC     3

static void MM_TransitionTo(LA_FlightPhase_t new_phase)
{
    if (new_phase == MM.CurrentPhase) return;

    MM.PreviousPhase = MM.CurrentPhase;
    MM.CurrentPhase  = new_phase;
    MM.PhaseTimer_s  = 0;

    static const char *phase_names[] = {
        "PREFLIGHT", "POWERED_ASC", "COAST", "POWERED_DES",
        "HOVER", "TERMINAL", "LANDED", "ABORT", "SAFED"
    };

    uint16_t evt_type = (new_phase == LA_PHASE_ABORT)
                        ? CFE_EVS_EventType_CRITICAL
                        : CFE_EVS_EventType_INFORMATION;

    CFE_EVS_SendEvent(10 + new_phase, evt_type,
                      "MM: Phase transition %s → %s",
                      phase_names[MM.PreviousPhase],
                      phase_names[MM.CurrentPhase]);

    /* ── Subsystem commands on phase entry ── */
    switch (new_phase) {

    case LA_PHASE_POWERED_ASC:
        /* Arm and start engine */
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_ARM_CC);
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_START_CC);
        if (MM.MissionType == MM_MISSION_SURFACE) {
            MM_SendSubsystemCmd(LA_LG_CMD_MID, LG_RETRACT_CC);
            CFE_EVS_SendEvent(50, CFE_EVS_EventType_INFORMATION,
                              "MM: Commanding PROP arm+start, LG retract");
        } else {
            CFE_EVS_SendEvent(50, CFE_EVS_EventType_INFORMATION,
                              "MM: Commanding PROP arm+start (docking profile)");
        }
        break;

    case LA_PHASE_COAST:
        /* Engine cutoff for coast */
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_SHUTDOWN_CC);
        CFE_EVS_SendEvent(51, CFE_EVS_EventType_INFORMATION,
                          "MM: Commanding PROP shutdown for coast");
        break;

    case LA_PHASE_POWERED_DES:
        /* Restart engine for descent/approach burn */
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_ARM_CC);
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_START_CC);
        if (MM.MissionType == MM_MISSION_SURFACE) {
            CFE_EVS_SendEvent(52, CFE_EVS_EventType_INFORMATION,
                              "MM: Commanding PROP arm+start for descent");
        } else {
            CFE_EVS_SendEvent(52, CFE_EVS_EventType_INFORMATION,
                              "MM: Commanding PROP arm+start for rendezvous burn");
        }
        break;

    case LA_PHASE_HOVER:
        if (MM.MissionType == MM_MISSION_SURFACE) {
            /* Surface: deploy landing gear */
            MM_SendSubsystemCmd(LA_LG_CMD_MID, LG_DEPLOY_CC);
            CFE_EVS_SendEvent(53, CFE_EVS_EventType_INFORMATION,
                              "MM: Commanding LG deploy for landing");
        } else {
            /* Docking: station-keeping, no gear */
            CFE_EVS_SendEvent(53, CFE_EVS_EventType_INFORMATION,
                              "MM: Station-keeping for docking approach");
        }
        break;

    case LA_PHASE_LANDED:
        /* Shutdown engine */
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_SHUTDOWN_CC);
        if (MM.MissionType == MM_MISSION_DOCK) {
            CFE_EVS_SendEvent(54, CFE_EVS_EventType_INFORMATION,
                              "MM: DOCKED — commanding PROP shutdown");
        } else {
            CFE_EVS_SendEvent(54, CFE_EVS_EventType_INFORMATION,
                              "MM: TOUCHDOWN — commanding PROP shutdown");
        }
        break;

    case LA_PHASE_ABORT:
        /* Emergency: shutdown engine, deploy gear */
        MM_SendSubsystemCmd(LA_PROP_CMD_MID, PROP_SHUTDOWN_CC);
        MM_SendSubsystemCmd(LA_LG_CMD_MID,   LG_DEPLOY_CC);
        CFE_EVS_SendEvent(55, CFE_EVS_EventType_CRITICAL,
                          "MM: ABORT — commanding PROP shutdown, LG deploy");
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
                          "MM: NOOP received — v1.0");
        break;

    case MM_RESET_CC:
        MM.CmdCount = 0;
        MM.ErrCount = 0;
        break;

    case MM_START_MISSION_CC:
        if (MM.CurrentPhase == LA_PHASE_PREFLIGHT ||
            MM.CurrentPhase == LA_PHASE_LANDED ||
            MM.CurrentPhase == LA_PHASE_SAFED)
        {
            MM.MET_s = 0.0;
            MM.BurnTime_s = 0.0;
            MM.AbortReason = 0;
            MM.MissionMode = 0; /* auto */
            MM_TransitionTo(LA_PHASE_POWERED_ASC);
            MM.CmdCount++;
        } else {
            MM.ErrCount++;
            CFE_EVS_SendEvent(21, CFE_EVS_EventType_ERROR,
                              "MM: START rejected — not in PREFLIGHT/LANDED/SAFED");
        }
        break;

    case MM_ABORT_CC:
        MM.AbortReason = 3; /* commanded */
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
        MM.MissionType = MM_MISSION_DOCK;
        MM.CmdCount++;
        CFE_EVS_SendEvent(25, CFE_EVS_EventType_INFORMATION,
                          "MM: Mission profile set to ORBITAL DOCKING");
        break;

    case MM_SET_SURFACE_CC:
        MM.MissionType = MM_MISSION_SURFACE;
        MM.CmdCount++;
        CFE_EVS_SendEvent(26, CFE_EVS_EventType_INFORMATION,
                          "MM: Mission profile set to SURFACE LANDING");
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
    MM.PhasePkt.MET_s           = MM.MET_s;
    MM.PhasePkt.PhaseTimer_s    = MM.PhaseTimer_s;

    /* Compute remaining Δv from prop status */
    if (MM.PropValid) {
        double total_prop = MM.LastProp.OxMass_kg
                          + MM.LastProp.FuelMass_kg;
        MM.PhasePkt.PropRemaining_kg = total_prop;
        /* Δv = Isp * g0 * ln(m_wet / m_dry) — simplified */
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
