/**
 * @file la_lc_app.c
 * @brief Luna-Aegis Limit Checker (LC) Application
 *
 * Evaluates watchpoints (boolean conditions on telemetry values)
 * and triggers actionpoints (responses: events, commands, safing).
 *
 * Subscribes: All *_StatusPkt, SCH Wakeup (1 Hz)
 * Publishes:  LC_ActionPkt (1 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include <string.h>

#define LC_PIPE_DEPTH   32
#define LC_PIPE_NAME    "LC_PIPE"

/* Maximum watchpoints and actionpoints */
#define LC_MAX_WP   32
#define LC_MAX_AP   16

/* Watchpoint types */
typedef enum {
    LC_WP_NONE = 0,
    LC_WP_LT,         /* value < threshold */
    LC_WP_GT,         /* value > threshold */
    LC_WP_RANGE_OUT,  /* value outside [lo, hi] */
} LC_WP_Type_t;

typedef struct {
    uint8_t        type;        /* LC_WP_Type_t */
    CFE_SB_MsgId_t msg_id;     /* Which packet */
    uint16_t       offset;     /* Byte offset of double in packet */
    double         threshold;  /* For LT/GT */
    double         lo, hi;     /* For RANGE_OUT */
    bool           result;     /* Last evaluation */
} LC_Watchpoint_t;

typedef struct {
    uint8_t  wp_index;   /* Which watchpoint triggers this */
    uint8_t  action;     /* 0=event, 1=command, 2=safing */
    bool     triggered;
} LC_Actionpoint_t;

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    LC_Watchpoint_t   WP[LC_MAX_WP];
    LC_Actionpoint_t  AP[LC_MAX_AP];
    uint16_t         ActiveWP;
    uint16_t         ActiveAP;

    uint8_t  LCState;  /* 0=active, 1=passive, 2=disabled */

    /* Cached telemetry for evaluation */
    LA_PROP_StatusPkt_t  CachedProp;
    LA_EPS_StatusPkt_t   CachedEPS;
    LA_LSS_StatusPkt_t   CachedLSS;
    LA_NAV_StatePkt_t    CachedNav;

    LA_LC_ActionPkt_t ActionPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} LC_AppData_t;

static LC_AppData_t LC;

static CFE_Status_t LC_Init(void)
{
    memset(&LC, 0, sizeof(LC));
    LC.RunStatus = CFE_ES_RunStatus_APP_RUN;
    LC.LCState   = 0; /* active */

    /*
     * Default watchpoint table (loaded from table in real cFS).
     * Here we hardcode critical limits.
     */

    /* WP0: Propellant low */
    LC.WP[0].type      = LC_WP_LT;
    LC.WP[0].msg_id    = CFE_SB_ValueToMsgId(LA_PROP_STATUS_TLM_MID);
    LC.WP[0].threshold = 100.0; /* kg total prop */
    LC.ActiveWP++;

    /* WP1: Battery SOC critical */
    LC.WP[1].type      = LC_WP_LT;
    LC.WP[1].msg_id    = CFE_SB_ValueToMsgId(LA_EPS_STATUS_TLM_MID);
    LC.WP[1].threshold = 10.0; /* % */
    LC.ActiveWP++;

    /* WP2: CO2 over limit */
    LC.WP[2].type      = LC_WP_GT;
    LC.WP[2].msg_id    = CFE_SB_ValueToMsgId(LA_LSS_STATUS_TLM_MID);
    LC.WP[2].threshold = 0.53; /* kPa */
    LC.ActiveWP++;

    /* WP3: Nav health lost */
    LC.WP[3].type      = LC_WP_GT;
    LC.WP[3].msg_id    = CFE_SB_ValueToMsgId(LA_NAV_STATE_TLM_MID);
    LC.WP[3].threshold = 1.5; /* NavHealth > 1 = lost */
    LC.ActiveWP++;

    /* AP0: Prop low → event */
    LC.AP[0].wp_index = 0;
    LC.AP[0].action   = 0;
    LC.ActiveAP++;

    /* AP1: Battery critical → load shed event */
    LC.AP[1].wp_index = 1;
    LC.AP[1].action   = 0;
    LC.ActiveAP++;

    /* AP2: CO2 → safing */
    LC.AP[2].wp_index = 2;
    LC.AP[2].action   = 2;
    LC.ActiveAP++;

    /* AP3: Nav lost → event */
    LC.AP[3].wp_index = 3;
    LC.AP[3].action   = 0;
    LC.ActiveAP++;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&LC.CmdPipe, LC_PIPE_DEPTH, LC_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LC_CMD_MID),             LC.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x12)),   LC.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_PROP_STATUS_TLM_MID),    LC.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_EPS_STATUS_TLM_MID),     LC.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LSS_STATUS_TLM_MID),     LC.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_NAV_STATE_TLM_MID),      LC.CmdPipe);

    CFE_MSG_Init(&LC.ActionPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_LC_ACTION_TLM_MID), sizeof(LA_LC_ActionPkt_t));

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "LC: Limit checker initialized — %u WP, %u AP active",
                      LC.ActiveWP, LC.ActiveAP);
    return CFE_SUCCESS;
}

static void LC_EvalWatchpoints(void)
{
    /*
     * Simplified evaluation — in a real cFS LC app, this would
     * use the table-defined offset into the raw packet bytes.
     * Here we evaluate against cached struct fields directly.
     */

    /* WP0: Total prop < 100 kg */
    double total_prop = LC.CachedProp.OxMass_kg + LC.CachedProp.FuelMass_kg;
    LC.WP[0].result = (total_prop < LC.WP[0].threshold);

    /* WP1: SOC < 10% */
    LC.WP[1].result = (LC.CachedEPS.BattSOC_pct < LC.WP[1].threshold);

    /* WP2: CO2 > 0.53 kPa */
    LC.WP[2].result = (LC.CachedLSS.CO2_Partial_kPa > LC.WP[2].threshold);

    /* WP3: NavHealth > 1 */
    LC.WP[3].result = (LC.CachedNav.NavHealth > 1);
}

static void LC_EvalActionpoints(void)
{
    uint32_t wp_results = 0;
    uint32_t ap_results = 0;

    for (uint16_t i = 0; i < LC.ActiveWP; i++) {
        if (LC.WP[i].result) wp_results |= (1 << i);
    }

    for (uint16_t i = 0; i < LC.ActiveAP; i++) {
        uint8_t wp = LC.AP[i].wp_index;
        if (wp < LC.ActiveWP && LC.WP[wp].result) {
            if (!LC.AP[i].triggered) {
                LC.AP[i].triggered = true;
                ap_results |= (1 << i);

                CFE_EVS_SendEvent(20 + i,
                    (LC.AP[i].action == 2) ? CFE_EVS_EventType_CRITICAL
                                           : CFE_EVS_EventType_ERROR,
                    "LC: Actionpoint %u triggered (WP%u, action=%u)",
                    i, wp, LC.AP[i].action);
            }
        } else {
            LC.AP[i].triggered = false;
        }
    }

    LC.ActionPkt.WP_Results    = wp_results;
    LC.ActionPkt.AP_Results    = ap_results;
    LC.ActionPkt.ActiveWPCount = LC.ActiveWP;
    LC.ActionPkt.ActiveAPCount = LC.ActiveAP;
    LC.ActionPkt.LCState       = LC.LCState;
}

static void LC_Process(void)
{
    if (LC.LCState == 2) return; /* Disabled */

    LC_EvalWatchpoints();
    if (LC.LCState == 0) { /* Active: evaluate actionpoints too */
        LC_EvalActionpoints();
    }

    CFE_SB_TransmitMsg(&LC.ActionPkt.TlmHdr.Msg, true);
}

void LC_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (LC_Init() != CFE_SUCCESS) { LC.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&LC.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, LC.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x12): LC_Process(); LC.CycleCount++; break;
        case LA_PROP_STATUS_TLM_MID:
            memcpy(&LC.CachedProp, SBBufPtr, sizeof(LA_PROP_StatusPkt_t)); break;
        case LA_EPS_STATUS_TLM_MID:
            memcpy(&LC.CachedEPS, SBBufPtr, sizeof(LA_EPS_StatusPkt_t)); break;
        case LA_LSS_STATUS_TLM_MID:
            memcpy(&LC.CachedLSS, SBBufPtr, sizeof(LA_LSS_StatusPkt_t)); break;
        case LA_NAV_STATE_TLM_MID:
            memcpy(&LC.CachedNav, SBBufPtr, sizeof(LA_NAV_StatePkt_t)); break;
        case LA_LC_CMD_MID: LC.CmdCount++; break;
        default: break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
