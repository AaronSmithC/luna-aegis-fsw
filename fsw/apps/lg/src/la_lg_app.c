/**
 * @file la_lg_app.c
 * @brief Luna-Aegis Landing Gear Manager (LG) Application
 *
 * Deploy/retract sequencing, load monitoring, touchdown detection.
 *
 * Subscribes: MM_PhasePkt (1 Hz), SCH Wakeup (1 Hz), LG_CMD
 * Publishes:  LG_StatusPkt (1 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include "../../hal/inc/la_hal.h"
#include <string.h>

#define LG_PIPE_DEPTH   16
#define LG_PIPE_NAME    "LG_PIPE"

#define LG_NOOP_CC      0
#define LG_RESET_CC     1
#define LG_DEPLOY_CC    2
#define LG_RETRACT_CC   3

/* Load limits */
#define LG_LOAD_WARN_N    8000.0
#define LG_LOAD_CRIT_N   12000.0

/* Touchdown: 3 of 4 legs must have contact */
#define LG_TD_THRESHOLD   3

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    LA_MM_PhasePkt_t LastPhase;
    bool             PhaseValid;
    bool             AutoDeployDone;

    LA_LG_StatusPkt_t StatusPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} LG_AppData_t;

static LG_AppData_t LG;

static CFE_Status_t LG_Init(void)
{
    memset(&LG, 0, sizeof(LG));
    LG.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&LG.CmdPipe, LG_PIPE_DEPTH, LG_PIPE_NAME);

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LG_CMD_MID),           LG.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),     LG.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x0D)), LG.CmdPipe);

    CFE_MSG_Init(&LG.StatusPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_LG_STATUS_TLM_MID), sizeof(LA_LG_StatusPkt_t));

    HAL_LG_Init();

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "LG: Landing gear manager initialized");
    return CFE_SUCCESS;
}

static void LG_Process(void)
{
    /* Read HAL */
    HAL_LG_Tlm_t hw;
    HAL_LG_Read(&hw);

    LG.StatusPkt.DeployState = hw.state;
    LG.StatusPkt.LoadLeg1_N  = hw.load_N[0];
    LG.StatusPkt.LoadLeg2_N  = hw.load_N[1];
    LG.StatusPkt.LoadLeg3_N  = hw.load_N[2];
    LG.StatusPkt.LoadLeg4_N  = hw.load_N[3];

    /* Per-leg health */
    uint8_t td_mask = 0;
    for (int i = 0; i < 4; i++) {
        if (hw.load_N[i] > LG_LOAD_CRIT_N) {
            LG.StatusPkt.LegStatus[i] = 2; /* fault */
            CFE_EVS_SendEvent(10 + i, CFE_EVS_EventType_CRITICAL,
                              "LG: Leg %d OVERLOAD %.0f N", i+1, hw.load_N[i]);
        } else if (hw.load_N[i] > LG_LOAD_WARN_N) {
            LG.StatusPkt.LegStatus[i] = 1; /* warn */
        } else {
            LG.StatusPkt.LegStatus[i] = 0; /* OK */
        }
        if (hw.touchdown[i]) td_mask |= (1 << i);
    }
    LG.StatusPkt.TouchdownDetect = td_mask;

    /* Auto-deploy disabled — MM commands deploy based on mission type
     * (surface missions get explicit deploy command, docking missions don't) */

    CFE_SB_TransmitMsg(&LG.StatusPkt.TlmHdr.Msg, true);
}

static void LG_ProcessCommand(const CFE_MSG_Message_t *MsgPtr)
{
    CFE_MSG_FcnCode_t fc;
    CFE_MSG_GetFcnCode(MsgPtr, &fc);

    switch (fc) {
    case LG_NOOP_CC:   LG.CmdCount++; break;
    case LG_RESET_CC:  LG.CmdCount = 0; LG.ErrCount = 0; break;
    case LG_DEPLOY_CC:
        HAL_LG_Deploy();
        LG.CmdCount++;
        CFE_EVS_SendEvent(21, CFE_EVS_EventType_INFORMATION, "LG: Manual deploy");
        break;
    case LG_RETRACT_CC:
        HAL_LG_Retract();
        LG.CmdCount++;
        CFE_EVS_SendEvent(22, CFE_EVS_EventType_INFORMATION, "LG: Manual retract");
        break;
    default:
        LG.ErrCount++;
        break;
    }
}

void LG_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (LG_Init() != CFE_SUCCESS) { LG.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&LG.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, LG.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x0D): LG_Process(); LG.CycleCount++; break;
        case LA_MM_PHASE_TLM_MID:
            memcpy(&LG.LastPhase, SBBufPtr, sizeof(LA_MM_PhasePkt_t));
            LG.PhaseValid = true; break;
        case LA_LG_CMD_MID:
            LG_ProcessCommand(&SBBufPtr->Msg); break;
        default: break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
