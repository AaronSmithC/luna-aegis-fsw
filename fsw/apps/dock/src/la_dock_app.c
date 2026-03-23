/**
 * @file la_dock_app.c
 * @brief Luna-Aegis Docking Manager (DOCK) Application
 *
 * Soft-seal telescoping collar control, pressure equalization,
 * mate/demate sequencing for Aegis Station, surface habs, and
 * Aegis-Class Rover suitport.
 *
 * Subscribes: MM_PhasePkt (1 Hz), NAV_StatePkt, SCH Wakeup (1 Hz)
 * Publishes:  DOCK_StatusPkt (1 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include "../../hal/inc/la_hal.h"
#include <string.h>
#include <math.h>

#define DOCK_PIPE_DEPTH  16
#define DOCK_PIPE_NAME   "DOCK_PIPE"

#define DOCK_NOOP_CC      0
#define DOCK_RESET_CC     1
#define DOCK_MATE_CC      2
#define DOCK_DEMATE_CC    3
#define DOCK_HARD_CC      4

/* Pressure equalization tolerance (kPa) */
#define DOCK_PRESS_EQ_TOL   0.5

/* Docking FSM */
typedef enum {
    DOCK_FSM_IDLE       = 0,
    DOCK_FSM_EXTENDING  = 1,
    DOCK_FSM_SOFT       = 2,
    DOCK_FSM_PRESS_EQ   = 3,
    DOCK_FSM_HARD       = 4,
    DOCK_FSM_RELEASING  = 5,
} DOCK_FSM_State_t;

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    DOCK_FSM_State_t FSM;
    uint32_t         FSMTimer_s;

    LA_MM_PhasePkt_t LastPhase;
    bool             PhaseValid;

    LA_DOCK_StatusPkt_t StatusPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} DOCK_AppData_t;

static DOCK_AppData_t DOCK;

static CFE_Status_t DOCK_Init(void)
{
    memset(&DOCK, 0, sizeof(DOCK));
    DOCK.RunStatus = CFE_ES_RunStatus_APP_RUN;
    DOCK.FSM       = DOCK_FSM_IDLE;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&DOCK.CmdPipe, DOCK_PIPE_DEPTH, DOCK_PIPE_NAME);

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_DOCK_CMD_MID),          DOCK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),      DOCK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x0E)),  DOCK.CmdPipe);

    CFE_MSG_Init(&DOCK.StatusPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_DOCK_STATUS_TLM_MID), sizeof(LA_DOCK_StatusPkt_t));

    HAL_Dock_Init();

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "DOCK: Docking manager initialized — IDLE");
    return CFE_SUCCESS;
}

static void DOCK_UpdateFSM(void)
{
    HAL_Dock_Tlm_t hw;
    HAL_Dock_Read(&hw);

    DOCK.FSMTimer_s++;

    switch (DOCK.FSM) {
    case DOCK_FSM_IDLE:
        /* Nothing to do */
        break;

    case DOCK_FSM_EXTENDING:
        /* Wait for collar to reach soft-dock */
        if (hw.state == HAL_DOCK_SOFT_DOCK || DOCK.FSMTimer_s > 30) {
            DOCK.FSM = DOCK_FSM_SOFT;
            DOCK.FSMTimer_s = 0;
            CFE_EVS_SendEvent(10, CFE_EVS_EventType_INFORMATION,
                              "DOCK: Soft-dock achieved");
        }
        break;

    case DOCK_FSM_SOFT:
        /* Await command for pressure equalization */
        break;

    case DOCK_FSM_PRESS_EQ:
        /* Monitor pressure differential */
        if (hw.delta_p_kPa < DOCK_PRESS_EQ_TOL) {
            DOCK.FSM = DOCK_FSM_HARD;
            DOCK.FSMTimer_s = 0;
            HAL_Dock_HardLatch();
            CFE_EVS_SendEvent(11, CFE_EVS_EventType_INFORMATION,
                              "DOCK: Pressure equalized — hard-dock engaged");
        } else if (DOCK.FSMTimer_s > 120) {
            CFE_EVS_SendEvent(12, CFE_EVS_EventType_ERROR,
                              "DOCK: Pressure eq timeout — dP=%.2f kPa",
                              hw.delta_p_kPa);
        }
        break;

    case DOCK_FSM_HARD:
        /* Mated — monitor seal */
        if (hw.delta_p_kPa > DOCK_PRESS_EQ_TOL * 3.0) {
            CFE_EVS_SendEvent(13, CFE_EVS_EventType_CRITICAL,
                              "DOCK: SEAL LEAK — dP=%.2f kPa", hw.delta_p_kPa);
        }
        break;

    case DOCK_FSM_RELEASING:
        if (hw.state == HAL_DOCK_RETRACTED || DOCK.FSMTimer_s > 30) {
            DOCK.FSM = DOCK_FSM_IDLE;
            DOCK.FSMTimer_s = 0;
            CFE_EVS_SendEvent(14, CFE_EVS_EventType_INFORMATION,
                              "DOCK: Demate complete — IDLE");
        }
        break;
    }

    /* Update status packet */
    DOCK.StatusPkt.CollarState   = hw.state;
    DOCK.StatusPkt.SealPressOK   = (hw.delta_p_kPa < DOCK_PRESS_EQ_TOL) ? 1 : 0;
    DOCK.StatusPkt.LatchState    = hw.latch_engaged > 0 ? 2 : 0;
    DOCK.StatusPkt.TunnelPress_kPa = hw.tunnel_press_kPa;
    DOCK.StatusPkt.DeltaP_kPa     = hw.delta_p_kPa;
    DOCK.StatusPkt.MateTarget    = 0; /* Set by command */

    CFE_SB_TransmitMsg(&DOCK.StatusPkt.TlmHdr.Msg, true);
}

static void DOCK_ProcessCommand(const CFE_MSG_Message_t *MsgPtr)
{
    CFE_MSG_FcnCode_t fc;
    CFE_MSG_GetFcnCode(MsgPtr, &fc);

    switch (fc) {
    case DOCK_NOOP_CC:  DOCK.CmdCount++; break;
    case DOCK_RESET_CC: DOCK.CmdCount = 0; DOCK.ErrCount = 0; break;
    case DOCK_MATE_CC:
        if (DOCK.FSM == DOCK_FSM_IDLE) {
            HAL_Dock_Extend();
            DOCK.FSM = DOCK_FSM_EXTENDING;
            DOCK.FSMTimer_s = 0;
            DOCK.CmdCount++;
            CFE_EVS_SendEvent(20, CFE_EVS_EventType_INFORMATION,
                              "DOCK: Mate sequence initiated");
        } else { DOCK.ErrCount++; }
        break;
    case DOCK_DEMATE_CC:
        if (DOCK.FSM == DOCK_FSM_HARD || DOCK.FSM == DOCK_FSM_SOFT) {
            HAL_Dock_Release();
            HAL_Dock_Retract();
            DOCK.FSM = DOCK_FSM_RELEASING;
            DOCK.FSMTimer_s = 0;
            DOCK.CmdCount++;
            CFE_EVS_SendEvent(21, CFE_EVS_EventType_INFORMATION,
                              "DOCK: Demate sequence initiated");
        } else { DOCK.ErrCount++; }
        break;
    case DOCK_HARD_CC:
        if (DOCK.FSM == DOCK_FSM_SOFT) {
            DOCK.FSM = DOCK_FSM_PRESS_EQ;
            DOCK.FSMTimer_s = 0;
            DOCK.CmdCount++;
        } else { DOCK.ErrCount++; }
        break;
    default:
        DOCK.ErrCount++;
        break;
    }
}

void DOCK_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (DOCK_Init() != CFE_SUCCESS) { DOCK.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&DOCK.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, DOCK.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x0E): DOCK_UpdateFSM(); DOCK.CycleCount++; break;
        case LA_MM_PHASE_TLM_MID:
            memcpy(&DOCK.LastPhase, SBBufPtr, sizeof(LA_MM_PhasePkt_t));
            DOCK.PhaseValid = true; break;
        case LA_DOCK_CMD_MID:
            DOCK_ProcessCommand(&SBBufPtr->Msg); break;
        default: break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
