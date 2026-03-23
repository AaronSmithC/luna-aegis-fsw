/**
 * @file la_eps_app.c
 * @brief Luna-Aegis Electrical Power System (EPS) Application
 *
 * Battery SOC monitoring, bus voltage management, load shedding,
 * and passive solar backup tracking.
 *
 * Subscribes: SCH Wakeup (1 Hz), EPS_CMD
 * Publishes:  EPS_StatusPkt (1 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include "../../hal/inc/la_hal.h"
#include <string.h>

#define EPS_PIPE_DEPTH  16
#define EPS_PIPE_NAME   "EPS_PIPE"

/* Thresholds */
#define EPS_SOC_WARN_PCT    30.0
#define EPS_SOC_CRIT_PCT    15.0
#define EPS_BUS_LOW_V       24.0
#define EPS_BUS_HIGH_V      32.0
#define EPS_TEMP_WARN_C     45.0

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;
    uint8_t          CurrentShedLevel;  /* 0=none, 1=non-essential, 2=critical */

    LA_EPS_StatusPkt_t StatusPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} EPS_AppData_t;

static EPS_AppData_t EPS;

static CFE_Status_t EPS_Init(void)
{
    memset(&EPS, 0, sizeof(EPS));
    EPS.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&EPS.CmdPipe, EPS_PIPE_DEPTH, EPS_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_EPS_CMD_MID),           EPS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x09)),  EPS.CmdPipe);

    CFE_MSG_Init(&EPS.StatusPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_EPS_STATUS_TLM_MID), sizeof(LA_EPS_StatusPkt_t));

    HAL_BMU_Init();

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "EPS: Power system initialized");
    return CFE_SUCCESS;
}

static void EPS_Process(void)
{
    HAL_BMU_Tlm_t bmu;
    HAL_BMU_Read(&bmu);

    /* Load shed logic */
    uint8_t new_shed = 0;
    if (bmu.soc_pct < EPS_SOC_CRIT_PCT || bmu.bus_voltage_V < EPS_BUS_LOW_V) {
        new_shed = 2;
    } else if (bmu.soc_pct < EPS_SOC_WARN_PCT) {
        new_shed = 1;
    }

    if (new_shed != EPS.CurrentShedLevel) {
        EPS.CurrentShedLevel = new_shed;
        HAL_BMU_SetLoadShed(new_shed);
        if (new_shed > 0) {
            CFE_EVS_SendEvent(10, new_shed == 2 ?
                              CFE_EVS_EventType_CRITICAL : CFE_EVS_EventType_ERROR,
                              "EPS: Load shed level %u — SOC %.1f%% Bus %.1fV",
                              new_shed, bmu.soc_pct, bmu.bus_voltage_V);
        }
    }

    /* Battery health */
    uint8_t health = 0;
    if (bmu.fault_flags || bmu.pack_temp_C > EPS_TEMP_WARN_C) {
        health = (bmu.fault_flags) ? 2 : 1;
    }

    EPS.StatusPkt.BusVoltage_V   = bmu.bus_voltage_V;
    EPS.StatusPkt.BusCurrent_A   = bmu.bus_current_A;
    EPS.StatusPkt.BattSOC_pct    = bmu.soc_pct;
    EPS.StatusPkt.BattTemp_C     = bmu.pack_temp_C;
    EPS.StatusPkt.SolarInput_W   = 0.0; /* Passive backup — context-dependent */
    EPS.StatusPkt.LoadShedLevel  = EPS.CurrentShedLevel;
    EPS.StatusPkt.BattHealth     = health;

    CFE_SB_TransmitMsg(&EPS.StatusPkt.TlmHdr.Msg, true);
}

void EPS_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (EPS_Init() != CFE_SUCCESS) { EPS.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&EPS.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, EPS.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x09): EPS_Process(); EPS.CycleCount++; break;
        case LA_EPS_CMD_MID: EPS.CmdCount++; break;
        default: break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
