/**
 * @file la_lss_app.c
 * @brief Luna-Aegis Life Support System (LSS) Application
 *
 * Cabin pressure regulation, O2/CO2 management, humidity control,
 * consumables tracking.  Orion-class LSS heritage.
 *
 * Subscribes: SCH Wakeup (1 Hz), MM_PhasePkt, LSS_CMD
 * Publishes:  LSS_StatusPkt (1 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include <string.h>

#define LSS_PIPE_DEPTH  16
#define LSS_PIPE_NAME   "LSS_PIPE"

/* Atmospheric limits */
#define LSS_CABIN_PRESS_NOM_KPA   70.3   /* ~10.2 psi, low-pressure ops */
#define LSS_O2_PARTIAL_NOM_KPA    21.3
#define LSS_CO2_LIMIT_KPA         0.53   /* 4 mmHg SMAC limit */
#define LSS_HUMIDITY_LOW_PCT      25.0
#define LSS_HUMIDITY_HIGH_PCT     75.0

/* Consumables model */
#define LSS_O2_RATE_KG_HR_PER_CREW  0.84  /* ~0.84 kg/day → /24 */
#define LSS_CO2_RATE_KG_HR_PER_CREW 1.0   /* ~1.0 kg/day → /24 */
#define LSS_O2_RESERVE_KG           50.0
#define LSS_SCRUBBER_CAPACITY_KG    20.0

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    uint8_t  CrewCount;
    uint8_t  LSSMode;         /* 0=nominal, 1=low-power, 2=emergency */
    double   O2_Remaining_kg;
    double   ScrubberUsed_kg;

    /* Simulated atmosphere */
    double CabinPress_kPa;
    double O2_Partial_kPa;
    double CO2_Partial_kPa;
    double Humidity_pct;
    double CabinTemp_C;

    LA_LSS_StatusPkt_t StatusPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} LSS_AppData_t;

static LSS_AppData_t LSS;

static CFE_Status_t LSS_Init(void)
{
    memset(&LSS, 0, sizeof(LSS));
    LSS.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /* Initial: nominal atmosphere, 4 crew, full consumables */
    LSS.CrewCount       = 4;
    LSS.LSSMode         = 0;
    LSS.O2_Remaining_kg = LSS_O2_RESERVE_KG;
    LSS.ScrubberUsed_kg = 0.0;
    LSS.CabinPress_kPa  = LSS_CABIN_PRESS_NOM_KPA;
    LSS.O2_Partial_kPa  = LSS_O2_PARTIAL_NOM_KPA;
    LSS.CO2_Partial_kPa = 0.1;
    LSS.Humidity_pct     = 45.0;
    LSS.CabinTemp_C      = 22.0;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&LSS.CmdPipe, LSS_PIPE_DEPTH, LSS_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LSS_CMD_MID),           LSS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),      LSS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x0A)),  LSS.CmdPipe);

    CFE_MSG_Init(&LSS.StatusPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_LSS_STATUS_TLM_MID), sizeof(LA_LSS_StatusPkt_t));

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "LSS: Life support initialized — %u crew, nominal",
                      LSS.CrewCount);
    return CFE_SUCCESS;
}

static void LSS_Process(void)
{
    double dt_hr = 1.0 / 3600.0; /* 1 second in hours */

    if (LSS.CrewCount > 0) {
        /* O2 consumption */
        double o2_used = LSS_O2_RATE_KG_HR_PER_CREW * LSS.CrewCount * dt_hr;
        LSS.O2_Remaining_kg -= o2_used;
        if (LSS.O2_Remaining_kg < 0.0) LSS.O2_Remaining_kg = 0.0;

        /* CO2 production */
        double co2_prod = LSS_CO2_RATE_KG_HR_PER_CREW * LSS.CrewCount * dt_hr;
        LSS.CO2_Partial_kPa += co2_prod * 0.01; /* Simplified conversion */

        /* Scrubber removes CO2 */
        double scrub_rate = 0.02; /* kPa/s scrub rate */
        double scrubbed = scrub_rate * 1.0; /* per second */
        if (LSS.CO2_Partial_kPa > 0.05) {
            LSS.CO2_Partial_kPa -= scrubbed;
            LSS.ScrubberUsed_kg += scrubbed * 0.1;
        }
        if (LSS.CO2_Partial_kPa < 0.0) LSS.CO2_Partial_kPa = 0.0;
    }

    /* CO2 alarm */
    if (LSS.CO2_Partial_kPa > LSS_CO2_LIMIT_KPA) {
        CFE_EVS_SendEvent(10, CFE_EVS_EventType_CRITICAL,
                          "LSS: CO2 ALARM — ppCO2=%.2f kPa > limit %.2f",
                          LSS.CO2_Partial_kPa, LSS_CO2_LIMIT_KPA);
        if (LSS.LSSMode < 2) LSS.LSSMode = 2;
    }

    /* O2 depletion warning */
    double o2_hours = 0.0;
    if (LSS.CrewCount > 0) {
        o2_hours = LSS.O2_Remaining_kg /
                   (LSS_O2_RATE_KG_HR_PER_CREW * LSS.CrewCount);
    }
    if (o2_hours < 8.0 && o2_hours > 0.0) {
        CFE_EVS_SendEvent(11, CFE_EVS_EventType_ERROR,
                          "LSS: O2 warning — %.1f hours remaining", o2_hours);
    }

    /* Build telemetry */
    LSS.StatusPkt.CabinPress_kPa  = LSS.CabinPress_kPa;
    LSS.StatusPkt.O2_Partial_kPa  = LSS.O2_Partial_kPa;
    LSS.StatusPkt.CO2_Partial_kPa = LSS.CO2_Partial_kPa;
    LSS.StatusPkt.Humidity_pct    = LSS.Humidity_pct;
    LSS.StatusPkt.CabinTemp_C     = LSS.CabinTemp_C;
    LSS.StatusPkt.O2_Remaining_hr = o2_hours;
    LSS.StatusPkt.CO2_Scrub_pct   = (LSS.ScrubberUsed_kg < LSS_SCRUBBER_CAPACITY_KG)
                                    ? (1.0 - LSS.ScrubberUsed_kg / LSS_SCRUBBER_CAPACITY_KG) * 100.0
                                    : 0.0;
    LSS.StatusPkt.CrewCount       = LSS.CrewCount;
    LSS.StatusPkt.LSSMode         = LSS.LSSMode;

    CFE_SB_TransmitMsg(&LSS.StatusPkt.TlmHdr.Msg, true);
}

void LSS_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (LSS_Init() != CFE_SUCCESS) { LSS.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&LSS.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, LSS.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x0A): LSS_Process(); LSS.CycleCount++; break;
        case LA_LSS_CMD_MID: LSS.CmdCount++; break;
        default: break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
