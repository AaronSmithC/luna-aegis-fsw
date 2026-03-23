/**
 * @file la_tcs_app.c
 * @brief Luna-Aegis Thermal Control System (TCS) Application
 *
 * SSSRA (Stacked Solar-Shade Radiator Architecture) monitoring,
 * heater zone control, cryo boiloff tracking.  Runs at 0.1 Hz
 * (every 10th major frame) — thermal dynamics are slow.
 *
 * Subscribes: SCH Wakeup (0.1 Hz), TCS_CMD
 * Publishes:  TCS_StatusPkt (0.1 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include "../../hal/inc/la_hal.h"
#include <string.h>

#define TCS_PIPE_DEPTH  8
#define TCS_PIPE_NAME   "TCS_PIPE"

/* Thermal limits (Celsius) */
#define TCS_RAD_HOT_LIMIT     80.0
#define TCS_RAD_COLD_LIMIT   -120.0
#define TCS_CABIN_LOW          18.0
#define TCS_CABIN_HIGH         28.0
#define TCS_PROPLINE_FREEZE   -250.0  /* LH2 boiling: 20 K */

/* Heater zone assignments */
#define TCS_ZONE_CABIN      0
#define TCS_ZONE_AVIONICS   1
#define TCS_ZONE_PROPLINE   2
#define TCS_ZONE_DOCKING    3
#define TCS_ZONE_BATTERY    4

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    double ZoneTemps[HAL_HEATER_ZONE_COUNT];
    uint8_t ThermalMode;  /* 0=passive, 1=active, 2=survival */

    LA_TCS_StatusPkt_t StatusPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} TCS_AppData_t;

static TCS_AppData_t TCS;

static CFE_Status_t TCS_Init(void)
{
    memset(&TCS, 0, sizeof(TCS));
    TCS.RunStatus = CFE_ES_RunStatus_APP_RUN;
    TCS.ThermalMode = 1; /* active */

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&TCS.CmdPipe, TCS_PIPE_DEPTH, TCS_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_TCS_CMD_MID),           TCS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x13)),  TCS.CmdPipe);

    CFE_MSG_Init(&TCS.StatusPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_TCS_STATUS_TLM_MID), sizeof(LA_TCS_StatusPkt_t));

    HAL_Heater_Init();

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "TCS: Thermal control initialized — SSSRA active mode");
    return CFE_SUCCESS;
}

static void TCS_Process(void)
{
    HAL_Heater_GetTemps(TCS.ZoneTemps);

    uint8_t heater_mask = 0;

    /* Thermostat logic per zone */
    for (int z = 0; z < HAL_HEATER_ZONE_COUNT; z++) {
        bool need_heat = false;
        double setpoint_low = -40.0; /* default cold limit */

        switch (z) {
        case TCS_ZONE_CABIN:     setpoint_low = TCS_CABIN_LOW; break;
        case TCS_ZONE_AVIONICS:  setpoint_low = -10.0; break;
        case TCS_ZONE_PROPLINE:  setpoint_low = -200.0; break;
        case TCS_ZONE_BATTERY:   setpoint_low = 0.0; break;
        default:                 setpoint_low = -60.0; break;
        }

        if (TCS.ZoneTemps[z] < setpoint_low) {
            need_heat = true;
        }

        if (TCS.ThermalMode >= 1) {
            uint8_t duty = need_heat ? 80 : 0;
            HAL_Heater_SetZone(z, need_heat, duty);
            if (need_heat) heater_mask |= (1 << z);
        }
    }

    /* SSSRA radiator estimate (simplified: use zone 0-1 avg as proxy) */
    double rad_temp = (TCS.ZoneTemps[0] + TCS.ZoneTemps[1]) / 2.0 - 30.0;
    double pv_temp  = rad_temp + 40.0; /* PV array runs hotter */
    double cabin_temp = TCS.ZoneTemps[TCS_ZONE_CABIN];
    double propline_temp = TCS.ZoneTemps[TCS_ZONE_PROPLINE];

    /* Heat rejection estimate: Stefan-Boltzmann simplified */
    double T_K = rad_temp + 273.15;
    double rad_area = 4.0; /* m^2 SSSRA area */
    double epsilon = 0.85;
    double sigma = 5.67e-8;
    double heat_reject = epsilon * sigma * rad_area * T_K * T_K * T_K * T_K;

    TCS.StatusPkt.RadiatorTemp_C    = rad_temp;
    TCS.StatusPkt.SolarArrayTemp_C  = pv_temp;
    TCS.StatusPkt.CabinTemp_C       = cabin_temp;
    TCS.StatusPkt.PropLineTemp_C    = propline_temp;
    TCS.StatusPkt.HeatRejection_W   = heat_reject;
    TCS.StatusPkt.HeaterZoneMask    = heater_mask;
    TCS.StatusPkt.ThermalMode       = TCS.ThermalMode;

    CFE_SB_TransmitMsg(&TCS.StatusPkt.TlmHdr.Msg, true);
}

void TCS_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (TCS_Init() != CFE_SUCCESS) { TCS.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&TCS.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, TCS.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x13): TCS_Process(); TCS.CycleCount++; break;
        case LA_TCS_CMD_MID: TCS.CmdCount++; break;
        default: break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
