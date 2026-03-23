/**
 * @file la_alt_mgr_app.c
 * @brief Luna-Aegis Altimeter Manager (ALT_MGR) Application
 *
 * Reads lidar and radar altimeters via HAL, performs cross-check
 * and weighted fusion, publishes AGL + vertical rate at 10 Hz.
 *
 * Subscribes: SCH Wakeup (10 Hz), ALT_MGR_CMD
 * Publishes:  ALT_DataPkt (10 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include "../../hal/inc/la_hal.h"
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ALT_PIPE_DEPTH   16
#define ALT_PIPE_NAME    "ALT_PIPE"

/* Sensor noise (1-sigma) for weighted fusion */
#define ALT_LIDAR_SIGMA_M   0.05
#define ALT_RADAR_SIGMA_M   1.0
#define ALT_DISAGREE_THRESH  5.0  /* m — flag if sensors diverge */

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    HAL_Altimeter_Raw_t Lidar;
    HAL_Altimeter_Raw_t Radar;

    double PrevAGL_m;
    double DT;

    LA_ALT_DataPkt_t DataPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} ALT_AppData_t;

static ALT_AppData_t ALT;

static CFE_Status_t ALT_Init(void)
{
    memset(&ALT, 0, sizeof(ALT));
    ALT.RunStatus = CFE_ES_RunStatus_APP_RUN;
    ALT.DT = 0.1; /* 10 Hz */

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&ALT.CmdPipe, ALT_PIPE_DEPTH, ALT_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_ALT_MGR_CMD_MID),      ALT.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x04)), ALT.CmdPipe);

    CFE_MSG_Init(&ALT.DataPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_ALT_DATA_TLM_MID), sizeof(LA_ALT_DataPkt_t));

    HAL_Altimeter_Init();

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "ALT_MGR: Initialized — lidar + radar fusion @ 10 Hz");
    return CFE_SUCCESS;
}

static void ALT_Process(void)
{
    HAL_Status_t ls = HAL_Altimeter_ReadLidar(&ALT.Lidar);
    HAL_Status_t rs = HAL_Altimeter_ReadRadar(&ALT.Radar);

    bool lidar_ok = (ls == HAL_OK && ALT.Lidar.valid);
    bool radar_ok = (rs == HAL_OK && ALT.Radar.valid);

    ALT.DataPkt.LidarValid = lidar_ok ? 1 : 0;
    ALT.DataPkt.RadarValid = radar_ok ? 1 : 0;

    double fused_agl, fused_rate;

    if (lidar_ok && radar_ok) {
        /* Weighted fusion: inverse-variance */
        double w_l = 1.0 / (ALT_LIDAR_SIGMA_M * ALT_LIDAR_SIGMA_M);
        double w_r = 1.0 / (ALT_RADAR_SIGMA_M * ALT_RADAR_SIGMA_M);
        double w_total = w_l + w_r;

        fused_agl  = (ALT.Lidar.range_m * w_l + ALT.Radar.range_m * w_r) / w_total;
        fused_rate = (ALT.Lidar.rate_mps * w_l + ALT.Radar.rate_mps * w_r) / w_total;

        /* Cross-check */
        double disagree = fabs(ALT.Lidar.range_m - ALT.Radar.range_m);
        if (disagree > ALT_DISAGREE_THRESH) {
            CFE_EVS_SendEvent(10, CFE_EVS_EventType_ERROR,
                              "ALT_MGR: Lidar/radar disagree %.1f m — preferring lidar",
                              disagree);
            fused_agl  = ALT.Lidar.range_m;
            fused_rate = ALT.Lidar.rate_mps;
        }
    } else if (lidar_ok) {
        fused_agl  = ALT.Lidar.range_m;
        fused_rate = ALT.Lidar.rate_mps;
    } else if (radar_ok) {
        fused_agl  = ALT.Radar.range_m;
        fused_rate = ALT.Radar.rate_mps;
    } else {
        /* Both failed: hold last + differentiate */
        fused_agl  = ALT.PrevAGL_m;
        fused_rate = 0.0;
        CFE_EVS_SendEvent(11, CFE_EVS_EventType_CRITICAL,
                          "ALT_MGR: ALL ALTIMETERS FAILED");
    }

    /* Terrain slope estimate from rate of AGL change vs horizontal vel */
    double d_agl = fused_agl - ALT.PrevAGL_m;
    ALT.DataPkt.TerrainSlope_deg = atan2(d_agl, 10.0) * 180.0 / M_PI;

    ALT.DataPkt.AGL_m       = fused_agl;
    ALT.DataPkt.VertRate_mps = fused_rate;
    ALT.PrevAGL_m            = fused_agl;

    CFE_SB_TransmitMsg(&ALT.DataPkt.TlmHdr.Msg, true);
}

void ALT_MGR_AppMain(void)
{
    CFE_Status_t     status;
    CFE_SB_Buffer_t *SBBufPtr;
    CFE_SB_MsgId_t   MsgId;

    if (ALT_Init() != CFE_SUCCESS) { ALT.RunStatus = CFE_ES_RunStatus_APP_ERROR; }

    while (CFE_ES_RunLoop(&ALT.RunStatus) == true) {
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, ALT.CmdPipe, CFE_SB_PEND_FOREVER);
        if (status != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x04):
            ALT_Process();
            ALT.CycleCount++;
            break;
        case LA_ALT_MGR_CMD_MID:
            ALT.CmdCount++;
            break;
        default:
            break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
