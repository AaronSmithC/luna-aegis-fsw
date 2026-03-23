/**
 * @file la_comm_app.c
 * @brief Luna-Aegis Communications Manager (COMM) Application
 *
 * S-band/UHF link management, HGA pointing, link budget monitoring.
 *
 * Subscribes: SCH Wakeup (1 Hz), COMM_CMD
 * Publishes:  COMM_LinkPkt (1 Hz)
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

#define COMM_PIPE_DEPTH  16
#define COMM_PIPE_NAME   "COMM_PIPE"

/* Link thresholds */
#define COMM_SNR_MIN_DB      6.0   /* Minimum usable SNR */
#define COMM_SNR_GOOD_DB    12.0

/* HGA pointing — track Aegis Station (simplified: fixed angles) */
#define COMM_HGA_DEFAULT_AZ  45.0
#define COMM_HGA_DEFAULT_EL  30.0

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    uint8_t  LinkState;    /* 0=no link, 1=UHF, 2=S-band, 3=HGA */
    uint8_t  DataRate;     /* 0=low, 1=med, 2=high */
    double   HGA_Az_cmd;
    double   HGA_El_cmd;

    LA_COMM_LinkPkt_t LinkPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} COMM_AppData_t;

static COMM_AppData_t COMM;

static CFE_Status_t COMM_Init(void)
{
    memset(&COMM, 0, sizeof(COMM));
    COMM.RunStatus = CFE_ES_RunStatus_APP_RUN;
    COMM.LinkState  = 2;  /* S-band default */
    COMM.HGA_Az_cmd = COMM_HGA_DEFAULT_AZ;
    COMM.HGA_El_cmd = COMM_HGA_DEFAULT_EL;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&COMM.CmdPipe, COMM_PIPE_DEPTH, COMM_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_COMM_CMD_MID),          COMM.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x0B)),  COMM.CmdPipe);

    CFE_MSG_Init(&COMM.LinkPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_COMM_LINK_TLM_MID), sizeof(LA_COMM_LinkPkt_t));

    HAL_Antenna_Init();
    HAL_Antenna_Point(COMM.HGA_Az_cmd, COMM.HGA_El_cmd);

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "COMM: Comms manager initialized — S-band active");
    return CFE_SUCCESS;
}

static void COMM_Process(void)
{
    /* Read antenna pointing */
    double az_act, el_act;
    HAL_Antenna_GetPointing(&az_act, &el_act);

    /* Simulate link quality based on pointing error */
    double az_err = fabs(az_act - COMM.HGA_Az_cmd);
    double el_err = fabs(el_act - COMM.HGA_El_cmd);
    double point_err = sqrt(az_err * az_err + el_err * el_err);

    /* SNR model: base SNR degraded by pointing error */
    double base_snr = 20.0; /* dB — good conditions */
    double snr_penalty = point_err * 2.0; /* 2 dB per degree error */
    double snr = base_snr - snr_penalty;

    /* Determine link state and data rate */
    if (snr >= COMM_SNR_GOOD_DB) {
        COMM.LinkState = 3; /* HGA */
        COMM.DataRate  = 2; /* high */
    } else if (snr >= COMM_SNR_MIN_DB) {
        COMM.LinkState = 2; /* S-band */
        COMM.DataRate  = 1; /* med */
    } else if (snr > 0.0) {
        COMM.LinkState = 1; /* UHF fallback */
        COMM.DataRate  = 0; /* low */
    } else {
        COMM.LinkState = 0; /* no link */
        COMM.DataRate  = 0;
        CFE_EVS_SendEvent(10, CFE_EVS_EventType_ERROR,
                          "COMM: Link lost — SNR %.1f dB", snr);
    }

    /* Update HGA pointing command */
    HAL_Antenna_Point(COMM.HGA_Az_cmd, COMM.HGA_El_cmd);

    COMM.LinkPkt.UplinkSNR_dB    = snr - 1.0; /* Uplink slightly worse */
    COMM.LinkPkt.DownlinkSNR_dB  = snr;
    COMM.LinkPkt.HGA_Az_deg      = az_act;
    COMM.LinkPkt.HGA_El_deg      = el_act;
    COMM.LinkPkt.LinkState        = COMM.LinkState;
    COMM.LinkPkt.DataRate         = COMM.DataRate;

    CFE_SB_TransmitMsg(&COMM.LinkPkt.TlmHdr.Msg, true);
}

void COMM_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (COMM_Init() != CFE_SUCCESS) { COMM.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&COMM.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, COMM.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x0B): COMM_Process(); COMM.CycleCount++; break;
        case LA_COMM_CMD_MID: COMM.CmdCount++; break;
        default: break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
