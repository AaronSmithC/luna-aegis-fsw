/**
 * @file la_to_app.c
 * @brief Luna-Aegis Telemetry Output (TO) Application
 *
 * Subscribes to all telemetry on SB, filters/throttles per
 * output table, packetizes for downlink via COMM subsystem.
 * In simulation: sends to UDP for ground console.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include <string.h>

#define TO_PIPE_DEPTH  128
#define TO_PIPE_NAME   "TO_PIPE"
#define TO_UDP_PORT    1235

/* Output filter entry */
typedef struct {
    CFE_SB_MsgId_t MsgId;
    uint8_t        Decimation;  /* Send every Nth packet */
    uint8_t        Counter;
} TO_FilterEntry_t;

#define TO_MAX_FILTERS  40

typedef struct {
    CFE_SB_PipeId_t CmdPipe;
    uint32_t         RunStatus;

    TO_FilterEntry_t Filters[TO_MAX_FILTERS];
    uint16_t         FilterCount;

    uint32_t PktsSent;
    uint32_t PktsDropped;
    uint32_t CycleCount;
} TO_AppData_t;

static TO_AppData_t TO;

static void TO_AddFilter(CFE_SB_MsgId_t mid, uint8_t decim)
{
    if (TO.FilterCount >= TO_MAX_FILTERS) return;
    TO.Filters[TO.FilterCount].MsgId      = mid;
    TO.Filters[TO.FilterCount].Decimation  = decim;
    TO.Filters[TO.FilterCount].Counter     = 0;
    TO.FilterCount++;
    CFE_SB_Subscribe(mid, TO.CmdPipe);
}

static CFE_Status_t TO_Init(void)
{
    memset(&TO, 0, sizeof(TO));
    TO.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&TO.CmdPipe, TO_PIPE_DEPTH, TO_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_TO_CMD_MID),            TO.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x10)),  TO.CmdPipe);

    /* Subscribe to all telemetry with decimation filters */
    /* 40 Hz → decimate to 1 Hz for downlink */
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_NAV_STATE_TLM_MID),     40);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_IMU_DATA_TLM_MID),      40);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_ACS_ACTUATOR_CMD_MID),  40);

    /* 10 Hz → decimate to 1 Hz */
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_ALT_DATA_TLM_MID),      10);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_GDN_CMD_TLM_MID),       10);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_PROP_STATUS_TLM_MID),   10);

    /* 1-2 Hz: send every packet */
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_TRN_HAZARD_TLM_MID),     1);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),       1);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_LG_STATUS_TLM_MID),      1);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_DOCK_STATUS_TLM_MID),    1);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_EPS_STATUS_TLM_MID),     1);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_LSS_STATUS_TLM_MID),     1);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_COMM_LINK_TLM_MID),      1);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_LUNET_BEACON_TLM_MID),   1);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_HS_ALERT_TLM_MID),       1);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_LC_ACTION_TLM_MID),      1);
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_HK_COMBINED_TLM_MID),    1);

    /* 0.1 Hz: send every packet */
    TO_AddFilter(CFE_SB_ValueToMsgId(LA_TCS_STATUS_TLM_MID),     1);

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "TO: Telemetry output initialized — %u filters, UDP port %d",
                      TO.FilterCount, TO_UDP_PORT);
    return CFE_SUCCESS;
}

static void TO_OutputPacket(const CFE_MSG_Message_t *MsgPtr)
{
    /*
     * In simulation: send raw CCSDS packet to UDP for ground console.
     * sendto(sock, MsgPtr, pkt_len, 0, &ground_addr, sizeof(ground_addr));
     *
     * In flight: queue to COMM downlink buffer.
     */
    TO.PktsSent++;
    (void)MsgPtr; /* Suppress unused warning in stub */
}

void TO_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (TO_Init() != CFE_SUCCESS) { TO.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&TO.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, TO.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

        if (CFE_SB_MsgIdToValue(MsgId) == LA_TO_CMD_MID) { TO.CycleCount++; continue; }
        if (CFE_SB_MsgIdToValue(MsgId) == LA_SCH_WAKEUP_MID(0x10)) { TO.CycleCount++; continue; }

        /* Check decimation filter */
        for (uint16_t i = 0; i < TO.FilterCount; i++) {
            if (CFE_SB_MsgIdToValue(TO.Filters[i].MsgId) == CFE_SB_MsgIdToValue(MsgId)) {
                TO.Filters[i].Counter++;
                if (TO.Filters[i].Counter >= TO.Filters[i].Decimation) {
                    TO.Filters[i].Counter = 0;
                    TO_OutputPacket(&SBBufPtr->Msg);
                } else {
                    TO.PktsDropped++;
                }
                break;
            }
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
