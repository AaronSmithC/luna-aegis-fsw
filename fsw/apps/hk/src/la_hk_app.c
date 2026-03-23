/**
 * @file la_hk_app.c
 * @brief Luna-Aegis Housekeeping (HK) Application
 *
 * Aggregates housekeeping telemetry from all apps into combined
 * packets for efficient downlink.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include <string.h>

#define HK_PIPE_DEPTH      48
#define HK_PIPE_NAME   "HK_PIPE"

/* Combined HK packet — summary of all app states */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHdr;
    uint32_t AppAliveFlags;     /* Bitfield: received HK from each app */
    uint32_t TotalCmdCount;
    uint32_t TotalErrCount;
    uint32_t UptimeSeconds;
} LA_HK_CombinedPkt_t;

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    uint32_t AppAliveFlags;
    uint32_t UptimeSeconds;

    LA_HK_CombinedPkt_t CombinedPkt;

    uint32_t CycleCount;
} HK_AppData_t;

static HK_AppData_t HK;

static CFE_Status_t HK_Init(void)
{
    memset(&HK, 0, sizeof(HK));
    HK.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&HK.CmdPipe, HK_PIPE_DEPTH, HK_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_HK_CMD_MID),            HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x0F)),  HK.CmdPipe);

    /* Subscribe to all app HK packets */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_IMU_MGR_HK_TLM_MID), HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_ALT_MGR_HK_TLM_MID), HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_NAV_HK_TLM_MID),     HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_GDN_HK_TLM_MID),     HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_ACS_HK_TLM_MID),     HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_TRN_HK_TLM_MID),     HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_HK_TLM_MID),      HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_PROP_HK_TLM_MID),    HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LG_HK_TLM_MID),      HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_DOCK_HK_TLM_MID),    HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_EPS_HK_TLM_MID),     HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_TCS_HK_TLM_MID),     HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LSS_HK_TLM_MID),     HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_COMM_HK_TLM_MID),    HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LUNET_HK_TLM_MID),   HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_HS_HK_TLM_MID),      HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LC_HK_TLM_MID),      HK.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SC_HK_TLM_MID),      HK.CmdPipe);

    CFE_MSG_Init(&HK.CombinedPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_HK_COMBINED_TLM_MID), sizeof(LA_HK_CombinedPkt_t));

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "HK: Housekeeping initialized — 18 app subscriptions");
    return CFE_SUCCESS;
}

static void HK_Process(void)
{
    HK.UptimeSeconds++;

    HK.CombinedPkt.AppAliveFlags  = HK.AppAliveFlags;
    HK.CombinedPkt.UptimeSeconds  = HK.UptimeSeconds;

    CFE_SB_TransmitMsg(&HK.CombinedPkt.TlmHdr.Msg, true);

    /* Reset alive flags for next cycle */
    HK.AppAliveFlags = 0;
}

void HK_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (HK_Init() != CFE_SUCCESS) { HK.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&HK.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, HK.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x0F): HK_Process(); HK.CycleCount++; break;
        /* Mark apps alive when their HK arrives */
        case LA_IMU_MGR_HK_TLM_MID: HK.AppAliveFlags |= (1 << 0); break;
        case LA_ALT_MGR_HK_TLM_MID: HK.AppAliveFlags |= (1 << 1); break;
        case LA_NAV_HK_TLM_MID:     HK.AppAliveFlags |= (1 << 2); break;
        case LA_GDN_HK_TLM_MID:     HK.AppAliveFlags |= (1 << 3); break;
        case LA_ACS_HK_TLM_MID:     HK.AppAliveFlags |= (1 << 4); break;
        case LA_TRN_HK_TLM_MID:     HK.AppAliveFlags |= (1 << 5); break;
        case LA_MM_HK_TLM_MID:      HK.AppAliveFlags |= (1 << 6); break;
        case LA_PROP_HK_TLM_MID:    HK.AppAliveFlags |= (1 << 7); break;
        case LA_LG_HK_TLM_MID:      HK.AppAliveFlags |= (1 << 8); break;
        case LA_DOCK_HK_TLM_MID:    HK.AppAliveFlags |= (1 << 9); break;
        case LA_EPS_HK_TLM_MID:     HK.AppAliveFlags |= (1 << 10); break;
        case LA_TCS_HK_TLM_MID:     HK.AppAliveFlags |= (1 << 11); break;
        case LA_LSS_HK_TLM_MID:     HK.AppAliveFlags |= (1 << 12); break;
        case LA_COMM_HK_TLM_MID:    HK.AppAliveFlags |= (1 << 13); break;
        case LA_LUNET_HK_TLM_MID:   HK.AppAliveFlags |= (1 << 14); break;
        case LA_HS_HK_TLM_MID:      HK.AppAliveFlags |= (1 << 15); break;
        case LA_LC_HK_TLM_MID:      HK.AppAliveFlags |= (1 << 16); break;
        case LA_SC_HK_TLM_MID:      HK.AppAliveFlags |= (1 << 17); break;
        default: break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
