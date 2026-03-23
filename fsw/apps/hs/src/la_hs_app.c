/**
 * @file la_hs_app.c
 * @brief Luna-Aegis Health & Safety (HS) Application
 *
 * Application watchdog monitoring, CPU/memory tracking,
 * redundancy state management, critical event response.
 *
 * Subscribes: All *_HkPacket (via HK combined), SCH Wakeup (1 Hz)
 * Publishes:  HS_AlertPkt (1 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include <string.h>

#define HS_PIPE_DEPTH  32
#define HS_PIPE_NAME   "HS_PIPE"

/* Watchdog timeout (cycles @ 1 Hz) */
#define HS_WD_TIMEOUT   5

/* CPU limits */
#define HS_CPU_WARN_PCT   80
#define HS_CPU_CRIT_PCT   95

/* Monitored apps: bitfield positions */
#define HS_APP_IMU    0
#define HS_APP_NAV    1
#define HS_APP_GDN    2
#define HS_APP_ACS    3
#define HS_APP_MM     4
#define HS_APP_PROP   5
#define HS_APP_EPS    6
#define HS_APP_COMM   7
#define HS_APP_COUNT  8

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    /* Watchdog counters — incremented by HK, checked by HS */
    uint32_t WD_Counter[HS_APP_COUNT];
    uint32_t WD_LastSeen[HS_APP_COUNT];
    uint16_t WD_AliveMask;

    /* System health */
    uint32_t CPULoad_x100;
    uint32_t MemFree;
    uint16_t CritEventCount;
    uint8_t  RedundState;   /* 0=primary, 1=backup, 2=split */
    uint8_t  RebootCount;

    LA_HS_AlertPkt_t AlertPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} HS_AppData_t;

static HS_AppData_t HS;

static CFE_Status_t HS_Init(void)
{
    memset(&HS, 0, sizeof(HS));
    HS.RunStatus = CFE_ES_RunStatus_APP_RUN;
    HS.RedundState  = 0;  /* primary */
    HS.MemFree      = 256 * 1024 * 1024; /* 256 MB sim */
    HS.WD_AliveMask = 0xFFFF; /* Assume all alive at start */

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&HS.CmdPipe, HS_PIPE_DEPTH, HS_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_HS_CMD_MID),            HS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x11)),  HS.CmdPipe);

    /* Subscribe to key app HK for watchdog */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_IMU_MGR_HK_TLM_MID), HS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_NAV_HK_TLM_MID),     HS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_HK_TLM_MID),      HS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_PROP_HK_TLM_MID),    HS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_EPS_HK_TLM_MID),     HS.CmdPipe);

    CFE_MSG_Init(&HS.AlertPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_HS_ALERT_TLM_MID), sizeof(LA_HS_AlertPkt_t));

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "HS: Health & safety initialized — watchdog active");
    return CFE_SUCCESS;
}

static void HS_CheckWatchdogs(void)
{
    HS.WD_AliveMask = 0;
    for (int i = 0; i < HS_APP_COUNT; i++) {
        if (HS.WD_Counter[i] != HS.WD_LastSeen[i]) {
            /* App is alive — counter changed */
            HS.WD_AliveMask |= (1 << i);
            HS.WD_LastSeen[i] = HS.WD_Counter[i];
        }
        /* Stale apps are reflected in WD_AliveMask = 0 for that bit.
         * No EVS event — the console reads WD_AliveMask from telemetry. */
    }
}

static void HS_Process(void)
{
    HS_CheckWatchdogs();

    /* Simulated CPU load (cycles-based in real impl) */
    HS.CPULoad_x100 = 3500 + (HS.CycleCount % 500); /* ~35% nominal */

    if (HS.CPULoad_x100 > HS_CPU_CRIT_PCT * 100) {
        CFE_EVS_SendEvent(20, CFE_EVS_EventType_CRITICAL,
                          "HS: CPU CRITICAL %u.%02u%%",
                          HS.CPULoad_x100/100, HS.CPULoad_x100%100);
        HS.CritEventCount++;
    }

    HS.AlertPkt.CPULoad_pct_x100  = HS.CPULoad_x100;
    HS.AlertPkt.MemFree_bytes     = HS.MemFree;
    HS.AlertPkt.AppWatchdogMask   = HS.WD_AliveMask;
    HS.AlertPkt.CritEventCount    = HS.CritEventCount;
    HS.AlertPkt.RedundState       = HS.RedundState;
    HS.AlertPkt.RebootCount       = HS.RebootCount;

    CFE_SB_TransmitMsg(&HS.AlertPkt.TlmHdr.Msg, true);
}

void HS_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (HS_Init() != CFE_SUCCESS) { HS.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&HS.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, HS.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x11): HS_Process(); HS.CycleCount++; break;
        /* Watchdog: increment counter when HK received */
        case LA_IMU_MGR_HK_TLM_MID:  HS.WD_Counter[HS_APP_IMU]++;  break;
        case LA_NAV_HK_TLM_MID:      HS.WD_Counter[HS_APP_NAV]++;  break;
        case LA_MM_HK_TLM_MID:       HS.WD_Counter[HS_APP_MM]++;   break;
        case LA_PROP_HK_TLM_MID:     HS.WD_Counter[HS_APP_PROP]++; break;
        case LA_EPS_HK_TLM_MID:      HS.WD_Counter[HS_APP_EPS]++;  break;
        case LA_HS_CMD_MID: HS.CmdCount++; break;
        default: break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
