/**
 * @file la_ds_app.c
 * @brief Luna-Aegis Data Storage (DS) Application
 *
 * Onboard file logging of telemetry and event packets.
 * Manages log file rotation, storage limits, and downlink queue.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include <string.h>

#define DS_PIPE_DEPTH  48
#define DS_PIPE_NAME   "DS_PIPE"
#define DS_MAX_FILES    8
#define DS_FILE_SIZE_MAX  (1024 * 1024)  /* 1 MB per file */

typedef struct {
    char     Filename[64];
    uint32_t BytesWritten;
    bool     Open;
} DS_FileEntry_t;

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    DS_FileEntry_t   Files[DS_MAX_FILES];
    uint32_t         TotalBytesLogged;
    uint32_t         PktsLogged;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} DS_AppData_t;

static DS_AppData_t DS;

static CFE_Status_t DS_Init(void)
{
    memset(&DS, 0, sizeof(DS));
    DS.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&DS.CmdPipe, DS_PIPE_DEPTH, DS_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_DS_CMD_MID), DS.CmdPipe);

    /* Subscribe to key telemetry for logging */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_NAV_STATE_TLM_MID),   DS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),    DS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_PROP_STATUS_TLM_MID), DS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_HS_ALERT_TLM_MID),    DS.CmdPipe);

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "DS: Data storage initialized");
    return CFE_SUCCESS;
}

void DS_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (DS_Init() != CFE_SUCCESS) { DS.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&DS.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, DS.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

        if (CFE_SB_MsgIdToValue(MsgId) == LA_DS_CMD_MID) { DS.CmdCount++; continue; }

        /* Log packet (simulated: just count) */
        DS.PktsLogged++;
        DS.TotalBytesLogged += 128; /* Approximate packet size */
        DS.CycleCount++;
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
