/**
 * @file la_fm_app.c
 * @brief Luna-Aegis File Manager (FM) Application
 *
 * Onboard file operations: copy, move, delete, directory listing,
 * decompression.  Supports table uploads and log management.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include <string.h>

#define FM_PIPE_DEPTH  16
#define FM_PIPE_NAME   "FM_PIPE"

#define FM_NOOP_CC     0
#define FM_RESET_CC    1
#define FM_COPY_CC     2
#define FM_MOVE_CC     3
#define FM_DELETE_CC   4
#define FM_DIRLIST_CC  5

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    uint32_t FilesManaged;
    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} FM_AppData_t;

static FM_AppData_t FM;

static CFE_Status_t FM_Init(void)
{
    memset(&FM, 0, sizeof(FM));
    FM.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&FM.CmdPipe, FM_PIPE_DEPTH, FM_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_FM_CMD_MID), FM.CmdPipe);

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "FM: File manager initialized");
    return CFE_SUCCESS;
}

static void FM_ProcessCommand(const CFE_MSG_Message_t *MsgPtr)
{
    CFE_MSG_FcnCode_t fc;
    CFE_MSG_GetFcnCode(MsgPtr, &fc);

    switch (fc) {
    case FM_NOOP_CC:
        FM.CmdCount++;
        CFE_EVS_SendEvent(10, CFE_EVS_EventType_INFORMATION,
                          "FM: NOOP — v1.0");
        break;
    case FM_RESET_CC:
        FM.CmdCount = 0; FM.ErrCount = 0;
        break;
    case FM_COPY_CC:
    case FM_MOVE_CC:
    case FM_DELETE_CC:
    case FM_DIRLIST_CC:
        /* Placeholder: real impl would parse filename args from cmd packet */
        FM.CmdCount++;
        FM.FilesManaged++;
        break;
    default:
        FM.ErrCount++;
        break;
    }
}

void FM_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (FM_Init() != CFE_SUCCESS) { FM.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&FM.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, FM.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_FM_CMD_MID:
            FM_ProcessCommand(&SBBufPtr->Msg);
            break;
        default: break;
        }
        FM.CycleCount++;
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
