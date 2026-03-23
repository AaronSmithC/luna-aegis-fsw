/**
 * @file cfe.h
 * @brief Minimal cFE API stubs for Luna-Aegis mission apps
 *
 * Provides function declarations that mirror the cFE 7.x API
 * surface.  Implementations live in a shim library for desktop
 * simulation; on flight hardware these resolve to the real cFE.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 */

#ifndef CFE_H
#define CFE_H

#include "cfe_sb_types.h"
#include <stdint.h>
#include <stdio.h>

/* ── App Lifecycle ───────────────────────────────────────── */

typedef uint32_t CFE_ES_AppId_t;

CFE_Status_t CFE_ES_RegisterApp(void);
void         CFE_ES_ExitApp(uint32_t ExitStatus);
CFE_Status_t CFE_ES_RunLoop(uint32_t *RunStatus);
void         CFE_ES_PerfLogEntry(uint32_t id);
void         CFE_ES_PerfLogExit(uint32_t id);
void         CFE_ES_WaitForStartupSync(uint32_t timeout_ms);

#define CFE_ES_RunStatus_APP_RUN   1
#define CFE_ES_RunStatus_APP_EXIT  2
#define CFE_ES_RunStatus_APP_ERROR 3

/* ── Software Bus ────────────────────────────────────────── */

CFE_Status_t CFE_SB_CreatePipe(CFE_SB_PipeId_t *PipeId,
                               uint16_t Depth,
                               const char *PipeName);

CFE_Status_t CFE_SB_Subscribe(CFE_SB_MsgId_t MsgId,
                              CFE_SB_PipeId_t PipeId);

CFE_Status_t CFE_SB_ReceiveBuffer(CFE_SB_Buffer_t **BufPtr,
                                  CFE_SB_PipeId_t PipeId,
                                  int32_t TimeOut);

CFE_Status_t CFE_SB_TransmitMsg(const CFE_MSG_Message_t *MsgPtr,
                                bool IncrementSequence);

void         CFE_MSG_Init(CFE_MSG_Message_t *MsgPtr,
                          CFE_SB_MsgId_t MsgId,
                          uint16_t Size);

void         CFE_MSG_SetFcnCode(CFE_MSG_Message_t *MsgPtr,
                                uint16_t FcnCode);

CFE_Status_t CFE_MSG_GetMsgId(const CFE_MSG_Message_t *MsgPtr,
                              CFE_SB_MsgId_t *MsgId);

CFE_Status_t CFE_MSG_GetFcnCode(const CFE_MSG_Message_t *MsgPtr,
                                uint16_t *FcnCode);

/* ── Event Services ──────────────────────────────────────── */

typedef uint16_t CFE_EVS_EventID_t;

#define CFE_EVS_EventType_INFORMATION  1
#define CFE_EVS_EventType_DEBUG        2
#define CFE_EVS_EventType_ERROR        3
#define CFE_EVS_EventType_CRITICAL     4

CFE_Status_t CFE_EVS_Register(const void *Filters,
                              uint16_t NumFilters,
                              uint16_t FilterScheme);

CFE_Status_t CFE_EVS_SendEvent(CFE_EVS_EventID_t EventID,
                               uint16_t EventType,
                               const char *Fmt, ...);

/* ── Table Services ──────────────────────────────────────── */

typedef int32_t CFE_TBL_Handle_t;

CFE_Status_t CFE_TBL_Register(CFE_TBL_Handle_t *TblHandle,
                              const char *Name,
                              uint32_t Size,
                              uint16_t Options,
                              void *ValidationFunc);

CFE_Status_t CFE_TBL_Load(CFE_TBL_Handle_t TblHandle,
                           uint16_t SrcType,
                           const void *SrcData);

CFE_Status_t CFE_TBL_GetAddress(void **TblPtr,
                                CFE_TBL_Handle_t TblHandle);

CFE_Status_t CFE_TBL_ReleaseAddress(CFE_TBL_Handle_t TblHandle);

#define CFE_TBL_OPT_DEFAULT  0
#define CFE_TBL_OPT_SNGL_BUFFER  1
#define CFE_TBL_SRC_FILE  0
#define CFE_TBL_SRC_ADDRESS  1

/* ── Time Services ───────────────────────────────────────── */

typedef struct {
    uint32_t Seconds;
    uint32_t Subseconds;
} CFE_TIME_SysTime_t;

CFE_TIME_SysTime_t CFE_TIME_GetTime(void);
CFE_TIME_SysTime_t CFE_TIME_GetMET(void);

/* ── Convenience Macros ──────────────────────────────────── */

#define OS_printf  printf

#endif /* CFE_H */
