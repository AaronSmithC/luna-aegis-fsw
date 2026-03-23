/**
 * @file la_sc_app.c
 * @brief Luna-Aegis Stored Command (SC) Application
 *
 * Executes time-tagged command sequences for abort, safing,
 * contingency, and nominal mission operations.  Sequences
 * are stored as onboard tables.
 *
 * Subscribes: TIME_1HzCmd, MM_PhasePkt, SC_CMD
 * Publishes:  SC_ExecCmd (async), SC_HK_TLM
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include <string.h>

#define SC_PIPE_DEPTH   16
#define SC_PIPE_NAME    "SC_PIPE"

/* Sequence table sizes */
#define SC_MAX_CMDS_PER_SEQ  64
#define SC_MAX_SEQUENCES      8

/* Stored command entry */
typedef struct {
    double          ExecTime_MET_s;  /* When to execute (MET) */
    CFE_SB_MsgId_t  MsgId;          /* Command MID to send */
    uint16_t        FcnCode;        /* Function code */
    bool            Executed;
} SC_CmdEntry_t;

typedef struct {
    char            Name[16];
    SC_CmdEntry_t   Cmds[SC_MAX_CMDS_PER_SEQ];
    uint16_t        Count;
    bool            Active;
} SC_Sequence_t;

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    SC_Sequence_t    Sequences[SC_MAX_SEQUENCES];
    double           CurrentMET_s;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t CmdsExecuted;
    uint16_t ErrCount;
} SC_AppData_t;

static SC_AppData_t SC;

static void SC_LoadDefaultSequences(void)
{
    /*
     * Sequence 0: Abort safing sequence
     * Triggered by MM entering ABORT phase.
     */
    SC_Sequence_t *s = &SC.Sequences[0];
    strncpy(s->Name, "ABORT_SAFE", 15);
    s->Count = 3;
    s->Active = false;

    /* t+0: Shutdown engine */
    s->Cmds[0].ExecTime_MET_s = 0.0;
    s->Cmds[0].MsgId          = CFE_SB_ValueToMsgId(LA_PROP_CMD_MID);
    s->Cmds[0].FcnCode        = 5; /* SHUTDOWN */

    /* t+5: Retract docking collar if extended */
    s->Cmds[1].ExecTime_MET_s = 5.0;
    s->Cmds[1].MsgId          = CFE_SB_ValueToMsgId(LA_DOCK_CMD_MID);
    s->Cmds[1].FcnCode        = 3; /* DEMATE */

    /* t+10: Deploy landing gear */
    s->Cmds[2].ExecTime_MET_s = 10.0;
    s->Cmds[2].MsgId          = CFE_SB_ValueToMsgId(LA_LG_CMD_MID);
    s->Cmds[2].FcnCode        = 2; /* DEPLOY */

    /*
     * Sequence 1: Pre-flight startup
     */
    s = &SC.Sequences[1];
    strncpy(s->Name, "PREFLT_START", 15);
    s->Count  = 2;
    s->Active = false;

    s->Cmds[0].ExecTime_MET_s = 0.0;
    s->Cmds[0].MsgId          = CFE_SB_ValueToMsgId(LA_PROP_CMD_MID);
    s->Cmds[0].FcnCode        = 2; /* ARM */

    s->Cmds[1].ExecTime_MET_s = 2.0;
    s->Cmds[1].MsgId          = CFE_SB_ValueToMsgId(LA_PROP_CMD_MID);
    s->Cmds[1].FcnCode        = 4; /* START */
}

static CFE_Status_t SC_Init(void)
{
    memset(&SC, 0, sizeof(SC));
    SC.RunStatus = CFE_ES_RunStatus_APP_RUN;

    SC_LoadDefaultSequences();

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&SC.CmdPipe, SC_PIPE_DEPTH, SC_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SC_CMD_MID),           SC.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),     SC.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x11)), SC.CmdPipe); /* Shares HS slot */

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "SC: Stored command initialized — %d sequences loaded", 2);
    return CFE_SUCCESS;
}

static void SC_ExecuteActive(void)
{
    for (int s = 0; s < SC_MAX_SEQUENCES; s++) {
        SC_Sequence_t *seq = &SC.Sequences[s];
        if (!seq->Active) continue;

        for (uint16_t c = 0; c < seq->Count; c++) {
            SC_CmdEntry_t *cmd = &seq->Cmds[c];
            if (cmd->Executed) continue;

            if (SC.CurrentMET_s >= cmd->ExecTime_MET_s) {
                /* Build and send command */
                CFE_MSG_CommandHeader_t msg;
                CFE_MSG_Init(&msg.Msg,
                             cmd->MsgId, sizeof(msg));
                CFE_MSG_SetFcnCode(&msg.Msg, cmd->FcnCode);
                CFE_SB_TransmitMsg(&msg.Msg, true);

                cmd->Executed = true;
                SC.CmdsExecuted++;

                CFE_EVS_SendEvent(10, CFE_EVS_EventType_INFORMATION,
                    "SC: Seq '%s' cmd %u exec — MID 0x%04X FC %u",
                    seq->Name, c, (unsigned int)CFE_SB_MsgIdToValue(cmd->MsgId), cmd->FcnCode);
            }
        }

        /* Check if sequence complete */
        bool all_done = true;
        for (uint16_t c = 0; c < seq->Count; c++) {
            if (!seq->Cmds[c].Executed) { all_done = false; break; }
        }
        if (all_done) {
            seq->Active = false;
            CFE_EVS_SendEvent(11, CFE_EVS_EventType_INFORMATION,
                              "SC: Sequence '%s' complete", seq->Name);
        }
    }
}

void SC_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (SC_Init() != CFE_SUCCESS) { SC.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&SC.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, SC.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x11):
            SC.CurrentMET_s += 1.0;
            SC_ExecuteActive();
            SC.CycleCount++;
            break;
        case LA_MM_PHASE_TLM_MID: {
            /* Auto-trigger abort sequence on ABORT phase */
            LA_MM_PhasePkt_t *ph = (LA_MM_PhasePkt_t *)SBBufPtr;
            if (ph->CurrentPhase == LA_PHASE_ABORT &&
                !SC.Sequences[0].Active)
            {
                SC.Sequences[0].Active = true;
                for (uint16_t c = 0; c < SC.Sequences[0].Count; c++) {
                    SC.Sequences[0].Cmds[c].Executed = false;
                    SC.Sequences[0].Cmds[c].ExecTime_MET_s += SC.CurrentMET_s;
                }
                CFE_EVS_SendEvent(12, CFE_EVS_EventType_INFORMATION,
                                  "SC: Abort safing sequence activated");
            }
            break;
        }
        case LA_SC_CMD_MID: SC.CmdCount++; break;
        default: break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
