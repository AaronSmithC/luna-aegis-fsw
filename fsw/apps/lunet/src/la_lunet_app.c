/**
 * @file la_lunet_app.c
 * @brief Luna-Aegis LUNET Interface Application
 *
 * Interfaces with LUNET surface infrastructure nodes for:
 * - Beacon reception (range/bearing for nav)
 * - Refueling coordination (ISRU cartridge port status)
 * - Field diagnostics via rover interface or node terminal
 *
 * Subscribes: SCH Wakeup (1 Hz), COMM_LinkPkt, LUNET_CMD
 * Publishes:  LUNET_BeaconPkt (1 Hz), LUNET_DiagPkt (async)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include <string.h>
#include <math.h>

#define LUNET_PIPE_DEPTH  16
#define LUNET_PIPE_NAME   "LUNET_PIPE"

/* Simulated LUNET beacon at south-pole landing site */
#define LUNET_BEACON_X_M     100.0
#define LUNET_BEACON_Y_M     -50.0
#define LUNET_BEACON_Z_M       0.0
#define LUNET_BEACON_ID        1

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    LA_COMM_LinkPkt_t  LastComm;
    bool               CommValid;

    LA_LUNET_BeaconPkt_t BeaconPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} LUNET_AppData_t;

static LUNET_AppData_t LUNET;

static CFE_Status_t LUNET_Init(void)
{
    memset(&LUNET, 0, sizeof(LUNET));
    LUNET.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&LUNET.CmdPipe, LUNET_PIPE_DEPTH, LUNET_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LUNET_CMD_MID),         LUNET.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_COMM_LINK_TLM_MID),     LUNET.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x0C)),  LUNET.CmdPipe);

    CFE_MSG_Init(&LUNET.BeaconPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_LUNET_BEACON_TLM_MID), sizeof(LA_LUNET_BeaconPkt_t));

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "LUNET: Interface initialized — beacon ID %u",
                      LUNET_BEACON_ID);
    return CFE_SUCCESS;
}

static void LUNET_Process(void)
{
    /* Simulate beacon reception */
    LUNET.BeaconPkt.BeaconPos_MCMF_m.x = LUNET_BEACON_X_M;
    LUNET.BeaconPkt.BeaconPos_MCMF_m.y = LUNET_BEACON_Y_M;
    LUNET.BeaconPkt.BeaconPos_MCMF_m.z = LUNET_BEACON_Z_M;

    /* Range: Euclidean from origin (vehicle at ~0,0,0 on surface) */
    double dx = LUNET_BEACON_X_M;
    double dy = LUNET_BEACON_Y_M;
    double dz = LUNET_BEACON_Z_M;
    double range = sqrt(dx*dx + dy*dy + dz*dz);

    LUNET.BeaconPkt.BeaconRange_m    = range;
    LUNET.BeaconPkt.BeaconBearing_rad = atan2(dy, dx);
    LUNET.BeaconPkt.BeaconID          = LUNET_BEACON_ID;

    /* Signal quality depends on comms link */
    uint8_t sq = 200; /* Good default */
    if (LUNET.CommValid && LUNET.LastComm.LinkState == 0) {
        sq = 50; /* Degraded without relay */
    }
    LUNET.BeaconPkt.SignalQuality = sq;

    /* Refuel port status — simulated: ready when landed */
    LUNET.BeaconPkt.RefuelReady = 1;

    CFE_SB_TransmitMsg(&LUNET.BeaconPkt.TlmHdr.Msg, true);
}

void LUNET_AppMain(void)
{
    CFE_Status_t s; CFE_SB_Buffer_t *SBBufPtr; CFE_SB_MsgId_t MsgId;
    if (LUNET_Init() != CFE_SUCCESS) { LUNET.RunStatus = CFE_ES_RunStatus_APP_ERROR; }
    while (CFE_ES_RunLoop(&LUNET.RunStatus) == true) {
        s = CFE_SB_ReceiveBuffer(&SBBufPtr, LUNET.CmdPipe, CFE_SB_PEND_FOREVER);
        if (s != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x0C): LUNET_Process(); LUNET.CycleCount++; break;
        case LA_COMM_LINK_TLM_MID:
            memcpy(&LUNET.LastComm, SBBufPtr, sizeof(LA_COMM_LinkPkt_t));
            LUNET.CommValid = true; break;
        case LA_LUNET_CMD_MID: LUNET.CmdCount++; break;
        default: break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
