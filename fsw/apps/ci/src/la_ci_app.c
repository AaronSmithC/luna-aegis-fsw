/**
 * @file la_ci_app.c
 * @brief Luna-Aegis Command Ingest (CI) Application
 *
 * Receives uplink command stream from ground or Aegis Station
 * via COMM subsystem, deserializes CCSDS command packets, and
 * routes them onto the cFE Software Bus.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include <string.h>

#define CI_PIPE_DEPTH  32
#define CI_PIPE_NAME   "CI_PIPE"

/* In simulation: CI listens on a UDP socket for ground commands.
 * In flight: CI reads from the COMM subsystem's uplink buffer. */
#define CI_UDP_PORT    1234

typedef struct {
    CFE_SB_PipeId_t CmdPipe;
    uint32_t         RunStatus;
    uint32_t        CmdsIngested;
    uint32_t        CmdsRejected;
    uint32_t        CycleCount;
} CI_AppData_t;

static CI_AppData_t CI;

static CFE_Status_t CI_Init(void)
{
    memset(&CI, 0, sizeof(CI));
    CI.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&CI.CmdPipe, CI_PIPE_DEPTH, CI_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_CI_CMD_MID), CI.CmdPipe);

    /* In simulation: would open UDP socket here
     * int sock = socket(AF_INET, SOCK_DGRAM, 0);
     * bind(sock, ...CI_UDP_PORT...);
     */

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "CI: Command ingest initialized — UDP port %d (sim)",
                      CI_UDP_PORT);
    return CFE_SUCCESS;
}

void CI_AppMain(void)
{
    if (CI_Init() != CFE_SUCCESS) { CI.RunStatus = CFE_ES_RunStatus_APP_ERROR; }

    while (CFE_ES_RunLoop(&CI.RunStatus) == true) {
        /*
         * Simulation mode: poll UDP socket for incoming commands.
         * Deserialize CCSDS packet, validate checksum, route to SB.
         *
         * uint8_t buf[512];
         * ssize_t n = recvfrom(sock, buf, sizeof(buf), MSG_DONTWAIT, ...);
         * if (n > 0) {
         *     CFE_SB_MsgId_t mid;
         *     CFE_MSG_GetMsgId((CFE_MSG_Message_t*)buf, &mid);
         *     CFE_SB_TransmitMsg((CFE_MSG_Message_t*)buf, false);
         *     CI.CmdsIngested++;
         * }
         */

        CI.CycleCount++;
        break; /* Single pass for compile */
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
