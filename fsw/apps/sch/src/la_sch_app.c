/**
 * @file la_sch_app.c
 * @brief Luna-Aegis Scheduler (SCH) Application
 *
 * Major/minor frame scheduler.  Driven by cFE TIME 1 Hz tick,
 * sends wakeup messages to all cyclic apps per the schedule
 * table.  40 Hz minor frame (25 ms).
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include <string.h>

#define SCH_PIPE_DEPTH   8
#define SCH_PIPE_NAME    "SCH_PIPE"
#define SCH_MINOR_PER_MAJOR  40  /* 40 minor frames per 1 Hz major */

/* Schedule table entry */
typedef struct {
    uint16_t        WakeupMID;  /* Raw integer — wrapped at point of use */
    uint16_t        Divisor;    /* Fire every N minor frames */
    uint16_t        Offset;     /* Phase offset within divisor */
} SCH_Entry_t;

/* Schedule table — defines all cyclic wakeups */
static const SCH_Entry_t SCH_Table[] = {
    /* 40 Hz apps (every minor frame) */
    { LA_SCH_WAKEUP_MID(0x01),  1, 0 },  /* IMU_MGR */
    { LA_SCH_WAKEUP_MID(0x02),  1, 0 },  /* ACS */
    { LA_SCH_WAKEUP_MID(0x03),  1, 0 },  /* NAV */

    /* 10 Hz apps (every 4th minor frame) */
    { LA_SCH_WAKEUP_MID(0x04),  4, 0 },  /* ALT_MGR */
    { LA_SCH_WAKEUP_MID(0x05),  4, 1 },  /* GDN */
    { LA_SCH_WAKEUP_MID(0x06),  4, 2 },  /* PROP */

    /* 2 Hz apps (every 20th minor frame) */
    { LA_SCH_WAKEUP_MID(0x07), 20, 0 },  /* TRN */

    /* 1 Hz apps (major frame only = every 40th) */
    { LA_SCH_WAKEUP_MID(0x08), 40, 0 },  /* MM */
    { LA_SCH_WAKEUP_MID(0x09), 40, 5 },  /* EPS */
    { LA_SCH_WAKEUP_MID(0x0A), 40, 10 }, /* LSS */
    { LA_SCH_WAKEUP_MID(0x0B), 40, 15 }, /* COMM */
    { LA_SCH_WAKEUP_MID(0x0C), 40, 20 }, /* LUNET */
    { LA_SCH_WAKEUP_MID(0x0D), 40, 25 }, /* LG */
    { LA_SCH_WAKEUP_MID(0x0E), 40, 26 }, /* DOCK */
    { LA_SCH_WAKEUP_MID(0x0F), 40, 30 }, /* HK */
    { LA_SCH_WAKEUP_MID(0x10), 40, 32 }, /* TO */
    { LA_SCH_WAKEUP_MID(0x11), 40, 34 }, /* HS + SC */
    { LA_SCH_WAKEUP_MID(0x12), 40, 36 }, /* LC */

    /* 0.1 Hz (every 400th minor = every 10 major frames) */
    { LA_SCH_WAKEUP_MID(0x13), 400, 0 }, /* TCS */
};

#define SCH_TABLE_SIZE  (sizeof(SCH_Table) / sizeof(SCH_Table[0]))

typedef struct {
    CFE_SB_PipeId_t CmdPipe;
    uint32_t         RunStatus;
    uint32_t        MinorFrame;
    uint32_t        MajorFrame;
    uint32_t        CycleCount;
} SCH_AppData_t;

static SCH_AppData_t SCH;

static CFE_Status_t SCH_Init(void)
{
    memset(&SCH, 0, sizeof(SCH));
    SCH.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&SCH.CmdPipe, SCH_PIPE_DEPTH, SCH_PIPE_NAME);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_CMD_MID), SCH.CmdPipe);
    /* In real cFS, SCH is driven by a timer or TIME 1Hz callback.
     * Here it's self-driven for simulation. */

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "SCH: Scheduler initialized — %zu entries, 40 Hz minor",
                      SCH_TABLE_SIZE);
    return CFE_SUCCESS;
}

static void SCH_ProcessMinorFrame(void)
{
    CFE_MSG_CommandHeader_t wakeup_msg;

    for (size_t i = 0; i < SCH_TABLE_SIZE; i++) {
        const SCH_Entry_t *e = &SCH_Table[i];
        if ((SCH.MinorFrame % e->Divisor) == e->Offset) {
            CFE_MSG_Init(&wakeup_msg.Msg, CFE_SB_ValueToMsgId(e->WakeupMID),
                         sizeof(wakeup_msg));
            CFE_SB_TransmitMsg(&wakeup_msg.Msg, true);
        }
    }

    SCH.MinorFrame++;
    if (SCH.MinorFrame >= SCH_MINOR_PER_MAJOR) {
        SCH.MinorFrame = 0;
        SCH.MajorFrame++;
    }
}

void SCH_AppMain(void)
{
    if (SCH_Init() != CFE_SUCCESS) { SCH.RunStatus = CFE_ES_RunStatus_APP_ERROR; }

    while (CFE_ES_RunLoop(&SCH.RunStatus) == true) {
        /* In simulation: called at minor frame rate (25 ms).
         * Real impl: timer ISR or TIME callback. */
        SCH_ProcessMinorFrame();
        SCH.CycleCount++;

        /* Yield — in real OSAL this is OS_TaskDelay(25) */
        /* For compilation only; sim harness drives timing */
        break; /* Single pass for compile check */
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
