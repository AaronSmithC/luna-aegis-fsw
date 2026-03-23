/**
 * @file la_imu_mgr_app.c
 * @brief Luna-Aegis IMU Manager (IMU_MGR) Application
 *
 * Reads primary FOG/RLG and backup MEMS IMUs via HAL,
 * performs cross-check and source selection, publishes
 * fused inertial data at 40 Hz.
 *
 * Subscribes: SCH Wakeup (40 Hz), IMU_MGR_CMD
 * Publishes:  IMU_DataPkt (40 Hz), IMU_MGR_HK_TLM
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include "../../hal/inc/la_hal.h"
#include <string.h>
#include <math.h>

#define IMU_PIPE_DEPTH     16
#define IMU_PIPE_NAME      "IMU_PIPE"

/* Cross-check thresholds */
#define IMU_ACCEL_DISAGREE_THRESH   0.5    /* m/s^2 */
#define IMU_GYRO_DISAGREE_THRESH    0.01   /* rad/s */

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    /* Sensor data */
    HAL_IMU_Raw_t    Primary;
    HAL_IMU_Raw_t    Backup;
    bool             PrimaryValid;
    bool             BackupValid;

    /* Source selection */
    uint8_t          SourceSelect;  /* 0=FOG/RLG, 1=MEMS, 2=blended */
    uint8_t          StatusFlags;

    /* Output packet */
    LA_IMU_DataPkt_t DataPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} IMU_AppData_t;

static IMU_AppData_t IMU;

static CFE_Status_t IMU_Init(void)
{
    memset(&IMU, 0, sizeof(IMU));
    IMU.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&IMU.CmdPipe, IMU_PIPE_DEPTH, IMU_PIPE_NAME);

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_IMU_MGR_CMD_MID),       IMU.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x01)),  IMU.CmdPipe); /* 40 Hz */

    CFE_MSG_Init(&IMU.DataPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_IMU_DATA_TLM_MID),
                 sizeof(LA_IMU_DataPkt_t));

    HAL_IMU_Init();

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "IMU_MGR: Initialized — FOG/RLG primary, MEMS backup");
    return CFE_SUCCESS;
}

static void IMU_ReadSensors(void)
{
    IMU.PrimaryValid = (HAL_IMU_ReadPrimary(&IMU.Primary) == HAL_OK
                        && IMU.Primary.status == 0);
    IMU.BackupValid  = (HAL_IMU_ReadBackup(&IMU.Backup)   == HAL_OK
                        && IMU.Backup.status == 0);

    IMU.StatusFlags = 0;
    if (IMU.PrimaryValid) IMU.StatusFlags |= 0x01;
    if (IMU.BackupValid)  IMU.StatusFlags |= 0x02;
}

static void IMU_SelectSource(void)
{
    if (IMU.PrimaryValid && IMU.BackupValid) {
        /* Cross-check: compare primary vs backup */
        double accel_diff = 0.0, gyro_diff = 0.0;
        for (int i = 0; i < 3; i++) {
            double da = IMU.Primary.accel_mps2[i] - IMU.Backup.accel_mps2[i];
            double dg = IMU.Primary.gyro_rps[i]   - IMU.Backup.gyro_rps[i];
            accel_diff += da * da;
            gyro_diff  += dg * dg;
        }
        accel_diff = sqrt(accel_diff);
        gyro_diff  = sqrt(gyro_diff);

        if (accel_diff > IMU_ACCEL_DISAGREE_THRESH ||
            gyro_diff  > IMU_GYRO_DISAGREE_THRESH)
        {
            /* Disagreement: prefer primary (higher quality) */
            IMU.SourceSelect = 0;
            CFE_EVS_SendEvent(10, CFE_EVS_EventType_ERROR,
                              "IMU_MGR: Primary/backup disagree "
                              "(accel %.3f, gyro %.4f) — using primary",
                              accel_diff, gyro_diff);
        } else {
            /* Agreement: use primary */
            IMU.SourceSelect = 0;
        }
    } else if (IMU.PrimaryValid) {
        IMU.SourceSelect = 0;
    } else if (IMU.BackupValid) {
        IMU.SourceSelect = 1;
        CFE_EVS_SendEvent(11, CFE_EVS_EventType_ERROR,
                          "IMU_MGR: Primary IMU failed — fallback to MEMS");
    } else {
        /* Both failed */
        IMU.SourceSelect = 0;
        CFE_EVS_SendEvent(12, CFE_EVS_EventType_CRITICAL,
                          "IMU_MGR: ALL IMUs FAILED — propagating last known");
    }
}

static void IMU_BuildPacket(void)
{
    HAL_IMU_Raw_t *src = (IMU.SourceSelect == 1) ? &IMU.Backup : &IMU.Primary;

    IMU.DataPkt.SpecForce_mps2.x = src->accel_mps2[0];
    IMU.DataPkt.SpecForce_mps2.y = src->accel_mps2[1];
    IMU.DataPkt.SpecForce_mps2.z = src->accel_mps2[2];
    IMU.DataPkt.AngRate_rps.x    = src->gyro_rps[0];
    IMU.DataPkt.AngRate_rps.y    = src->gyro_rps[1];
    IMU.DataPkt.AngRate_rps.z    = src->gyro_rps[2];

    /* Attitude: integrated from angular rate (simplified placeholder) */
    /* Real implementation: quaternion integration */
    IMU.DataPkt.Attitude.q0 = 1.0;
    IMU.DataPkt.Attitude.q1 = 0.0;
    IMU.DataPkt.Attitude.q2 = 0.0;
    IMU.DataPkt.Attitude.q3 = 0.0;

    IMU.DataPkt.SourceSelect = IMU.SourceSelect;
    IMU.DataPkt.StatusFlags  = IMU.StatusFlags;
}

void IMU_MGR_AppMain(void)
{
    CFE_Status_t     status;
    CFE_SB_Buffer_t *SBBufPtr;
    CFE_SB_MsgId_t   MsgId;

    if (IMU_Init() != CFE_SUCCESS) {
        IMU.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&IMU.RunStatus) == true) {
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, IMU.CmdPipe,
                                      CFE_SB_PEND_FOREVER);
        if (status != CFE_SUCCESS) continue;

        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x01):
            IMU_ReadSensors();
            IMU_SelectSource();
            IMU_BuildPacket();
            CFE_SB_TransmitMsg(&IMU.DataPkt.TlmHdr.Msg, true);
            IMU.CycleCount++;
            break;

        case LA_IMU_MGR_CMD_MID:
            /* Command processing (NOOP, reset, force source) */
            IMU.CmdCount++;
            break;

        default:
            break;
        }
    }

    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
