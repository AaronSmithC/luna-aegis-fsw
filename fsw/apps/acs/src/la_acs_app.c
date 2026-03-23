/**
 * @file la_acs_app.c
 * @brief Luna-Aegis Attitude Control System (ACS) Application
 *
 * PD attitude controller that mixes gimbal and RCS commands.
 * Gimbal provides pitch/yaw authority during powered flight;
 * RCS handles roll and fine station-keeping at all times.
 *
 * Subscribes: NAV_StatePkt (40 Hz), GDN_CmdPkt (10 Hz),
 *             SCH Wakeup (40 Hz)
 * Publishes:  ACS_ActuatorCmdPkt (40 Hz)
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

#define ACS_PIPE_DEPTH   32
#define ACS_PIPE_NAME    "ACS_PIPE"

/* PD gains */
#define ACS_KP_ATT    8.0    /* Nm/rad  — attitude proportional   */
#define ACS_KD_RATE   4.0    /* Nm/(rad/s) — rate damping         */

/* Gimbal authority limits (rad) */
#define ACS_GIMBAL_MAX   0.105  /* ±6 degrees */

/* RCS thruster layout (simplified 12-thruster config)
 * Pairs: 0/1=+/-roll, 2/3=+/-pitch, 4/5=+/-yaw,
 *         6-11 = translation (not used by ACS) */
#define ACS_RCS_ROLL_POS   0
#define ACS_RCS_ROLL_NEG   1
#define ACS_RCS_PITCH_POS  2
#define ACS_RCS_PITCH_NEG  3
#define ACS_RCS_YAW_POS    4
#define ACS_RCS_YAW_NEG    5

/* RCS deadband (Nm) — below this, don't fire */
#define ACS_RCS_DEADBAND   0.1
/* RCS moment arm (m) */
#define ACS_RCS_ARM        1.5
/* RCS thruster force (N) */
#define ACS_RCS_FORCE      22.0

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    LA_NAV_StatePkt_t  LastNav;
    LA_GDN_CmdPkt_t   LastGdn;
    bool               NavValid;
    bool               GdnValid;

    /* Commanded attitude (from GDN gimbal angles as proxy) */
    double CmdPitch_rad;
    double CmdYaw_rad;
    double CmdRoll_rad;  /* Always zero for now */

    LA_ACS_ActuatorCmdPkt_t ActPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} ACS_AppData_t;

static ACS_AppData_t ACS;

static CFE_Status_t ACS_Init(void)
{
    memset(&ACS, 0, sizeof(ACS));
    ACS.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&ACS.CmdPipe, ACS_PIPE_DEPTH, ACS_PIPE_NAME);

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_ACS_CMD_MID),           ACS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_NAV_STATE_TLM_MID),     ACS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_GDN_CMD_TLM_MID),       ACS.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x02)),  ACS.CmdPipe);

    CFE_MSG_Init(&ACS.ActPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_ACS_ACTUATOR_CMD_MID), sizeof(LA_ACS_ActuatorCmdPkt_t));

    HAL_Gimbal_Init();
    HAL_RCS_Init();

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "ACS: Attitude control initialized — PD + RCS mixing @ 40 Hz");
    return CFE_SUCCESS;
}

static double clamp(double val, double lo, double hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static void ACS_ComputeControl(void)
{
    if (!ACS.NavValid) {
        /* No nav: zero all outputs */
        memset(&ACS.ActPkt.GimbalPitchCmd_rad, 0,
               sizeof(ACS.ActPkt) - sizeof(CFE_MSG_TelemetryHeader_t));
        return;
    }

    /* Attitude error: difference between commanded and current.
     * Simplified: use angular rates as proxy for error (small-angle). */
    double pitch_rate = ACS.LastNav.AngRate_rps.y;
    double yaw_rate   = ACS.LastNav.AngRate_rps.z;
    double roll_rate  = ACS.LastNav.AngRate_rps.x;

    /* PD control torques */
    double pitch_err = ACS.GdnValid ? ACS.LastGdn.GimbalPitch_rad : 0.0;
    double yaw_err   = ACS.GdnValid ? ACS.LastGdn.GimbalYaw_rad  : 0.0;
    double roll_err  = 0.0; /* Roll target: zero */

    double torque_pitch = ACS_KP_ATT * pitch_err - ACS_KD_RATE * pitch_rate;
    double torque_yaw   = ACS_KP_ATT * yaw_err   - ACS_KD_RATE * yaw_rate;
    double torque_roll  = ACS_KP_ATT * roll_err   - ACS_KD_RATE * roll_rate;

    /*
     * Command mixing:
     *  - Powered flight: gimbal handles pitch/yaw, RCS handles roll
     *  - Coast/landed: RCS handles everything
     */
    bool powered = ACS.GdnValid &&
                   ACS.LastGdn.GuidancePhase > 0 &&
                   ACS.LastGdn.ThrustCmd_N > 100.0;

    if (powered) {
        /* Gimbal: pitch and yaw */
        ACS.ActPkt.GimbalPitchCmd_rad = clamp(pitch_err, -ACS_GIMBAL_MAX, ACS_GIMBAL_MAX);
        ACS.ActPkt.GimbalYawCmd_rad   = clamp(yaw_err,   -ACS_GIMBAL_MAX, ACS_GIMBAL_MAX);

        /* RCS: roll only (pitch/yaw residual if gimbal saturated) */
        double gimbal_pitch_used = ACS.ActPkt.GimbalPitchCmd_rad;
        double gimbal_yaw_used   = ACS.ActPkt.GimbalYawCmd_rad;
        double pitch_residual = torque_pitch - gimbal_pitch_used * ACS.LastGdn.ThrustCmd_N * ACS_RCS_ARM;
        double yaw_residual   = torque_yaw   - gimbal_yaw_used   * ACS.LastGdn.ThrustCmd_N * ACS_RCS_ARM;

        ACS.ActPkt.RCS_Roll_Nm  = torque_roll;
        ACS.ActPkt.RCS_Pitch_Nm = pitch_residual;
        ACS.ActPkt.RCS_Yaw_Nm   = yaw_residual;
    } else {
        /* All RCS */
        ACS.ActPkt.GimbalPitchCmd_rad = 0.0;
        ACS.ActPkt.GimbalYawCmd_rad   = 0.0;
        ACS.ActPkt.RCS_Roll_Nm  = torque_roll;
        ACS.ActPkt.RCS_Pitch_Nm = torque_pitch;
        ACS.ActPkt.RCS_Yaw_Nm   = torque_yaw;
    }

    /* Build RCS firing mask */
    uint8_t mask = 0;
    if (ACS.ActPkt.RCS_Roll_Nm  >  ACS_RCS_DEADBAND) mask |= (1 << ACS_RCS_ROLL_POS);
    if (ACS.ActPkt.RCS_Roll_Nm  < -ACS_RCS_DEADBAND) mask |= (1 << ACS_RCS_ROLL_NEG);
    if (ACS.ActPkt.RCS_Pitch_Nm >  ACS_RCS_DEADBAND) mask |= (1 << ACS_RCS_PITCH_POS);
    if (ACS.ActPkt.RCS_Pitch_Nm < -ACS_RCS_DEADBAND) mask |= (1 << ACS_RCS_PITCH_NEG);
    if (ACS.ActPkt.RCS_Yaw_Nm   >  ACS_RCS_DEADBAND) mask |= (1 << ACS_RCS_YAW_POS);
    if (ACS.ActPkt.RCS_Yaw_Nm   < -ACS_RCS_DEADBAND) mask |= (1 << ACS_RCS_YAW_NEG);
    ACS.ActPkt.RCS_FiringMask = mask;

    /* Execute HAL commands */
    HAL_Gimbal_SetAngles(ACS.ActPkt.GimbalPitchCmd_rad,
                         ACS.ActPkt.GimbalYawCmd_rad);

    if (mask) {
        HAL_RCS_Cmd_t rcs_cmd = {0};
        rcs_cmd.firing_mask = mask;
        for (int i = 0; i < HAL_RCS_THRUSTER_COUNT; i++) {
            rcs_cmd.pulse_ms[i] = (mask & (1 << i)) ? 25 : 0; /* 25 ms pulse */
        }
        HAL_RCS_Fire(&rcs_cmd);
    }
}

void ACS_AppMain(void)
{
    CFE_Status_t     status;
    CFE_SB_Buffer_t *SBBufPtr;
    CFE_SB_MsgId_t   MsgId;

    if (ACS_Init() != CFE_SUCCESS) { ACS.RunStatus = CFE_ES_RunStatus_APP_ERROR; }

    while (CFE_ES_RunLoop(&ACS.RunStatus) == true) {
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, ACS.CmdPipe, CFE_SB_PEND_FOREVER);
        if (status != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x02):
            ACS_ComputeControl();
            CFE_SB_TransmitMsg(&ACS.ActPkt.TlmHdr.Msg, true);
            ACS.CycleCount++;
            break;
        case LA_NAV_STATE_TLM_MID:
            memcpy(&ACS.LastNav, SBBufPtr, sizeof(LA_NAV_StatePkt_t));
            ACS.NavValid = true;
            break;
        case LA_GDN_CMD_TLM_MID:
            memcpy(&ACS.LastGdn, SBBufPtr, sizeof(LA_GDN_CmdPkt_t));
            ACS.GdnValid = true;
            break;
        case LA_ACS_CMD_MID:
            ACS.CmdCount++;
            break;
        default:
            break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
