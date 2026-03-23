/**
 * @file la_gdn_app.c
 * @brief Luna-Aegis Guidance (GDN) Application
 *
 * Computes thrust vector, gimbal angles, and throttle commands
 * based on current nav state and mission phase.  Implements
 * gravity-turn ascent, powered descent guidance (PDG), and
 * constant-velocity hover/terminal phases.
 *
 * Subscribes: NAV_StatePkt (40 Hz), MM_PhasePkt (1 Hz),
 *             SCH Wakeup (10 Hz)
 * Publishes:  GDN_CmdPkt (10 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include <string.h>
#include <math.h>

#define GDN_PIPE_DEPTH   32
#define GDN_PIPE_NAME    "GDN_PIPE"

/* Vehicle constants */
#define GDN_DRY_MASS_KG     5250.0
#define GDN_MAX_THRUST_N    30000.0
#define GDN_MOON_G          1.625      /* m/s^2 */
#define GDN_ISP_S            440.0
#define GDN_G0               9.80665

/* Guidance gains (simplified PD) */
#define GDN_KP_POS     0.5
#define GDN_KD_VEL     2.0
#define GDN_HOVER_ALT_M  50.0
#define GDN_TERM_RATE_MPS  -1.0  /* 1 m/s descent */

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    LA_NAV_StatePkt_t  LastNav;
    LA_MM_PhasePkt_t   LastPhase;
    bool               NavValid;
    bool               PhaseValid;

    LA_GDN_CmdPkt_t    CmdPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} GDN_AppData_t;

static GDN_AppData_t GDN;

static CFE_Status_t GDN_Init(void)
{
    memset(&GDN, 0, sizeof(GDN));
    GDN.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&GDN.CmdPipe, GDN_PIPE_DEPTH, GDN_PIPE_NAME);

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_GDN_CMD_MID),           GDN.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_NAV_STATE_TLM_MID),     GDN.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),      GDN.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x05)),  GDN.CmdPipe);

    CFE_MSG_Init(&GDN.CmdPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_GDN_CMD_TLM_MID), sizeof(LA_GDN_CmdPkt_t));

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "GDN: Guidance initialized");
    return CFE_SUCCESS;
}

static void GDN_ComputeGuidance(void)
{
    if (!GDN.NavValid || !GDN.PhaseValid) {
        /* No data: zero commands */
        GDN.CmdPkt.ThrustCmd_N     = 0.0;
        GDN.CmdPkt.GimbalPitch_rad = 0.0;
        GDN.CmdPkt.GimbalYaw_rad   = 0.0;
        GDN.CmdPkt.ThrottleCmd_pct = 0.0;
        GDN.CmdPkt.GuidancePhase   = 0;
        return;
    }

    double pos_z = GDN.LastNav.Pos_MCMF_m.z;
    double vel_z = GDN.LastNav.Vel_MCMF_mps.z;
    double vel_x = GDN.LastNav.Vel_MCMF_mps.x;
    double vel_y = GDN.LastNav.Vel_MCMF_mps.y;

    /* Estimate current mass from MM prop remaining */
    double mass = GDN_DRY_MASS_KG + GDN.LastPhase.PropRemaining_kg;
    if (mass < GDN_DRY_MASS_KG) mass = GDN_DRY_MASS_KG;

    double thrust_cmd = 0.0;
    double pitch_cmd  = 0.0;
    double yaw_cmd    = 0.0;
    uint8_t gdn_phase = 0;

    switch (GDN.LastPhase.CurrentPhase) {

    case LA_PHASE_POWERED_ASC:
        /*
         * Gravity-turn ascent: thrust along velocity vector
         * with gravity compensation.  Full throttle initially,
         * throttle back as target velocity is approached.
         */
        gdn_phase = 1; /* powered */
        thrust_cmd = GDN_MAX_THRUST_N; /* Full throttle ascent */

        /* Pitch: point along velocity vector */
        {
            double v_horiz = sqrt(vel_x * vel_x + vel_y * vel_y);
            pitch_cmd = atan2(vel_z, v_horiz);
            yaw_cmd   = atan2(vel_y, vel_x);
        }
        /* Clamp gimbal to ±6 deg (0.105 rad) */
        if (pitch_cmd >  0.105) pitch_cmd =  0.105;
        if (pitch_cmd < -0.105) pitch_cmd = -0.105;
        if (yaw_cmd >  0.105) yaw_cmd =  0.105;
        if (yaw_cmd < -0.105) yaw_cmd = -0.105;
        break;

    case LA_PHASE_COAST:
        /* No thrust during coast */
        gdn_phase  = 0;
        thrust_cmd = 0.0;
        break;

    case LA_PHASE_POWERED_DES:
        /*
         * Powered descent guidance (simplified).
         * PD controller on altitude + vertical rate.
         * Lateral braking via horizontal velocity nulling.
         */
        gdn_phase = 1;
        {
            double az_cmd = GDN_KP_POS * (0.0 - pos_z)
                          + GDN_KD_VEL * (0.0 - vel_z);
            thrust_cmd = mass * (GDN_MOON_G + az_cmd);

            /* Lateral correction via gimbal */
            double ax_cmd = -GDN_KD_VEL * vel_x;
            double ay_cmd = -GDN_KD_VEL * vel_y;
            if (thrust_cmd > 100.0) {
                pitch_cmd = asin(fmin(fmax(ax_cmd * mass / thrust_cmd, -0.105), 0.105));
                yaw_cmd   = asin(fmin(fmax(ay_cmd * mass / thrust_cmd, -0.105), 0.105));
            }
        }
        break;

    case LA_PHASE_HOVER:
        /*
         * Hold altitude at hover setpoint.
         * PD controller, near-hover throttle.
         */
        gdn_phase = 2;
        {
            double alt_err = GDN_HOVER_ALT_M - pos_z;
            double az_cmd = GDN_KP_POS * alt_err
                          + GDN_KD_VEL * (0.0 - vel_z);
            thrust_cmd = mass * (GDN_MOON_G + az_cmd);

            /* Kill horizontal velocity */
            double ax_cmd = -GDN_KD_VEL * vel_x;
            double ay_cmd = -GDN_KD_VEL * vel_y;
            if (thrust_cmd > 100.0) {
                pitch_cmd = asin(fmin(fmax(ax_cmd * mass / thrust_cmd, -0.105), 0.105));
                yaw_cmd   = asin(fmin(fmax(ay_cmd * mass / thrust_cmd, -0.105), 0.105));
            }
        }
        break;

    case LA_PHASE_TERMINAL:
        /*
         * Constant descent rate to surface.
         */
        gdn_phase = 1;
        {
            double vz_err = GDN_TERM_RATE_MPS - vel_z;
            double az_cmd = GDN_KD_VEL * vz_err;
            thrust_cmd = mass * (GDN_MOON_G + az_cmd);
        }
        break;

    case LA_PHASE_ABORT:
        /* Abort: gravity-turn back to safe altitude */
        gdn_phase = 1;
        thrust_cmd = GDN_MAX_THRUST_N * 0.8;
        break;

    default:
        /* PREFLIGHT, LANDED, SAFED: no thrust */
        gdn_phase  = 0;
        thrust_cmd = 0.0;
        break;
    }

    /* Clamp thrust */
    if (thrust_cmd < 0.0)           thrust_cmd = 0.0;
    if (thrust_cmd > GDN_MAX_THRUST_N) thrust_cmd = GDN_MAX_THRUST_N;

    GDN.CmdPkt.ThrustCmd_N     = thrust_cmd;
    GDN.CmdPkt.GimbalPitch_rad = pitch_cmd;
    GDN.CmdPkt.GimbalYaw_rad   = yaw_cmd;
    GDN.CmdPkt.ThrottleCmd_pct = (thrust_cmd / GDN_MAX_THRUST_N) * 100.0;
    GDN.CmdPkt.GuidancePhase   = gdn_phase;
}

void GDN_AppMain(void)
{
    CFE_Status_t     status;
    CFE_SB_Buffer_t *SBBufPtr;
    CFE_SB_MsgId_t   MsgId;

    if (GDN_Init() != CFE_SUCCESS) { GDN.RunStatus = CFE_ES_RunStatus_APP_ERROR; }

    while (CFE_ES_RunLoop(&GDN.RunStatus) == true) {
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, GDN.CmdPipe, CFE_SB_PEND_FOREVER);
        if (status != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x05):
            GDN_ComputeGuidance();
            CFE_SB_TransmitMsg(&GDN.CmdPkt.TlmHdr.Msg, true);
            GDN.CycleCount++;
            break;
        case LA_NAV_STATE_TLM_MID:
            memcpy(&GDN.LastNav, SBBufPtr, sizeof(LA_NAV_StatePkt_t));
            GDN.NavValid = true;
            break;
        case LA_MM_PHASE_TLM_MID:
            memcpy(&GDN.LastPhase, SBBufPtr, sizeof(LA_MM_PhasePkt_t));
            GDN.PhaseValid = true;
            break;
        case LA_GDN_CMD_MID:
            GDN.CmdCount++;
            break;
        default:
            break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
