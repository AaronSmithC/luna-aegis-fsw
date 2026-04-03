/**
 * @file la_nav_app.c
 * @brief Luna-Aegis Navigation Filter (NAV) Application
 *
 * Extended Kalman Filter fusing IMU, altimeter, TRN, and LUNET
 * beacon data into a unified navigation solution.
 *
 * Subscribes: IMU_DataPkt (40 Hz), ALT_DataPkt (10 Hz),
 *             TRN_PosFix (2 Hz), LUNET_BeaconPkt (1 Hz),
 *             MM_PhasePkt (1 Hz), SCH Wakeup (40 Hz)
 * Publishes:  NAV_StatePkt (40 Hz)
 *
 * State Vector: [pos(3), vel(3), att_err(3), gyro_bias(3)] = 12
 *
 * Trajectory Model (Rev D — ECL-2026-005):
 *   Simplified 1D range model.  Distance-to-destination is
 *   initialized from the destination table at mission start and
 *   decremented by integrated speed during powered/coast phases.
 *   The model assumes a straight-line great-circle path.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include "../../common/la_destinations.h"
#include <string.h>
#include <math.h>

/* ── App Constants ───────────────────────────────────────── */

#define NAV_PIPE_DEPTH      48
#define NAV_PIPE_NAME       "NAV_PIPE"
#define NAV_STATE_DIM       12
#define NAV_DT_NOMINAL      0.025  /* 40 Hz */

/* Command function codes */
#define NAV_NOOP_CC         0
#define NAV_RESET_CC        1
#define NAV_SET_DEST_CC     2

/* Lunar constants */
#define MOON_MU             4.9048695e12  /* GM (m^3/s^2) */
#define MOON_RADIUS_M       1737400.0

/* Process noise tuning */
#define NAV_ACCEL_NOISE     0.01    /* m/s^2 */
#define NAV_GYRO_NOISE      0.0001  /* rad/s */
#define NAV_BIAS_DRIFT      1e-6    /* rad/s/sqrt(s) */

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    /* EKF state */
    double  state[NAV_STATE_DIM];     /* pos(3), vel(3), att_err(3), bias(3) */
    double  P[NAV_STATE_DIM][NAV_STATE_DIM]; /* Covariance */
    LA_Quat_t att_ref;                /* Reference attitude quaternion */

    /* Cached sensor inputs */
    LA_IMU_DataPkt_t   LastIMU;
    LA_ALT_DataPkt_t   LastAlt;
    LA_TRN_HazardPkt_t LastTRN;
    LA_LUNET_BeaconPkt_t LastBeacon;
    bool               IMU_fresh;
    bool               ALT_fresh;
    bool               TRN_fresh;
    bool               Beacon_fresh;

    /* Nav mode */
    uint8_t  NavMode;   /* 0=propagate, 1=IMU+alt, 2=full fusion */
    uint8_t  NavHealth;  /* 0=OK, 1=degraded, 2=lost */

    /* Trajectory / distance-to-destination model */
    LA_MM_PhasePkt_t   LastPhase;       /* Cached MM phase packet           */
    bool               Phase_fresh;
    uint8_t            OriginId;        /* Where we are (dest ID, 0=Shack)  */
    uint8_t            DestId;          /* Where we're going (0xFF = none)  */
    double             DistToDest_m;    /* Remaining range to destination   */
    double             InitDist_m;      /* Distance at mission start        */
    bool               TrajActive;      /* True while in powered/coast      */

    /* Output packet */
    LA_NAV_StatePkt_t  StatePkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} NAV_AppData_t;

static NAV_AppData_t NAV;

/* ── Forward Declarations ────────────────────────────────── */

static void NAV_Propagate(double dt);
static void NAV_UpdateIMU(void);
static void NAV_UpdateAltimeter(void);
static void NAV_UpdateTRN(void);
static void NAV_UpdateBeacon(void);
static void NAV_SendStatePkt(void);
static void NAV_AssessHealth(void);
static void NAV_UpdateTrajectory(double dt);
static void NAV_ProcessCommand(const CFE_MSG_Message_t *MsgPtr);

/* ── Initialization ──────────────────────────────────────── */

static CFE_Status_t NAV_Init(void)
{
    memset(&NAV, 0, sizeof(NAV));
    NAV.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /* Initial state: on surface at south pole (simplified) */
    NAV.state[0] = 0.0;       /* x position (local frame) */
    NAV.state[1] = 0.0;       /* y position */
    NAV.state[2] = 0.0;       /* z position (AGL) */
    /* vel, att_err, bias all zero initially */

    /* Initial attitude: upright on surface */
    NAV.att_ref.q0 = 1.0;
    NAV.att_ref.q1 = 0.0;
    NAV.att_ref.q2 = 0.0;
    NAV.att_ref.q3 = 0.0;

    /* Initial covariance: moderate uncertainty */
    for (int i = 0; i < NAV_STATE_DIM; i++) {
        NAV.P[i][i] = (i < 3) ? 10.0   /* 10 m pos */
                     : (i < 6) ? 0.1    /* 0.1 m/s vel */
                     : (i < 9) ? 0.01   /* 0.01 rad att */
                     :           1e-5;   /* bias */
    }

    NAV.NavMode   = 0; /* propagate only until sensors come in */
    NAV.NavHealth = 0;

    /* Trajectory model: start at Shackleton, no destination */
    NAV.OriginId     = 0;     /* Shackleton Base */
    NAV.DestId       = 0xFF;
    NAV.DistToDest_m = 0.0;
    NAV.InitDist_m   = 0.0;
    NAV.TrajActive   = false;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&NAV.CmdPipe, NAV_PIPE_DEPTH, NAV_PIPE_NAME);

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_NAV_CMD_MID),              NAV.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_IMU_DATA_TLM_MID),         NAV.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_ALT_DATA_TLM_MID),         NAV.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_TRN_POSFIX_TLM_MID),       NAV.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_LUNET_BEACON_TLM_MID),     NAV.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),        NAV.CmdPipe); /* 1 Hz */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x03)),     NAV.CmdPipe); /* 40 Hz */

    CFE_MSG_Init(&NAV.StatePkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_NAV_STATE_TLM_MID),
                 sizeof(LA_NAV_StatePkt_t));

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "NAV: Navigation filter initialized — EKF 12-state");

    return CFE_SUCCESS;
}

/* ── Main Loop ───────────────────────────────────────────── */

void NAV_AppMain(void)
{
    CFE_Status_t     status;
    CFE_SB_Buffer_t *SBBufPtr;
    CFE_SB_MsgId_t   MsgId;

    if (NAV_Init() != CFE_SUCCESS) {
        NAV.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&NAV.RunStatus) == true) {
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, NAV.CmdPipe,
                                      CFE_SB_PEND_FOREVER);
        if (status != CFE_SUCCESS) continue;

        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

        switch (CFE_SB_MsgIdToValue(MsgId)) {

        /* ── 40 Hz Wakeup: propagate + publish ── */
        case LA_SCH_WAKEUP_MID(0x03):
            NAV_Propagate(NAV_DT_NOMINAL);

            /* Process any fresh measurement updates */
            if (NAV.IMU_fresh)    { NAV_UpdateIMU();       NAV.IMU_fresh    = false; }
            if (NAV.ALT_fresh)    { NAV_UpdateAltimeter(); NAV.ALT_fresh    = false; }
            if (NAV.TRN_fresh)    { NAV_UpdateTRN();       NAV.TRN_fresh    = false; }
            if (NAV.Beacon_fresh) { NAV_UpdateBeacon();    NAV.Beacon_fresh = false; }

            NAV_UpdateTrajectory(NAV_DT_NOMINAL);
            NAV_AssessHealth();
            NAV_SendStatePkt();
            NAV.CycleCount++;
            break;

        /* ── Sensor data arrivals ── */
        case LA_IMU_DATA_TLM_MID:
            memcpy(&NAV.LastIMU, SBBufPtr, sizeof(LA_IMU_DataPkt_t));
            NAV.IMU_fresh = true;
            if (NAV.NavMode == 0) NAV.NavMode = 1; /* Got IMU: upgrade */
            break;

        case LA_ALT_DATA_TLM_MID:
            memcpy(&NAV.LastAlt, SBBufPtr, sizeof(LA_ALT_DataPkt_t));
            NAV.ALT_fresh = true;
            break;

        case LA_TRN_POSFIX_TLM_MID:
            memcpy(&NAV.LastTRN, SBBufPtr, sizeof(LA_TRN_HazardPkt_t));
            NAV.TRN_fresh = true;
            if (NAV.NavMode < 2) NAV.NavMode = 2; /* Full fusion */
            break;

        case LA_LUNET_BEACON_TLM_MID:
            memcpy(&NAV.LastBeacon, SBBufPtr, sizeof(LA_LUNET_BeaconPkt_t));
            NAV.Beacon_fresh = true;
            break;

        case LA_MM_PHASE_TLM_MID:
            memcpy(&NAV.LastPhase, SBBufPtr, sizeof(LA_MM_PhasePkt_t));
            NAV.Phase_fresh = true;
            break;

        case LA_NAV_CMD_MID:
            NAV_ProcessCommand(&SBBufPtr->Msg);
            break;

        default:
            break;
        }
    }

    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}

/* ── EKF Propagation (simplified) ────────────────────────── */

static void NAV_Propagate(double dt)
{
    /*
     * State propagation using IMU data as control input:
     *   pos += vel * dt
     *   vel += (specific_force - gravity) * dt
     *   att_err → 0 (folded into reference quaternion)
     *   bias ≈ constant (random walk)
     *
     * This is a simplified propagation.  A full implementation
     * would use the proper rotation matrices and integrate the
     * quaternion kinematics equation.
     */

    /* Position update */
    NAV.state[0] += NAV.state[3] * dt;
    NAV.state[1] += NAV.state[4] * dt;
    NAV.state[2] += NAV.state[5] * dt;

    /* Velocity update: accel from IMU - lunar gravity (simplified) */
    if (NAV.IMU_fresh) {
        NAV.state[3] += NAV.LastIMU.SpecForce_mps2.x * dt;
        NAV.state[4] += NAV.LastIMU.SpecForce_mps2.y * dt;
        NAV.state[5] += (NAV.LastIMU.SpecForce_mps2.z + 1.625) * dt;
    }

    /* Covariance propagation: P = F*P*F' + Q (simplified diagonal) */
    for (int i = 0; i < 3; i++) {
        NAV.P[i][i]     += NAV.P[i+3][i+3] * dt * dt
                         + NAV_ACCEL_NOISE * NAV_ACCEL_NOISE * dt;
        NAV.P[i+3][i+3] += NAV_ACCEL_NOISE * NAV_ACCEL_NOISE * dt;
        NAV.P[i+6][i+6] += NAV_GYRO_NOISE * NAV_GYRO_NOISE * dt;
        NAV.P[i+9][i+9] += NAV_BIAS_DRIFT * NAV_BIAS_DRIFT * dt;
    }
}

/* ── Measurement Updates (simplified scalar updates) ─────── */

static void NAV_UpdateIMU(void)
{
    /*
     * IMU provides attitude rate and specific force.
     * In a full EKF this feeds the propagation (already done above).
     * Here we also use angular rate to update gyro bias estimate.
     */
    for (int i = 0; i < 3; i++) {
        double meas_rate = (&NAV.LastIMU.AngRate_rps.x)[i];
        double pred_rate = NAV.state[9+i]; /* predicted bias */
        double innov = meas_rate - pred_rate;
        double S = NAV.P[9+i][9+i] + NAV_GYRO_NOISE * NAV_GYRO_NOISE;
        double K = NAV.P[9+i][9+i] / S;
        NAV.state[9+i] += K * innov;
        NAV.P[9+i][9+i] *= (1.0 - K);
    }
}

static void NAV_UpdateAltimeter(void)
{
    /*
     * Altimeter provides AGL measurement → updates z-position.
     * R_alt ~ 0.1 m for lidar, 1.0 m for radar
     */
    if (!NAV.LastAlt.LidarValid && !NAV.LastAlt.RadarValid) return;

    double R = NAV.LastAlt.LidarValid ? 0.01 : 1.0; /* measurement noise */
    double meas = NAV.LastAlt.AGL_m;
    double innov = meas - NAV.state[2];
    double S = NAV.P[2][2] + R;
    double K = NAV.P[2][2] / S;

    NAV.state[2] += K * innov;
    NAV.P[2][2]  *= (1.0 - K);

    /* Also update vertical velocity from alt rate */
    double R_rate = NAV.LastAlt.LidarValid ? 0.1 : 2.0;
    double innov_rate = NAV.LastAlt.VertRate_mps - NAV.state[5];
    double S_rate = NAV.P[5][5] + R_rate;
    double K_rate = NAV.P[5][5] / S_rate;

    NAV.state[5] += K_rate * innov_rate;
    NAV.P[5][5]  *= (1.0 - K_rate);
}

static void NAV_UpdateTRN(void)
{
    /*
     * TRN provides absolute position fix from terrain matching.
     * R_trn ~ 5 m
     */
    if (!NAV.LastTRN.FixValid) return;

    double R = 25.0; /* 5 m 1-sigma → R = 25 m^2 */
    for (int i = 0; i < 3; i++) {
        double meas = (&NAV.LastTRN.PosFix_MCMF_m.x)[i];
        double innov = meas - NAV.state[i];
        double S = NAV.P[i][i] + R;
        double K = NAV.P[i][i] / S;
        NAV.state[i] += K * innov;
        NAV.P[i][i]  *= (1.0 - K);
    }
}

static void NAV_UpdateBeacon(void)
{
    /*
     * LUNET beacon provides range measurement.
     * R_beacon ~ 1 m
     * Simplified: use range to update position toward beacon.
     */
    double dx = NAV.LastBeacon.BeaconPos_MCMF_m.x - NAV.state[0];
    double dy = NAV.LastBeacon.BeaconPos_MCMF_m.y - NAV.state[1];
    double dz = NAV.LastBeacon.BeaconPos_MCMF_m.z - NAV.state[2];
    double pred_range = sqrt(dx*dx + dy*dy + dz*dz);

    if (pred_range < 1.0) return; /* Too close, degenerate geometry */

    double innov = NAV.LastBeacon.BeaconRange_m - pred_range;
    double R = 1.0; /* 1 m measurement noise */

    /* Scalar update along range direction */
    for (int i = 0; i < 3; i++) {
        double H_i = -(&dx)[i] / pred_range; /* Range partial */
        double S = NAV.P[i][i] * H_i * H_i + R;
        double K = NAV.P[i][i] * H_i / S;
        NAV.state[i] += K * innov;
        NAV.P[i][i]  *= (1.0 - K * H_i);
    }
}

/* ── Trajectory Model (1D range-to-destination) ─────────── */

/**
 * Simplified 1D range model:
 *  - On transition to POWERED_ASC, compute initial great-circle
 *    distance to the active destination from the home base
 *    (Shackleton, ID 0) using the destination table.
 *  - During powered and coast phases, decrement distance by
 *    the integrated speed each cycle (speed * dt).
 *  - Clamp at zero on arrival.  Reset on LANDED/DOCKED/SAFED.
 *
 * The destination ID is inferred from MM's MissionType:
 *  - Surface missions: destination is set by ground command
 *    (future: NAV_SET_DEST cmd).  For now defaults to ID 0.
 *  - Orbital missions: destination is AEGIS STATION (ID 1).
 */
static void NAV_UpdateTrajectory(double dt)
{
    uint8_t phase = NAV.LastPhase.CurrentPhase;

    /* Detect mission start: transition into POWERED_ASC */
    if (NAV.Phase_fresh) {
        NAV.Phase_fresh = false;

        /* Assign destination on first ascent if not set */
        if (phase == LA_PHASE_POWERED_ASC && !NAV.TrajActive) {
            if (NAV.DestId == 0xFF) {
                /* Orbital → Aegis Station; surface → default home */
                NAV.DestId = (NAV.LastPhase.MissionType == 1) ? 1 : 0;
            }

            const LA_Destination_t *origin = LA_Dest_Lookup(NAV.OriginId);
            const LA_Destination_t *dest   = LA_Dest_Lookup(NAV.DestId);
            if (origin && dest) {
                if (dest->type == LA_DEST_TYPE_ORBITAL) {
                    NAV.InitDist_m  = dest->alt_m;
                } else {
                    NAV.InitDist_m  = LA_Dest_SurfaceDistance_m(
                        origin->lat_deg, origin->lon_deg,
                        dest->lat_deg,   dest->lon_deg);
                }
                NAV.DistToDest_m = NAV.InitDist_m;
                NAV.TrajActive   = true;

                CFE_EVS_SendEvent(10, CFE_EVS_EventType_INFORMATION,
                    "NAV: Trajectory %s -> %s, range %.0f m",
                    origin->name, dest->name, NAV.InitDist_m);
            }
        }

        /* On landing: update origin to where we arrived */
        if (phase == LA_PHASE_LANDED && NAV.DestId != 0xFF) {
            NAV.OriginId = NAV.DestId;
            CFE_EVS_SendEvent(11, CFE_EVS_EventType_INFORMATION,
                "NAV: Arrived at dest %u — origin updated", NAV.OriginId);
        }
        if (phase == LA_PHASE_DOCKED) {
            NAV.OriginId = 1; /* Aegis Station */
        }

        /* Reset trajectory on terminal phases */
        if (phase == LA_PHASE_LANDED  ||
            phase == LA_PHASE_DOCKED  ||
            phase == LA_PHASE_SAFED   ||
            phase == LA_PHASE_PREFLIGHT) {
            NAV.TrajActive   = false;
            NAV.DistToDest_m = 0.0;
            NAV.DestId       = 0xFF;
        }
    }

    /* Decrement range during active flight */
    if (NAV.TrajActive) {
        double speed = sqrt(NAV.state[3] * NAV.state[3] +
                            NAV.state[4] * NAV.state[4] +
                            NAV.state[5] * NAV.state[5]);
        NAV.DistToDest_m -= speed * dt;
        if (NAV.DistToDest_m < 0.0) {
            NAV.DistToDest_m = 0.0;
        }
    }
}

/* ── Health Assessment ───────────────────────────────────── */

static void NAV_AssessHealth(void)
{
    double pos_unc = sqrt(NAV.P[0][0] + NAV.P[1][1] + NAV.P[2][2]);
    double vel_unc = sqrt(NAV.P[3][3] + NAV.P[4][4] + NAV.P[5][5]);

    if (pos_unc > 1000.0 || vel_unc > 10.0) {
        NAV.NavHealth = 2; /* lost */
    } else if (pos_unc > 100.0 || vel_unc > 2.0) {
        NAV.NavHealth = 1; /* degraded */
    } else {
        NAV.NavHealth = 0; /* OK */
    }
}

/* ── Telemetry ───────────────────────────────────────────── */

static void NAV_SendStatePkt(void)
{
    NAV.StatePkt.Pos_MCMF_m.x  = NAV.state[0];
    NAV.StatePkt.Pos_MCMF_m.y  = NAV.state[1];
    NAV.StatePkt.Pos_MCMF_m.z  = NAV.state[2];
    NAV.StatePkt.Vel_MCMF_mps.x = NAV.state[3];
    NAV.StatePkt.Vel_MCMF_mps.y = NAV.state[4];
    NAV.StatePkt.Vel_MCMF_mps.z = NAV.state[5];
    NAV.StatePkt.AngRate_rps.x  = NAV.LastIMU.AngRate_rps.x - NAV.state[9];
    NAV.StatePkt.AngRate_rps.y  = NAV.LastIMU.AngRate_rps.y - NAV.state[10];
    NAV.StatePkt.AngRate_rps.z  = NAV.LastIMU.AngRate_rps.z - NAV.state[11];
    NAV.StatePkt.Att_BodyMCMF   = NAV.att_ref;
    NAV.StatePkt.PosUncert_m    = sqrt(NAV.P[0][0] + NAV.P[1][1] + NAV.P[2][2]);
    NAV.StatePkt.VelUncert_mps  = sqrt(NAV.P[3][3] + NAV.P[4][4] + NAV.P[5][5]);
    NAV.StatePkt.AttUncert_rad  = sqrt(NAV.P[6][6] + NAV.P[7][7] + NAV.P[8][8]);
    NAV.StatePkt.DistToDest_m   = NAV.DistToDest_m;
    NAV.StatePkt.NavMode        = NAV.NavMode;
    NAV.StatePkt.NavHealth      = NAV.NavHealth;
    NAV.StatePkt.DestId         = NAV.DestId;

    CFE_SB_TransmitMsg(&NAV.StatePkt.TlmHdr.Msg, true);
}

/* ── Command Processing ─────────────────────────────────── */

static void NAV_ProcessCommand(const CFE_MSG_Message_t *MsgPtr)
{
    CFE_MSG_FcnCode_t fc;
    CFE_MSG_GetFcnCode(MsgPtr, &fc);

    switch (fc) {
    case NAV_NOOP_CC:
        NAV.CmdCount++;
        break;

    case NAV_RESET_CC:
        NAV.CmdCount = 0;
        NAV.ErrCount = 0;
        break;

    case NAV_SET_DEST_CC: {
        /* Payload: 1 byte destination ID */
        const uint8_t *payload = (const uint8_t *)MsgPtr + sizeof(CFE_MSG_CommandHeader_t);
        uint8_t dest_id = payload[0];
        const LA_Destination_t *dest = LA_Dest_Lookup(dest_id);
        if (dest) {
            NAV.DestId = dest_id;

            /* Pre-compute distance from current origin for display */
            const LA_Destination_t *origin = LA_Dest_Lookup(NAV.OriginId);
            if (origin && !NAV.TrajActive) {
                if (dest->type == LA_DEST_TYPE_ORBITAL) {
                    NAV.DistToDest_m = dest->alt_m;
                } else {
                    NAV.DistToDest_m = LA_Dest_SurfaceDistance_m(
                        origin->lat_deg, origin->lon_deg,
                        dest->lat_deg,   dest->lon_deg);
                }
            }

            NAV.CmdCount++;
            CFE_EVS_SendEvent(12, CFE_EVS_EventType_INFORMATION,
                "NAV: SET_DEST %u (%s) — %.0f m from %s",
                dest_id, dest->name, NAV.DistToDest_m,
                origin ? origin->name : "?");
        } else {
            NAV.ErrCount++;
            CFE_EVS_SendEvent(13, CFE_EVS_EventType_ERROR,
                "NAV: SET_DEST rejected — unknown ID %u", dest_id);
        }
        break;
    }

    default:
        NAV.ErrCount++;
        break;
    }
}
