/**
 * @file la_msg_structs.h
 * @brief Luna-Aegis Short Hopper — Packet Structure Definitions
 *
 * Every telemetry and command packet exchanged on the cFE
 * Software Bus.  Structures are packed for CCSDS compliance.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#ifndef LA_MSG_STRUCTS_H
#define LA_MSG_STRUCTS_H

#ifdef SIM_MODE
  /* Standalone build: use our stubs */
  #include "../cfe_hdr/cfe_sb_types.h"
#else
  /* cFS integrated build: use real cFE headers */
  #include "cfe.h"
#endif


/* ── Common Types ────────────────────────────────────────── */

typedef struct {
    double x;
    double y;
    double z;
} LA_Vec3d_t;

typedef struct {
    double q0;   /* scalar */
    double q1;
    double q2;
    double q3;
} LA_Quat_t;

/* ══════════════════════════════════════════════════════════
 * TIER 2 — GN&C PACKETS
 * ══════════════════════════════════════════════════════════ */

/**
 * IMU_MGR → SB @ 40 Hz
 * Fused inertial data from FOG/RLG primary + MEMS backup
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    LA_Quat_t   Attitude;         /* Body-to-MCMF quaternion        */
    LA_Vec3d_t  AngRate_rps;      /* Body angular rate (rad/s)       */
    LA_Vec3d_t  SpecForce_mps2;   /* Specific force (m/s^2)          */
    uint8_t     SourceSelect;     /* 0=FOG/RLG, 1=MEMS, 2=blended   */
    uint8_t     StatusFlags;      /* Bit 0: FOG valid, Bit 1: MEMS   */
    uint16_t    Padding;
} LA_IMU_DataPkt_t;

/**
 * ALT_MGR → SB @ 10 Hz
 * Fused altitude data from lidar + radar altimeter
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    double      AGL_m;            /* Above ground level (m)          */
    double      VertRate_mps;     /* Vertical rate (m/s, down +)     */
    double      TerrainSlope_deg; /* Local terrain slope estimate    */
    uint8_t     LidarValid;
    uint8_t     RadarValid;
    uint16_t    Padding;
} LA_ALT_DataPkt_t;

/**
 * NAV → SB @ 40 Hz
 * EKF state estimate — primary navigation solution
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    LA_Vec3d_t  Pos_MCMF_m;      /* Position in Moon-centered frame */
    LA_Vec3d_t  Vel_MCMF_mps;    /* Velocity in Moon-centered frame */
    LA_Quat_t   Att_BodyMCMF;    /* Attitude quaternion              */
    LA_Vec3d_t  AngRate_rps;      /* Angular rate (rad/s)             */
    double      PosUncert_m;      /* 1-sigma position uncertainty     */
    double      VelUncert_mps;    /* 1-sigma velocity uncertainty     */
    double      AttUncert_rad;    /* 1-sigma attitude uncertainty     */
    uint8_t     NavMode;          /* 0=propagate, 1=IMU+alt, 2=full  */
    uint8_t     NavHealth;        /* 0=OK, 1=degraded, 2=lost        */
    uint16_t    Padding;
} LA_NAV_StatePkt_t;

/**
 * GDN → SB @ 10 Hz
 * Guidance commands to propulsion and ACS
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    double      ThrustCmd_N;      /* Commanded thrust (N)             */
    double      GimbalPitch_rad;  /* Commanded gimbal pitch           */
    double      GimbalYaw_rad;    /* Commanded gimbal yaw             */
    double      ThrottleCmd_pct;  /* Throttle 0–100%                  */
    uint8_t     GuidancePhase;    /* 0=coast, 1=powered, 2=hover      */
    uint8_t     Padding[3];
} LA_GDN_CmdPkt_t;

/**
 * ACS → SB @ 40 Hz
 * Actuator commands to gimbal and RCS hardware controllers
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    double      GimbalPitchCmd_rad;
    double      GimbalYawCmd_rad;
    double      RCS_Roll_Nm;      /* Roll torque command              */
    double      RCS_Pitch_Nm;     /* Pitch torque (supplement gimbal) */
    double      RCS_Yaw_Nm;       /* Yaw torque (supplement gimbal)   */
    uint8_t     RCS_FiringMask;   /* Bitfield: which thrusters fire   */
    uint8_t     Padding[3];
} LA_ACS_ActuatorCmdPkt_t;

/**
 * TRN → SB @ 2 Hz
 * Terrain-relative navigation hazard assessment
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    LA_Vec3d_t  PosFix_MCMF_m;   /* Terrain-matched position fix     */
    double      FixUncert_m;      /* Fix uncertainty (m)              */
    double      HazardDist_m;     /* Distance to nearest hazard       */
    double      MaxSlope_deg;     /* Max slope in landing zone        */
    uint8_t     HazardCount;      /* Hazards detected in FOV          */
    uint8_t     FixValid;
    uint16_t    Padding;
} LA_TRN_HazardPkt_t;

/* ══════════════════════════════════════════════════════════
 * TIER 3 — MISSION MANAGEMENT PACKETS
 * ══════════════════════════════════════════════════════════ */

/**
 * Flight phases for Mission Manager FSM
 */
typedef enum {
    LA_PHASE_PREFLIGHT   = 0,
    LA_PHASE_POWERED_ASC = 1,
    LA_PHASE_COAST       = 2,
    LA_PHASE_POWERED_DES = 3,
    LA_PHASE_HOVER       = 4,
    LA_PHASE_TERMINAL    = 5,
    LA_PHASE_LANDED      = 6,
    LA_PHASE_ABORT       = 7,
    LA_PHASE_SAFED       = 8,
    LA_PHASE_COUNT
} LA_FlightPhase_t;

/**
 * MM → SB @ 1 Hz
 * Current flight phase and mission status
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    uint8_t     CurrentPhase;     /* LA_FlightPhase_t                 */
    uint8_t     PreviousPhase;
    uint8_t     AbortReason;      /* 0=none, 1=prop, 2=nav, 3=cmd    */
    uint8_t     MissionMode;      /* 0=auto, 1=manual, 2=abort-seq   */
    double      MET_s;            /* Mission elapsed time (s)         */
    double      DvRemaining_mps;  /* Estimated remaining Δv           */
    double      PropRemaining_kg; /* Propellant remaining (kg)        */
    uint32_t    PhaseTimer_s;     /* Time in current phase            */
} LA_MM_PhasePkt_t;

/**
 * PROP → SB @ 10 Hz
 * Propulsion system status
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    double      ChamberPress_kPa; /* Engine chamber pressure          */
    double      OxTankPress_kPa;  /* LOX tank pressure                */
    double      FuelTankPress_kPa;/* LH2 tank pressure                */
    double      OxMass_kg;        /* LOX remaining                    */
    double      FuelMass_kg;      /* LH2 remaining                    */
    double      MixtureRatio;     /* Actual O/F ratio                 */
    double      ThrustEst_N;      /* Estimated thrust from Pc         */
    uint8_t     EngineState;      /* 0=off, 1=chill, 2=start, 3=run  */
    uint8_t     ValveStates;      /* Bitfield: main, ign, purge       */
    uint16_t    Padding;
} LA_PROP_StatusPkt_t;

/**
 * LG → SB @ 1 Hz
 * Landing gear status
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    uint8_t     DeployState;      /* 0=stowed, 1=transit, 2=deployed  */
    uint8_t     LegStatus[4];     /* Per-leg: 0=OK, 1=warn, 2=fault  */
    uint8_t     TouchdownDetect;  /* Bitfield: which legs have contact*/
    uint16_t    Padding;
    double      LoadLeg1_N;       /* Axial load, leg 1                */
    double      LoadLeg2_N;
    double      LoadLeg3_N;
    double      LoadLeg4_N;
} LA_LG_StatusPkt_t;

/**
 * DOCK → SB @ 1 Hz
 * Docking system status
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    uint8_t     CollarState;      /* 0=retracted, 1=extending, 2=soft, 3=hard */
    uint8_t     SealPressOK;      /* Seal pressure check pass         */
    uint8_t     LatchState;       /* 0=open, 1=closing, 2=locked      */
    uint8_t     MateTarget;       /* 0=none, 1=station, 2=hab, 3=rover */
    double      TunnelPress_kPa;  /* Tunnel pressure                  */
    double      DeltaP_kPa;       /* Differential across seal         */
} LA_DOCK_StatusPkt_t;

/* ══════════════════════════════════════════════════════════
 * TIER 4 — VEHICLE SYSTEMS PACKETS
 * ══════════════════════════════════════════════════════════ */

/**
 * EPS → SB @ 1 Hz
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    double      BusVoltage_V;     /* Main power bus voltage           */
    double      BusCurrent_A;     /* Bus current draw                 */
    double      BattSOC_pct;      /* State of charge (%)              */
    double      BattTemp_C;       /* Battery pack temperature         */
    double      SolarInput_W;     /* Solar backup power (if any)      */
    uint8_t     LoadShedLevel;    /* 0=none, 1=non-essential, 2=crit  */
    uint8_t     BattHealth;       /* 0=OK, 1=degraded, 2=fault        */
    uint16_t    Padding;
} LA_EPS_StatusPkt_t;

/**
 * TCS → SB @ 0.1 Hz
 * Thermal control / SSSRA status
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    double      RadiatorTemp_C;   /* SSSRA radiator avg temp          */
    double      SolarArrayTemp_C; /* PV array temperature             */
    double      CabinTemp_C;      /* Cabin air temperature            */
    double      PropLineTemp_C;   /* Cryo feed line temperature       */
    double      HeatRejection_W;  /* Current heat rejection rate      */
    uint8_t     HeaterZoneMask;   /* Which heater zones are active    */
    uint8_t     ThermalMode;      /* 0=passive, 1=active, 2=survival  */
    uint16_t    Padding;
} LA_TCS_StatusPkt_t;

/**
 * LSS → SB @ 1 Hz
 * Life support system status
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    double      CabinPress_kPa;   /* Cabin total pressure             */
    double      O2_Partial_kPa;   /* Oxygen partial pressure          */
    double      CO2_Partial_kPa;  /* CO2 partial pressure             */
    double      Humidity_pct;     /* Relative humidity                 */
    double      CabinTemp_C;      /* Cabin temperature                 */
    double      O2_Remaining_hr;  /* Hours of O2 remaining            */
    double      CO2_Scrub_pct;    /* Scrubber capacity remaining      */
    uint8_t     CrewCount;        /* Current crew aboard              */
    uint8_t     LSSMode;          /* 0=nominal, 1=low-power, 2=emerg */
    uint16_t    Padding;
} LA_LSS_StatusPkt_t;

/**
 * COMM → SB @ 1 Hz
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    double      UplinkSNR_dB;     /* S-band uplink SNR                */
    double      DownlinkSNR_dB;   /* S-band downlink SNR              */
    double      HGA_Az_deg;       /* High-gain antenna azimuth        */
    double      HGA_El_deg;       /* High-gain antenna elevation      */
    uint8_t     LinkState;        /* 0=no link, 1=UHF, 2=S-band, 3=HGA */
    uint8_t     DataRate;         /* 0=low, 1=med, 2=high             */
    uint16_t    Padding;
} LA_COMM_LinkPkt_t;

/**
 * LUNET → SB @ 1 Hz
 * LUNET beacon data from surface infrastructure node
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    LA_Vec3d_t  BeaconPos_MCMF_m; /* Beacon position in MCMF         */
    double      BeaconRange_m;     /* Measured range to beacon         */
    double      BeaconBearing_rad; /* Bearing to beacon                */
    uint8_t     BeaconID;          /* Which LUNET node                 */
    uint8_t     SignalQuality;     /* 0–255 quality metric             */
    uint8_t     RefuelReady;       /* Node refuel port status          */
    uint8_t     Padding;
} LA_LUNET_BeaconPkt_t;

/* ══════════════════════════════════════════════════════════
 * TIER 5 — HEALTH & SAFETY PACKETS
 * ══════════════════════════════════════════════════════════ */

/**
 * HS → SB @ 1 Hz
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    uint32_t    CPULoad_pct_x100; /* CPU load * 100                   */
    uint32_t    MemFree_bytes;    /* Free memory                      */
    uint16_t    AppWatchdogMask;  /* Bit per app: 1=alive, 0=stale    */
    uint16_t    CritEventCount;   /* Critical events since boot       */
    uint8_t     RedundState;      /* 0=primary, 1=backup, 2=split     */
    uint8_t     RebootCount;
    uint16_t    Padding;
} LA_HS_AlertPkt_t;

/**
 * LC → SB @ 1 Hz
 */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TlmHdr;
    uint32_t    WP_Results;       /* Watchpoint pass/fail bitfield    */
    uint32_t    AP_Results;       /* Actionpoint triggered bitfield   */
    uint16_t    ActiveWPCount;
    uint16_t    ActiveAPCount;
    uint8_t     LCState;          /* 0=active, 1=passive, 2=disabled  */
    uint8_t     Padding[3];
} LA_LC_ActionPkt_t;


#endif /* LA_MSG_STRUCTS_H */
