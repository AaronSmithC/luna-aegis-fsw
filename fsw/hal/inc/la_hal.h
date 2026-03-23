/**
 * @file la_hal.h
 * @brief Luna-Aegis Short Hopper — Hardware Abstraction Layer API
 *
 * Clean boundary between cFS mission apps and dedicated HW
 * controllers.  Each subsystem gets init/read/write/status
 * functions.  Two implementations exist:
 *   - hal/src/  — flight hardware drivers (CAN/LVDS/SPI)
 *   - hal/sim/  — desktop simulation stubs
 *
 * The HAL is the ONLY code that talks to hardware registers,
 * bus controllers, or simulation models.  cFS apps call ONLY
 * these functions.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#ifndef LA_HAL_H
#define LA_HAL_H

#include <stdint.h>
#include <stdbool.h>

/* ── Return Codes ────────────────────────────────────────── */

typedef int32_t HAL_Status_t;

#define HAL_OK             ((HAL_Status_t) 0)
#define HAL_ERR_TIMEOUT    ((HAL_Status_t)-1)
#define HAL_ERR_BUS        ((HAL_Status_t)-2)
#define HAL_ERR_RANGE      ((HAL_Status_t)-3)
#define HAL_ERR_NOT_INIT   ((HAL_Status_t)-4)
#define HAL_ERR_HW_FAULT   ((HAL_Status_t)-5)

/* ══════════════════════════════════════════════════════════
 * ENGINE GIMBAL (LVDS bus to servo drive)
 * ══════════════════════════════════════════════════════════ */

HAL_Status_t HAL_Gimbal_Init(void);
HAL_Status_t HAL_Gimbal_SetAngles(double pitch_rad, double yaw_rad);
HAL_Status_t HAL_Gimbal_GetAngles(double *pitch_rad, double *yaw_rad);
HAL_Status_t HAL_Gimbal_GetStatus(uint8_t *health, double *temp_C);

/* ══════════════════════════════════════════════════════════
 * RCS VALVE DRIVERS (discrete I/O to solenoid bank)
 * ══════════════════════════════════════════════════════════ */

#define HAL_RCS_THRUSTER_COUNT  12

typedef struct {
    uint8_t  firing_mask;          /* Bitfield: which thrusters open  */
    uint16_t pulse_ms[HAL_RCS_THRUSTER_COUNT]; /* Pulse duration each */
} HAL_RCS_Cmd_t;

HAL_Status_t HAL_RCS_Init(void);
HAL_Status_t HAL_RCS_Fire(const HAL_RCS_Cmd_t *cmd);
HAL_Status_t HAL_RCS_GetValveState(uint16_t *cycle_counts,
                                   uint8_t *health_mask);

/* ══════════════════════════════════════════════════════════
 * BATTERY MANAGEMENT UNIT (SPI bus)
 * ══════════════════════════════════════════════════════════ */

typedef struct {
    double  bus_voltage_V;
    double  bus_current_A;
    double  soc_pct;
    double  pack_temp_C;
    uint8_t cell_balance_active;
    uint8_t fault_flags;
} HAL_BMU_Tlm_t;

HAL_Status_t HAL_BMU_Init(void);
HAL_Status_t HAL_BMU_Read(HAL_BMU_Tlm_t *tlm);
HAL_Status_t HAL_BMU_SetLoadShed(uint8_t level);

/* ══════════════════════════════════════════════════════════
 * CRYO TANK PRESSURE CONTROLLER (serial/SPI)
 * ══════════════════════════════════════════════════════════ */

typedef struct {
    double  ox_press_kPa;
    double  fuel_press_kPa;
    double  ox_temp_K;
    double  fuel_temp_K;
    double  ox_mass_kg;
    double  fuel_mass_kg;
    uint8_t vent_valve_state;      /* 0=closed, 1=venting             */
    uint8_t relief_active;
} HAL_CryoTank_Tlm_t;

HAL_Status_t HAL_CryoTank_Init(void);
HAL_Status_t HAL_CryoTank_Read(HAL_CryoTank_Tlm_t *tlm);
HAL_Status_t HAL_CryoTank_SetVent(bool open);

/* ══════════════════════════════════════════════════════════
 * ENGINE CONTROLLER (CAN bus)
 * ══════════════════════════════════════════════════════════ */

typedef enum {
    HAL_ENGINE_OFF       = 0,
    HAL_ENGINE_CHILL     = 1,
    HAL_ENGINE_IGNITION  = 2,
    HAL_ENGINE_RUNNING   = 3,
    HAL_ENGINE_SHUTDOWN  = 4,
    HAL_ENGINE_FAULT     = 5,
} HAL_EngineState_t;

typedef struct {
    double   chamber_press_kPa;
    double   mixture_ratio;
    double   thrust_est_N;
    uint8_t  state;               /* HAL_EngineState_t               */
    uint8_t  valve_positions;     /* Bitfield                        */
    uint16_t igniter_count;
} HAL_Engine_Tlm_t;

HAL_Status_t HAL_Engine_Init(void);
HAL_Status_t HAL_Engine_Command(HAL_EngineState_t cmd,
                                double throttle_pct);
HAL_Status_t HAL_Engine_Read(HAL_Engine_Tlm_t *tlm);

/* ══════════════════════════════════════════════════════════
 * LANDING GEAR ACTUATORS (CAN bus to motor controllers)
 * ══════════════════════════════════════════════════════════ */

typedef enum {
    HAL_LG_STOWED    = 0,
    HAL_LG_DEPLOYING = 1,
    HAL_LG_DEPLOYED  = 2,
    HAL_LG_RETRACTING= 3,
    HAL_LG_FAULT     = 4,
} HAL_LG_State_t;

typedef struct {
    uint8_t  state;                /* HAL_LG_State_t                  */
    double   load_N[4];           /* Axial load per leg               */
    uint8_t  touchdown[4];        /* Contact sensor per leg           */
    double   temp_C[4];           /* Strut temperature per leg        */
} HAL_LG_Tlm_t;

HAL_Status_t HAL_LG_Init(void);
HAL_Status_t HAL_LG_Deploy(void);
HAL_Status_t HAL_LG_Retract(void);
HAL_Status_t HAL_LG_Read(HAL_LG_Tlm_t *tlm);

/* ══════════════════════════════════════════════════════════
 * DOCKING COLLAR ACTUATORS (CAN bus)
 * ══════════════════════════════════════════════════════════ */

typedef enum {
    HAL_DOCK_RETRACTED  = 0,
    HAL_DOCK_EXTENDING  = 1,
    HAL_DOCK_SOFT_DOCK  = 2,
    HAL_DOCK_HARD_DOCK  = 3,
    HAL_DOCK_FAULT      = 4,
} HAL_DockState_t;

typedef struct {
    uint8_t  state;               /* HAL_DockState_t                  */
    double   seal_press_kPa;
    double   tunnel_press_kPa;
    double   delta_p_kPa;
    uint8_t  latch_engaged;       /* Number of latches engaged        */
} HAL_Dock_Tlm_t;

HAL_Status_t HAL_Dock_Init(void);
HAL_Status_t HAL_Dock_Extend(void);
HAL_Status_t HAL_Dock_Retract(void);
HAL_Status_t HAL_Dock_HardLatch(void);
HAL_Status_t HAL_Dock_Release(void);
HAL_Status_t HAL_Dock_Read(HAL_Dock_Tlm_t *tlm);

/* ══════════════════════════════════════════════════════════
 * HEATER CONTROLLERS (discrete I/O)
 * ══════════════════════════════════════════════════════════ */

#define HAL_HEATER_ZONE_COUNT  8

HAL_Status_t HAL_Heater_Init(void);
HAL_Status_t HAL_Heater_SetZone(uint8_t zone, bool enable,
                                uint8_t duty_pct);
HAL_Status_t HAL_Heater_GetTemps(double temps_C[HAL_HEATER_ZONE_COUNT]);

/* ══════════════════════════════════════════════════════════
 * ANTENNA GIMBAL (LVDS bus)
 * ══════════════════════════════════════════════════════════ */

HAL_Status_t HAL_Antenna_Init(void);
HAL_Status_t HAL_Antenna_Point(double az_deg, double el_deg);
HAL_Status_t HAL_Antenna_GetPointing(double *az_deg, double *el_deg);

/* ══════════════════════════════════════════════════════════
 * IMU SENSOR INTERFACE (high-speed serial)
 * ══════════════════════════════════════════════════════════ */

typedef struct {
    double  accel_mps2[3];        /* X, Y, Z specific force          */
    double  gyro_rps[3];          /* X, Y, Z angular rate            */
    double  temp_C;
    uint8_t status;               /* 0=OK, 1=degraded                */
} HAL_IMU_Raw_t;

HAL_Status_t HAL_IMU_Init(void);
HAL_Status_t HAL_IMU_ReadPrimary(HAL_IMU_Raw_t *data);
HAL_Status_t HAL_IMU_ReadBackup(HAL_IMU_Raw_t *data);

/* ══════════════════════════════════════════════════════════
 * ALTIMETER INTERFACE (serial)
 * ══════════════════════════════════════════════════════════ */

typedef struct {
    double  range_m;
    double  rate_mps;
    uint8_t valid;
} HAL_Altimeter_Raw_t;

HAL_Status_t HAL_Altimeter_Init(void);
HAL_Status_t HAL_Altimeter_ReadLidar(HAL_Altimeter_Raw_t *data);
HAL_Status_t HAL_Altimeter_ReadRadar(HAL_Altimeter_Raw_t *data);

/* ══════════════════════════════════════════════════════════
 * MASTER INIT — called once at boot
 * ══════════════════════════════════════════════════════════ */

HAL_Status_t HAL_InitAll(void);

#endif /* LA_HAL_H */
