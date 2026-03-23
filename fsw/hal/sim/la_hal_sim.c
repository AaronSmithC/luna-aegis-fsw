/**
 * @file la_hal_sim.c
 * @brief Luna-Aegis HAL — Simulation Mode Implementation
 *
 * Desktop simulation stubs for all hardware controllers.
 * Returns plausible telemetry values and accepts commands
 * without real hardware.  Used for FSW development, ground
 * console testing, and integration verification.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 */

#include "../inc/la_hal.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

/* ── Sim State ───────────────────────────────────────────── */

static struct {
    /* Gimbal */
    double gimbal_pitch_rad;
    double gimbal_yaw_rad;

    /* Engine */
    HAL_EngineState_t engine_state;
    double throttle_pct;

    /* Cryo tanks (initial: full load) */
    double ox_mass_kg;
    double fuel_mass_kg;

    /* Landing gear */
    HAL_LG_State_t lg_state;

    /* Docking collar */
    HAL_DockState_t dock_state;
    uint32_t        dock_eq_count;  /* Pressure equalization sim counter */

    /* EPS */
    double batt_soc_pct;

    /* Heaters */
    bool   heater_on[HAL_HEATER_ZONE_COUNT];
    uint8_t heater_duty[HAL_HEATER_ZONE_COUNT];

    /* Antenna */
    double ant_az_deg;
    double ant_el_deg;

    bool initialized;
} sim = {0};

/* ── Helpers ─────────────────────────────────────────────── */

static double sim_noise(double scale)
{
    return ((double)rand() / RAND_MAX - 0.5) * 2.0 * scale;
}

/* ══════════════════════════════════════════════════════════ */

HAL_Status_t HAL_InitAll(void)
{
    srand((unsigned)time(NULL));
    memset(&sim, 0, sizeof(sim));

    /* Initial conditions: pre-flight, fully fueled, on surface */
    sim.ox_mass_kg   = 2062.5;   /* LOX: 75% of 2750 kg propellant */
    sim.fuel_mass_kg = 687.5;    /* LH2: 25% (O/F ~3:1)           */
    sim.batt_soc_pct = 98.0;
    sim.lg_state     = HAL_LG_DEPLOYED;
    sim.dock_state   = HAL_DOCK_RETRACTED;
    sim.engine_state = HAL_ENGINE_OFF;
    sim.initialized  = true;

    return HAL_OK;
}

/* ── Engine Gimbal ───────────────────────────────────────── */

HAL_Status_t HAL_Gimbal_Init(void) { return HAL_OK; }

HAL_Status_t HAL_Gimbal_SetAngles(double pitch_rad, double yaw_rad)
{
    if (!sim.initialized) return HAL_ERR_NOT_INIT;
    /* Gimbal range: ±6 degrees */
    if (fabs(pitch_rad) > 0.105 || fabs(yaw_rad) > 0.105)
        return HAL_ERR_RANGE;

    sim.gimbal_pitch_rad = pitch_rad;
    sim.gimbal_yaw_rad   = yaw_rad;
    return HAL_OK;
}

HAL_Status_t HAL_Gimbal_GetAngles(double *pitch_rad, double *yaw_rad)
{
    *pitch_rad = sim.gimbal_pitch_rad + sim_noise(0.0001);
    *yaw_rad   = sim.gimbal_yaw_rad   + sim_noise(0.0001);
    return HAL_OK;
}

HAL_Status_t HAL_Gimbal_GetStatus(uint8_t *health, double *temp_C)
{
    *health = 0; /* OK */
    *temp_C = 35.0 + sim_noise(2.0);
    return HAL_OK;
}

/* ── RCS Valves ──────────────────────────────────────────── */

static uint16_t rcs_cycle_counts[HAL_RCS_THRUSTER_COUNT] = {0};

HAL_Status_t HAL_RCS_Init(void) { return HAL_OK; }

HAL_Status_t HAL_RCS_Fire(const HAL_RCS_Cmd_t *cmd)
{
    if (!sim.initialized) return HAL_ERR_NOT_INIT;
    for (int i = 0; i < HAL_RCS_THRUSTER_COUNT; i++) {
        if (cmd->firing_mask & (1 << i)) {
            rcs_cycle_counts[i]++;
        }
    }
    return HAL_OK;
}

HAL_Status_t HAL_RCS_GetValveState(uint16_t *cycle_counts,
                                   uint8_t *health_mask)
{
    memcpy(cycle_counts, rcs_cycle_counts,
           sizeof(uint16_t) * HAL_RCS_THRUSTER_COUNT);
    *health_mask = 0xFF; /* All healthy in sim */
    return HAL_OK;
}

/* ── Battery Management Unit ─────────────────────────────── */

HAL_Status_t HAL_BMU_Init(void) { return HAL_OK; }

HAL_Status_t HAL_BMU_Read(HAL_BMU_Tlm_t *tlm)
{
    if (!sim.initialized) return HAL_ERR_NOT_INIT;
    tlm->bus_voltage_V       = 28.0 + sim_noise(0.3);
    tlm->bus_current_A       = 12.0 + sim_noise(1.0);
    tlm->soc_pct             = sim.batt_soc_pct;
    tlm->pack_temp_C         = 22.0 + sim_noise(1.0);
    tlm->cell_balance_active = 0;
    tlm->fault_flags         = 0;
    return HAL_OK;
}

HAL_Status_t HAL_BMU_SetLoadShed(uint8_t level)
{
    (void)level;
    return HAL_OK;
}

/* ── Cryo Tanks ──────────────────────────────────────────── */

HAL_Status_t HAL_CryoTank_Init(void) { return HAL_OK; }

HAL_Status_t HAL_CryoTank_Read(HAL_CryoTank_Tlm_t *tlm)
{
    if (!sim.initialized) return HAL_ERR_NOT_INIT;
    tlm->ox_press_kPa   = 250.0 + sim_noise(5.0);
    tlm->fuel_press_kPa = 180.0 + sim_noise(3.0);
    tlm->ox_temp_K      = 90.0  + sim_noise(0.5);
    tlm->fuel_temp_K    = 20.5  + sim_noise(0.2);
    tlm->ox_mass_kg     = sim.ox_mass_kg;
    tlm->fuel_mass_kg   = sim.fuel_mass_kg;
    tlm->vent_valve_state = 0;
    tlm->relief_active    = 0;
    return HAL_OK;
}

HAL_Status_t HAL_CryoTank_SetVent(bool open)
{
    (void)open;
    return HAL_OK;
}

/* ── Engine Controller ───────────────────────────────────── */

HAL_Status_t HAL_Engine_Init(void) { return HAL_OK; }

HAL_Status_t HAL_Engine_Command(HAL_EngineState_t cmd,
                                double throttle_pct)
{
    if (!sim.initialized) return HAL_ERR_NOT_INIT;
    sim.engine_state = cmd;
    sim.throttle_pct = throttle_pct;
    return HAL_OK;
}

HAL_Status_t HAL_Engine_Read(HAL_Engine_Tlm_t *tlm)
{
    if (!sim.initialized) return HAL_ERR_NOT_INIT;
    tlm->state = (uint8_t)sim.engine_state;

    switch (sim.engine_state) {
    case HAL_ENGINE_RUNNING:
        {
            double thrust = 30000.0 * (sim.throttle_pct / 100.0);
            tlm->chamber_press_kPa = 3500.0 * (sim.throttle_pct / 100.0)
                                     + sim_noise(20.0);
            tlm->mixture_ratio     = 6.0 + sim_noise(0.05);
            tlm->thrust_est_N      = thrust + sim_noise(50.0);
            tlm->valve_positions   = 0x07; /* main+ox+fuel open */
        }
        break;

    case HAL_ENGINE_IGNITION:
        /* Pressure ramps up during ignition sequence */
        tlm->chamber_press_kPa = 2000.0 + sim_noise(200.0);
        tlm->mixture_ratio     = 4.0 + sim_noise(1.0);
        tlm->thrust_est_N      = 15000.0 + sim_noise(2000.0);
        tlm->valve_positions   = 0x07; /* valves open for ignition */
        break;

    case HAL_ENGINE_CHILL:
        /* Turbopump chill — small pressure from propellant flow */
        tlm->chamber_press_kPa = 50.0 + sim_noise(10.0);
        tlm->mixture_ratio     = 0.0;
        tlm->thrust_est_N      = 0.0;
        tlm->valve_positions   = 0x03; /* ox+fuel chill valves */
        break;

    case HAL_ENGINE_SHUTDOWN:
        /* Pressure bleeding down */
        tlm->chamber_press_kPa = 100.0 + sim_noise(50.0);
        tlm->mixture_ratio     = 0.0;
        tlm->thrust_est_N      = 0.0;
        tlm->valve_positions   = 0x00;
        break;

    default: /* OFF, FAULT */
        tlm->chamber_press_kPa = 0.0;
        tlm->mixture_ratio     = 0.0;
        tlm->thrust_est_N      = 0.0;
        tlm->valve_positions   = 0x00;
        break;
    }
    tlm->igniter_count = 0;
    return HAL_OK;
}

/* ── Landing Gear ────────────────────────────────────────── */

HAL_Status_t HAL_LG_Init(void) { return HAL_OK; }

HAL_Status_t HAL_LG_Deploy(void)
{
    sim.lg_state = HAL_LG_DEPLOYED;
    return HAL_OK;
}

HAL_Status_t HAL_LG_Retract(void)
{
    sim.lg_state = HAL_LG_STOWED;
    return HAL_OK;
}

HAL_Status_t HAL_LG_Read(HAL_LG_Tlm_t *tlm)
{
    if (!sim.initialized) return HAL_ERR_NOT_INIT;
    tlm->state = (uint8_t)sim.lg_state;
    for (int i = 0; i < 4; i++) {
        tlm->load_N[i]    = (sim.lg_state == HAL_LG_DEPLOYED)
                             ? 3250.0 + sim_noise(50.0) : 0.0;
        tlm->touchdown[i]  = (sim.lg_state == HAL_LG_DEPLOYED) ? 1 : 0;
        tlm->temp_C[i]     = -40.0 + sim_noise(5.0);
    }
    return HAL_OK;
}

/* ── Docking Collar ──────────────────────────────────────── */

HAL_Status_t HAL_Dock_Init(void)    { return HAL_OK; }
HAL_Status_t HAL_Dock_Extend(void)  { sim.dock_state = HAL_DOCK_EXTENDING; return HAL_OK; }
HAL_Status_t HAL_Dock_Retract(void) { sim.dock_state = HAL_DOCK_RETRACTED; return HAL_OK; }
HAL_Status_t HAL_Dock_HardLatch(void) { sim.dock_state = HAL_DOCK_HARD_DOCK; return HAL_OK; }
HAL_Status_t HAL_Dock_Release(void) { sim.dock_state = HAL_DOCK_SOFT_DOCK; return HAL_OK; }

HAL_Status_t HAL_Dock_Read(HAL_Dock_Tlm_t *tlm)
{
    if (!sim.initialized) return HAL_ERR_NOT_INIT;
    tlm->state           = (uint8_t)sim.dock_state;

    switch (sim.dock_state) {
    case HAL_DOCK_HARD_DOCK:
        /* Fully mated — both sides pressurized */
        tlm->seal_press_kPa   = 101.3 + sim_noise(0.2);
        tlm->tunnel_press_kPa = 101.3 + sim_noise(0.2);
        tlm->delta_p_kPa      = fabs(sim_noise(0.1));
        tlm->latch_engaged    = 12;
        break;
    case HAL_DOCK_SOFT_DOCK:
        /* Soft-dock: seal side pressurized, tunnel ramping up
         * Simulates pressure equalization over time.
         * Tunnel press approaches seal press via sim counter. */
        sim.dock_eq_count++;
        {
            double eq_frac = fmin(1.0, (double)sim.dock_eq_count / 8.0);
            tlm->seal_press_kPa   = 101.3 + sim_noise(0.3);
            tlm->tunnel_press_kPa = 101.3 * eq_frac + sim_noise(0.3);
            tlm->delta_p_kPa      = fabs(tlm->seal_press_kPa - tlm->tunnel_press_kPa);
        }
        tlm->latch_engaged = 0;
        break;
    case HAL_DOCK_EXTENDING:
        /* Collar extending — no pressure */
        tlm->seal_press_kPa   = 0.0;
        tlm->tunnel_press_kPa = 0.0;
        tlm->delta_p_kPa      = 0.0;
        tlm->latch_engaged    = 0;
        /* Auto-transition to soft-dock after extending */
        sim.dock_eq_count = 0;
        break;
    default:
        /* RETRACTED / other */
        tlm->seal_press_kPa   = 0.0;
        tlm->tunnel_press_kPa = 0.0;
        tlm->delta_p_kPa      = 0.0;
        tlm->latch_engaged    = 0;
        sim.dock_eq_count      = 0;
        break;
    }
    return HAL_OK;
}

/* ── Heaters ─────────────────────────────────────────────── */

HAL_Status_t HAL_Heater_Init(void) { return HAL_OK; }

HAL_Status_t HAL_Heater_SetZone(uint8_t zone, bool enable,
                                uint8_t duty_pct)
{
    if (zone >= HAL_HEATER_ZONE_COUNT) return HAL_ERR_RANGE;
    sim.heater_on[zone]   = enable;
    sim.heater_duty[zone] = duty_pct;
    return HAL_OK;
}

HAL_Status_t HAL_Heater_GetTemps(double temps_C[HAL_HEATER_ZONE_COUNT])
{
    for (int i = 0; i < HAL_HEATER_ZONE_COUNT; i++) {
        double base = sim.heater_on[i]
                      ? (-20.0 + sim.heater_duty[i] * 0.6)
                      : -80.0;
        temps_C[i] = base + sim_noise(3.0);
    }
    return HAL_OK;
}

/* ── Antenna Gimbal ──────────────────────────────────────── */

HAL_Status_t HAL_Antenna_Init(void) { return HAL_OK; }

HAL_Status_t HAL_Antenna_Point(double az_deg, double el_deg)
{
    sim.ant_az_deg = az_deg;
    sim.ant_el_deg = el_deg;
    return HAL_OK;
}

HAL_Status_t HAL_Antenna_GetPointing(double *az_deg, double *el_deg)
{
    *az_deg = sim.ant_az_deg + sim_noise(0.1);
    *el_deg = sim.ant_el_deg + sim_noise(0.1);
    return HAL_OK;
}

/* ── IMU Sensors ─────────────────────────────────────────── */

HAL_Status_t HAL_IMU_Init(void) { return HAL_OK; }

HAL_Status_t HAL_IMU_ReadPrimary(HAL_IMU_Raw_t *data)
{
    /* Sim: sitting on lunar surface, gravity in -Z body */
    data->accel_mps2[0] = sim_noise(0.001);
    data->accel_mps2[1] = sim_noise(0.001);
    data->accel_mps2[2] = -1.625 + sim_noise(0.001);
    data->gyro_rps[0]   = sim_noise(0.0001);
    data->gyro_rps[1]   = sim_noise(0.0001);
    data->gyro_rps[2]   = sim_noise(0.0001);
    data->temp_C         = 25.0 + sim_noise(1.0);
    data->status         = 0;
    return HAL_OK;
}

HAL_Status_t HAL_IMU_ReadBackup(HAL_IMU_Raw_t *data)
{
    /* MEMS backup — noisier */
    data->accel_mps2[0] = sim_noise(0.01);
    data->accel_mps2[1] = sim_noise(0.01);
    data->accel_mps2[2] = -1.625 + sim_noise(0.01);
    data->gyro_rps[0]   = sim_noise(0.001);
    data->gyro_rps[1]   = sim_noise(0.001);
    data->gyro_rps[2]   = sim_noise(0.001);
    data->temp_C         = 26.0 + sim_noise(2.0);
    data->status         = 0;
    return HAL_OK;
}

/* ── Altimeters ──────────────────────────────────────────── */

HAL_Status_t HAL_Altimeter_Init(void) { return HAL_OK; }

HAL_Status_t HAL_Altimeter_ReadLidar(HAL_Altimeter_Raw_t *data)
{
    data->range_m  = 0.0 + sim_noise(0.01);  /* On surface */
    data->rate_mps = 0.0 + sim_noise(0.005);
    data->valid    = 1;
    return HAL_OK;
}

HAL_Status_t HAL_Altimeter_ReadRadar(HAL_Altimeter_Raw_t *data)
{
    data->range_m  = 0.0 + sim_noise(0.1);
    data->rate_mps = 0.0 + sim_noise(0.05);
    data->valid    = 1;
    return HAL_OK;
}
