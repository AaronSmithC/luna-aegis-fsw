/**
 * @file prop_hal_sim.c
 * @brief PROP App HAL — Desktop Simulation Implementation
 *
 * Simulates engine startup sequence with realistic pressure
 * ramps, propellant consumption, and cryo tank state.
 */

#include "prop_hal.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

/* ── Sim State ───────────────────────────────────────────── */

static struct {
    PropHAL_EngineState_t engine_state;
    double throttle_pct;
    double ox_mass_kg;
    double fuel_mass_kg;
    bool   initialized;
} prop_sim = {0};

static double prop_sim_noise(double scale)
{
    return ((double)rand() / RAND_MAX - 0.5) * 2.0 * scale;
}

/* ── Engine ──────────────────────────────────────────────── */

HAL_Status_t PropHAL_Engine_Init(void)
{
    prop_sim.engine_state = PROP_HAL_ENGINE_OFF;
    prop_sim.throttle_pct = 0.0;
    prop_sim.ox_mass_kg   = 2062.5;   /* LOX: 75% of 2750 kg */
    prop_sim.fuel_mass_kg = 687.5;    /* LH2: 25% (O/F ~3:1) */
    prop_sim.initialized  = true;
    return HAL_OK;
}

HAL_Status_t PropHAL_Engine_Command(PropHAL_EngineState_t cmd)
{
    if (!prop_sim.initialized) return HAL_ERR_NOT_INIT;
    prop_sim.engine_state = cmd;
    if (cmd == PROP_HAL_ENGINE_OFF || cmd == PROP_HAL_ENGINE_SHUTDOWN) {
        prop_sim.throttle_pct = 0.0;
    }
    return HAL_OK;
}

HAL_Status_t PropHAL_Engine_SetThrottle(double pct)
{
    if (!prop_sim.initialized) return HAL_ERR_NOT_INIT;
    if (pct < 0.0) pct = 0.0;
    if (pct > 100.0) pct = 100.0;
    prop_sim.throttle_pct = pct;
    return HAL_OK;
}

HAL_Status_t PropHAL_Engine_Read(PropHAL_EngineTlm_t *tlm)
{
    if (!prop_sim.initialized) return HAL_ERR_NOT_INIT;
    tlm->state = (uint8_t)prop_sim.engine_state;

    switch (prop_sim.engine_state) {
    case PROP_HAL_ENGINE_RUNNING:
        {
            double thrust = 30000.0 * (prop_sim.throttle_pct / 100.0);
            tlm->chamber_press_kPa = 3500.0 * (prop_sim.throttle_pct / 100.0)
                                     + prop_sim_noise(20.0);
            tlm->mixture_ratio     = 6.0 + prop_sim_noise(0.05);
            tlm->thrust_est_N      = thrust + prop_sim_noise(50.0);
            tlm->valve_positions   = 0x07; /* main+ox+fuel open */
        }
        break;

    case PROP_HAL_ENGINE_IGNITION:
        /* Pressure ramps up during ignition sequence */
        tlm->chamber_press_kPa = 2000.0 + prop_sim_noise(200.0);
        tlm->mixture_ratio     = 4.0 + prop_sim_noise(1.0);
        tlm->thrust_est_N      = 15000.0 + prop_sim_noise(2000.0);
        tlm->valve_positions   = 0x07;
        break;

    case PROP_HAL_ENGINE_CHILL:
        /* Turbopump chill — small pressure from propellant flow */
        tlm->chamber_press_kPa = 50.0 + prop_sim_noise(10.0);
        tlm->mixture_ratio     = 0.0;
        tlm->thrust_est_N      = 0.0;
        tlm->valve_positions   = 0x03; /* ox+fuel chill valves */
        break;

    case PROP_HAL_ENGINE_SHUTDOWN:
        /* Pressure bleeding down */
        tlm->chamber_press_kPa = 100.0 + prop_sim_noise(50.0);
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

/* ── Cryo Tanks ──────────────────────────────────────────── */

HAL_Status_t PropHAL_CryoTank_Init(void)
{
    return HAL_OK;
}

HAL_Status_t PropHAL_CryoTank_Read(PropHAL_CryoTlm_t *tlm)
{
    if (!prop_sim.initialized) return HAL_ERR_NOT_INIT;
    tlm->ox_press_kPa   = 250.0 + prop_sim_noise(5.0);
    tlm->fuel_press_kPa = 180.0 + prop_sim_noise(3.0);
    tlm->ox_temp_K      = 90.0  + prop_sim_noise(0.5);
    tlm->fuel_temp_K    = 20.5  + prop_sim_noise(0.2);
    tlm->ox_mass_kg     = prop_sim.ox_mass_kg;
    tlm->fuel_mass_kg   = prop_sim.fuel_mass_kg;
    tlm->vent_valve_state = 0;
    tlm->relief_active    = 0;
    return HAL_OK;
}

HAL_Status_t PropHAL_CryoTank_SetVent(bool open)
{
    (void)open;
    return HAL_OK;
}

HAL_Status_t PropHAL_ConsumePropellant(double ox_kg, double fuel_kg)
{
    if (!prop_sim.initialized) return HAL_ERR_NOT_INIT;
    prop_sim.ox_mass_kg   -= ox_kg;
    prop_sim.fuel_mass_kg -= fuel_kg;
    if (prop_sim.ox_mass_kg < 0.0)   prop_sim.ox_mass_kg   = 0.0;
    if (prop_sim.fuel_mass_kg < 0.0) prop_sim.fuel_mass_kg = 0.0;
    return HAL_OK;
}
