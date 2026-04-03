/**
 * @file prop_hal.h
 * @brief PROP App Hardware Abstraction — Engine & Cryo Tanks
 *
 * Isolates PROP from hardware specifics.  Two implementations:
 *   prop_hal_sim.c  — desktop simulation
 *   prop_hal_hw.c   — flight hardware (future)
 *
 * PROP calls ONLY these functions for HW access.
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#ifndef PROP_HAL_H
#define PROP_HAL_H

#include <stdint.h>
#include <stdbool.h>

/* ── Return Codes (shared across all per-app HALs) ──────── */

typedef int32_t HAL_Status_t;

#define HAL_OK             ((HAL_Status_t) 0)
#define HAL_ERR_TIMEOUT    ((HAL_Status_t)-1)
#define HAL_ERR_BUS        ((HAL_Status_t)-2)
#define HAL_ERR_RANGE      ((HAL_Status_t)-3)
#define HAL_ERR_NOT_INIT   ((HAL_Status_t)-4)
#define HAL_ERR_HW_FAULT   ((HAL_Status_t)-5)

/* ── Engine States ──────────────────────────────────────── */

typedef enum {
    PROP_HAL_ENGINE_OFF       = 0,
    PROP_HAL_ENGINE_CHILL     = 1,
    PROP_HAL_ENGINE_IGNITION  = 2,
    PROP_HAL_ENGINE_RUNNING   = 3,
    PROP_HAL_ENGINE_SHUTDOWN  = 4,
    PROP_HAL_ENGINE_FAULT     = 5,
} PropHAL_EngineState_t;

/* ── Engine Telemetry ───────────────────────────────────── */

typedef struct {
    double   chamber_press_kPa;
    double   mixture_ratio;
    double   thrust_est_N;
    uint8_t  state;               /* PropHAL_EngineState_t        */
    uint8_t  valve_positions;     /* Bitfield: main/ox/fuel       */
    uint16_t igniter_count;
} PropHAL_EngineTlm_t;

/* ── Cryo Tank Telemetry ────────────────────────────────── */

typedef struct {
    double  ox_press_kPa;
    double  fuel_press_kPa;
    double  ox_temp_K;
    double  fuel_temp_K;
    double  ox_mass_kg;
    double  fuel_mass_kg;
    uint8_t vent_valve_state;
    uint8_t relief_active;
} PropHAL_CryoTlm_t;

/* ── Engine Interface ───────────────────────────────────── */

HAL_Status_t PropHAL_Engine_Init(void);
HAL_Status_t PropHAL_Engine_Command(PropHAL_EngineState_t cmd);
HAL_Status_t PropHAL_Engine_SetThrottle(double pct);
HAL_Status_t PropHAL_Engine_Read(PropHAL_EngineTlm_t *tlm);

/* ── Cryo Tank Interface ────────────────────────────────── */

HAL_Status_t PropHAL_CryoTank_Init(void);
HAL_Status_t PropHAL_CryoTank_Read(PropHAL_CryoTlm_t *tlm);
HAL_Status_t PropHAL_CryoTank_SetVent(bool open);

/* ── Propellant Consumption (sim drives mass decrement) ─── */

HAL_Status_t PropHAL_ConsumePropellant(double ox_kg, double fuel_kg);

#endif /* PROP_HAL_H */
