/**
 * @file tcs_hal_sim.c
 * @brief TCS HAL — Desktop Simulation
 */

#include "tcs_hal.h"
#include <stdlib.h>

static struct {
    bool    heater_on[TCS_HAL_ZONE_COUNT];
    uint8_t heater_duty[TCS_HAL_ZONE_COUNT];
    bool    initialized;
} tcs_sim = {0};

static double tcs_noise(double s)
{
    return ((double)rand() / RAND_MAX - 0.5) * 2.0 * s;
}

HAL_Status_t TcsHAL_Init(void)
{
    tcs_sim.initialized = true;
    return HAL_OK;
}

HAL_Status_t TcsHAL_Heater_SetZone(uint8_t zone, bool enable,
                                   uint8_t duty_pct)
{
    if (zone >= TCS_HAL_ZONE_COUNT) return HAL_ERR_RANGE;
    tcs_sim.heater_on[zone]   = enable;
    tcs_sim.heater_duty[zone] = duty_pct;
    return HAL_OK;
}

HAL_Status_t TcsHAL_GetTemps(double temps_C[TCS_HAL_ZONE_COUNT])
{
    for (int i = 0; i < TCS_HAL_ZONE_COUNT; i++) {
        double base = tcs_sim.heater_on[i]
                      ? 25.0 + (tcs_sim.heater_duty[i] * 0.3)
                      : -40.0;
        temps_C[i] = base + tcs_noise(2.0);
    }
    return HAL_OK;
}
