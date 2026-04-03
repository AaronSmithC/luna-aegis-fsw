/**
 * @file alt_hal_sim.c
 * @brief ALT_MGR HAL — Desktop Simulation
 */

#include "alt_hal.h"
#include <stdlib.h>
#include <stdbool.h>

static bool alt_sim_init = false;
static double alt_sim_noise(double s)
{
    return ((double)rand() / RAND_MAX - 0.5) * 2.0 * s;
}

HAL_Status_t AltHAL_Init(void) { alt_sim_init = true; return HAL_OK; }

HAL_Status_t AltHAL_ReadLidar(AltHAL_Raw_t *data)
{
    if (!alt_sim_init) return HAL_ERR_NOT_INIT;
    data->range_m  = 0.0 + alt_sim_noise(0.05);  /* on surface */
    data->rate_mps = 0.0 + alt_sim_noise(0.01);
    data->valid    = 1;
    return HAL_OK;
}

HAL_Status_t AltHAL_ReadRadar(AltHAL_Raw_t *data)
{
    if (!alt_sim_init) return HAL_ERR_NOT_INIT;
    data->range_m  = 0.0 + alt_sim_noise(0.5);   /* noisier */
    data->rate_mps = 0.0 + alt_sim_noise(0.05);
    data->valid    = 1;
    return HAL_OK;
}
