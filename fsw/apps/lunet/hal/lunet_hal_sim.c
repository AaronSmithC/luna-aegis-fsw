/**
 * @file lunet_hal_sim.c
 * @brief LUNET HAL — Desktop Simulation
 */

#include "lunet_hal.h"
#include <stdlib.h>

static bool lunet_sim_init = false;

static double lunet_noise(double s)
{
    return ((double)rand() / RAND_MAX - 0.5) * 2.0 * s;
}

HAL_Status_t LunetHAL_Init(void)
{
    lunet_sim_init = true;
    return HAL_OK;
}

HAL_Status_t LunetHAL_ReadBeacon(LunetHAL_BeaconRaw_t *data)
{
    if (!lunet_sim_init) return HAL_ERR_NOT_INIT;
    /* Sim: beacon ~500m away, strong signal */
    data->range_m       = 500.0 + lunet_noise(2.0);
    data->bearing_deg   = 45.0  + lunet_noise(0.5);
    data->elevation_deg = -2.0  + lunet_noise(0.3);
    data->signal_quality = 0.92 + lunet_noise(0.03);
    data->beacon_id     = 1;
    data->lock          = 1;
    return HAL_OK;
}
