/**
 * @file comm_hal_sim.c
 * @brief COMM HAL — Desktop Simulation
 */

#include "comm_hal.h"
#include <stdlib.h>

static struct {
    double az_deg;
    double el_deg;
    bool   initialized;
} comm_sim = {0};

HAL_Status_t CommHAL_Antenna_Init(void)
{
    comm_sim.az_deg      = 0.0;
    comm_sim.el_deg      = 45.0;
    comm_sim.initialized = true;
    return HAL_OK;
}

HAL_Status_t CommHAL_Antenna_Point(double az_deg, double el_deg)
{
    if (!comm_sim.initialized) return HAL_ERR_NOT_INIT;
    comm_sim.az_deg = az_deg;
    comm_sim.el_deg = el_deg;
    return HAL_OK;
}

HAL_Status_t CommHAL_Antenna_GetPointing(double *az_deg, double *el_deg)
{
    if (!comm_sim.initialized) return HAL_ERR_NOT_INIT;
    *az_deg = comm_sim.az_deg;
    *el_deg = comm_sim.el_deg;
    return HAL_OK;
}
