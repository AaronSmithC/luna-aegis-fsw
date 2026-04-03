/**
 * @file eps_hal_sim.c
 * @brief EPS HAL — Desktop Simulation
 */

#include "eps_hal.h"
#include <stdlib.h>

static struct {
    double soc_pct;
    bool   initialized;
} eps_sim = {0};

static double eps_noise(double s)
{
    return ((double)rand() / RAND_MAX - 0.5) * 2.0 * s;
}

HAL_Status_t EpsHAL_BMU_Init(void)
{
    eps_sim.soc_pct     = 98.0;
    eps_sim.initialized = true;
    return HAL_OK;
}

HAL_Status_t EpsHAL_BMU_Read(EpsHAL_BmuTlm_t *tlm)
{
    if (!eps_sim.initialized) return HAL_ERR_NOT_INIT;
    tlm->bus_voltage_V       = 28.0 + eps_noise(0.3);
    tlm->bus_current_A       = 12.0 + eps_noise(1.0);
    tlm->soc_pct             = eps_sim.soc_pct;
    tlm->pack_temp_C         = 22.0 + eps_noise(1.0);
    tlm->cell_balance_active = 0;
    tlm->fault_flags         = 0;
    return HAL_OK;
}

HAL_Status_t EpsHAL_BMU_SetLoadShed(uint8_t level)
{
    (void)level;
    return HAL_OK;
}

HAL_Status_t EpsHAL_DrainSOC(double delta_pct)
{
    if (!eps_sim.initialized) return HAL_ERR_NOT_INIT;
    eps_sim.soc_pct -= delta_pct;
    if (eps_sim.soc_pct < 0.0) eps_sim.soc_pct = 0.0;
    return HAL_OK;
}
