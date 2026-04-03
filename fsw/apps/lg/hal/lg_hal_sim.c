/**
 * @file lg_hal_sim.c
 * @brief LG HAL — Desktop Simulation
 */

#include "lg_hal.h"
#include <stdlib.h>

static struct {
    LgHAL_State_t state;
    bool          initialized;
} lg_sim = {0};

static double lg_noise(double s)
{
    return ((double)rand() / RAND_MAX - 0.5) * 2.0 * s;
}

HAL_Status_t LgHAL_Init(void)
{
    lg_sim.state       = LG_HAL_DEPLOYED;  /* start on surface */
    lg_sim.initialized = true;
    return HAL_OK;
}

HAL_Status_t LgHAL_Command(LgHAL_State_t cmd)
{
    if (!lg_sim.initialized) return HAL_ERR_NOT_INIT;
    lg_sim.state = cmd;
    return HAL_OK;
}

HAL_Status_t LgHAL_Read(LgHAL_Tlm_t *tlm)
{
    if (!lg_sim.initialized) return HAL_ERR_NOT_INIT;
    tlm->state = (uint8_t)lg_sim.state;
    for (int i = 0; i < 4; i++) {
        if (lg_sim.state == LG_HAL_DEPLOYED) {
            tlm->load_N[i]  = 325.0 + lg_noise(10.0); /* ~1/4 vehicle weight on Moon */
            tlm->contact[i] = 1;
        } else {
            tlm->load_N[i]  = 0.0;
            tlm->contact[i] = 0;
        }
    }
    return HAL_OK;
}
