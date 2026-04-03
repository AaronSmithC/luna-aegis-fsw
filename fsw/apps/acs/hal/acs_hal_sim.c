/**
 * @file acs_hal_sim.c
 * @brief ACS HAL — Desktop Simulation
 */

#include "acs_hal.h"
#include <stdlib.h>
#include <string.h>

static struct {
    double gimbal_pitch_rad;
    double gimbal_yaw_rad;
    bool   initialized;
} acs_sim = {0};

static double acs_noise(double s)
{
    return ((double)rand() / RAND_MAX - 0.5) * 2.0 * s;
}

HAL_Status_t AcsHAL_Gimbal_Init(void)
{
    acs_sim.gimbal_pitch_rad = 0.0;
    acs_sim.gimbal_yaw_rad   = 0.0;
    acs_sim.initialized      = true;
    return HAL_OK;
}

HAL_Status_t AcsHAL_Gimbal_SetAngles(double pitch_rad, double yaw_rad)
{
    if (!acs_sim.initialized) return HAL_ERR_NOT_INIT;
    acs_sim.gimbal_pitch_rad = pitch_rad;
    acs_sim.gimbal_yaw_rad   = yaw_rad;
    return HAL_OK;
}

HAL_Status_t AcsHAL_Gimbal_GetAngles(double *pitch_rad, double *yaw_rad)
{
    if (!acs_sim.initialized) return HAL_ERR_NOT_INIT;
    *pitch_rad = acs_sim.gimbal_pitch_rad + acs_noise(0.0001);
    *yaw_rad   = acs_sim.gimbal_yaw_rad   + acs_noise(0.0001);
    return HAL_OK;
}

HAL_Status_t AcsHAL_Gimbal_GetStatus(uint8_t *health, double *temp_C)
{
    if (!acs_sim.initialized) return HAL_ERR_NOT_INIT;
    *health = 0;
    *temp_C = 30.0 + acs_noise(2.0);
    return HAL_OK;
}

HAL_Status_t AcsHAL_RCS_Init(void) { return HAL_OK; }

HAL_Status_t AcsHAL_RCS_Fire(const AcsHAL_RcsCmd_t *cmd)
{
    (void)cmd;
    return HAL_OK;
}

HAL_Status_t AcsHAL_RCS_GetValveState(uint16_t *cycle_counts,
                                      uint8_t *health_mask)
{
    memset(cycle_counts, 0, ACS_HAL_RCS_COUNT * sizeof(uint16_t));
    *health_mask = 0xFF;
    return HAL_OK;
}
