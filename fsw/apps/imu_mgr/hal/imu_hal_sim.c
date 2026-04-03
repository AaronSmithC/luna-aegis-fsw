/**
 * @file imu_hal_sim.c
 * @brief IMU_MGR HAL — Desktop Simulation
 */

#include "imu_hal.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>

static bool imu_sim_init = false;

static double imu_noise(double scale)
{
    return ((double)rand() / RAND_MAX - 0.5) * 2.0 * scale;
}

HAL_Status_t ImuHAL_Init(void)
{
    imu_sim_init = true;
    return HAL_OK;
}

HAL_Status_t ImuHAL_ReadPrimary(ImuHAL_Raw_t *data)
{
    if (!imu_sim_init) return HAL_ERR_NOT_INIT;
    /* FOG/RLG: low noise, high accuracy */
    data->accel_mps2[0] = 0.0 + imu_noise(0.001);
    data->accel_mps2[1] = 0.0 + imu_noise(0.001);
    data->accel_mps2[2] = -1.625 + imu_noise(0.002); /* lunar g */
    data->gyro_rps[0]   = imu_noise(0.0001);
    data->gyro_rps[1]   = imu_noise(0.0001);
    data->gyro_rps[2]   = imu_noise(0.0001);
    data->temp_C         = 25.0 + imu_noise(0.5);
    data->status         = 0;
    return HAL_OK;
}

HAL_Status_t ImuHAL_ReadBackup(ImuHAL_Raw_t *data)
{
    if (!imu_sim_init) return HAL_ERR_NOT_INIT;
    /* MEMS: higher noise, lower accuracy */
    data->accel_mps2[0] = 0.0 + imu_noise(0.01);
    data->accel_mps2[1] = 0.0 + imu_noise(0.01);
    data->accel_mps2[2] = -1.625 + imu_noise(0.02);
    data->gyro_rps[0]   = imu_noise(0.001);
    data->gyro_rps[1]   = imu_noise(0.001);
    data->gyro_rps[2]   = imu_noise(0.001);
    data->temp_C         = 26.0 + imu_noise(1.0);
    data->status         = 0;
    return HAL_OK;
}
