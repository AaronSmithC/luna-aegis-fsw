/**
 * @file imu_hal.h
 * @brief IMU_MGR App Hardware Abstraction — IMU Sensors
 *
 * FOG/RLG primary + MEMS backup inertial measurement units.
 */

#ifndef IMU_HAL_H
#define IMU_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef int32_t HAL_Status_t;
#define HAL_OK             ((HAL_Status_t) 0)
#define HAL_ERR_TIMEOUT    ((HAL_Status_t)-1)
#define HAL_ERR_BUS        ((HAL_Status_t)-2)
#define HAL_ERR_NOT_INIT   ((HAL_Status_t)-4)
#define HAL_ERR_HW_FAULT   ((HAL_Status_t)-5)

typedef struct {
    double  accel_mps2[3];        /* X, Y, Z specific force          */
    double  gyro_rps[3];          /* X, Y, Z angular rate            */
    double  temp_C;
    uint8_t status;               /* 0=OK, 1=degraded                */
} ImuHAL_Raw_t;

HAL_Status_t ImuHAL_Init(void);
HAL_Status_t ImuHAL_ReadPrimary(ImuHAL_Raw_t *data);
HAL_Status_t ImuHAL_ReadBackup(ImuHAL_Raw_t *data);

#endif /* IMU_HAL_H */
