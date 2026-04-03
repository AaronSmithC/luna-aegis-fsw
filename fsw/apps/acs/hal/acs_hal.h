/**
 * @file acs_hal.h
 * @brief ACS App Hardware Abstraction — Gimbal & RCS
 */

#ifndef ACS_HAL_H
#define ACS_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef int32_t HAL_Status_t;
#define HAL_OK             ((HAL_Status_t) 0)
#define HAL_ERR_NOT_INIT   ((HAL_Status_t)-4)
#define HAL_ERR_RANGE      ((HAL_Status_t)-3)

#define ACS_HAL_RCS_COUNT  12

typedef struct {
    uint8_t  firing_mask;
    uint16_t pulse_ms[ACS_HAL_RCS_COUNT];
} AcsHAL_RcsCmd_t;

HAL_Status_t AcsHAL_Gimbal_Init(void);
HAL_Status_t AcsHAL_Gimbal_SetAngles(double pitch_rad, double yaw_rad);
HAL_Status_t AcsHAL_Gimbal_GetAngles(double *pitch_rad, double *yaw_rad);
HAL_Status_t AcsHAL_Gimbal_GetStatus(uint8_t *health, double *temp_C);

HAL_Status_t AcsHAL_RCS_Init(void);
HAL_Status_t AcsHAL_RCS_Fire(const AcsHAL_RcsCmd_t *cmd);
HAL_Status_t AcsHAL_RCS_GetValveState(uint16_t *cycle_counts,
                                      uint8_t *health_mask);

#endif /* ACS_HAL_H */
