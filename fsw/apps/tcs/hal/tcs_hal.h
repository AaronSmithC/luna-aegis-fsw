/**
 * @file tcs_hal.h
 * @brief TCS App Hardware Abstraction — Heaters & Temp Sensors
 */

#ifndef TCS_HAL_H
#define TCS_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef int32_t HAL_Status_t;
#define HAL_OK             ((HAL_Status_t) 0)
#define HAL_ERR_NOT_INIT   ((HAL_Status_t)-4)
#define HAL_ERR_RANGE      ((HAL_Status_t)-3)

#define TCS_HAL_ZONE_COUNT  8

HAL_Status_t TcsHAL_Init(void);
HAL_Status_t TcsHAL_Heater_SetZone(uint8_t zone, bool enable,
                                   uint8_t duty_pct);
HAL_Status_t TcsHAL_GetTemps(double temps_C[TCS_HAL_ZONE_COUNT]);

#endif /* TCS_HAL_H */
