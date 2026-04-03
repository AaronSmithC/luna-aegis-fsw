/**
 * @file comm_hal.h
 * @brief COMM App Hardware Abstraction — Antenna
 */

#ifndef COMM_HAL_H
#define COMM_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef int32_t HAL_Status_t;
#define HAL_OK             ((HAL_Status_t) 0)
#define HAL_ERR_NOT_INIT   ((HAL_Status_t)-4)

HAL_Status_t CommHAL_Antenna_Init(void);
HAL_Status_t CommHAL_Antenna_Point(double az_deg, double el_deg);
HAL_Status_t CommHAL_Antenna_GetPointing(double *az_deg, double *el_deg);

#endif /* COMM_HAL_H */
