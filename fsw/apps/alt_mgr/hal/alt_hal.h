/**
 * @file alt_hal.h
 * @brief ALT_MGR App Hardware Abstraction — Altimeters
 */

#ifndef ALT_HAL_H
#define ALT_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef int32_t HAL_Status_t;
#define HAL_OK             ((HAL_Status_t) 0)
#define HAL_ERR_NOT_INIT   ((HAL_Status_t)-4)

typedef struct {
    double  range_m;
    double  rate_mps;
    uint8_t valid;
} AltHAL_Raw_t;

HAL_Status_t AltHAL_Init(void);
HAL_Status_t AltHAL_ReadLidar(AltHAL_Raw_t *data);
HAL_Status_t AltHAL_ReadRadar(AltHAL_Raw_t *data);

#endif /* ALT_HAL_H */
