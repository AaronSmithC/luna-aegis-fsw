/**
 * @file lg_hal.h
 * @brief LG App Hardware Abstraction — Landing Gear
 */

#ifndef LG_HAL_H
#define LG_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef int32_t HAL_Status_t;
#define HAL_OK             ((HAL_Status_t) 0)
#define HAL_ERR_NOT_INIT   ((HAL_Status_t)-4)

typedef enum {
    LG_HAL_STOWED    = 0,
    LG_HAL_TRANSIT   = 1,
    LG_HAL_DEPLOYED  = 2,
} LgHAL_State_t;

typedef struct {
    uint8_t state;                /* LgHAL_State_t                   */
    double  load_N[4];            /* Per-leg axial load               */
    uint8_t contact[4];           /* Per-leg touchdown contact        */
} LgHAL_Tlm_t;

HAL_Status_t LgHAL_Init(void);
HAL_Status_t LgHAL_Command(LgHAL_State_t cmd);
HAL_Status_t LgHAL_Read(LgHAL_Tlm_t *tlm);

#endif /* LG_HAL_H */
