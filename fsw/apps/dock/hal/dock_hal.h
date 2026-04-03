/**
 * @file dock_hal.h
 * @brief DOCK App Hardware Abstraction — Docking Collar
 */

#ifndef DOCK_HAL_H
#define DOCK_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef int32_t HAL_Status_t;
#define HAL_OK             ((HAL_Status_t) 0)
#define HAL_ERR_NOT_INIT   ((HAL_Status_t)-4)

typedef enum {
    DOCK_HAL_RETRACTED   = 0,
    DOCK_HAL_EXTENDING   = 1,
    DOCK_HAL_SOFT_DOCK   = 2,
    DOCK_HAL_PRESS_EQ    = 3,
    DOCK_HAL_HARD_DOCK   = 4,
} DockHAL_State_t;

typedef struct {
    uint8_t state;                /* DockHAL_State_t                 */
    double  seal_press_kPa;
    double  tunnel_press_kPa;
    double  delta_p_kPa;
    uint8_t latch_engaged;        /* number of latches locked        */
} DockHAL_Tlm_t;

HAL_Status_t DockHAL_Init(void);
HAL_Status_t DockHAL_Command(DockHAL_State_t cmd);
HAL_Status_t DockHAL_Read(DockHAL_Tlm_t *tlm);

#endif /* DOCK_HAL_H */
