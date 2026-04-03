/**
 * @file eps_hal.h
 * @brief EPS App Hardware Abstraction — Battery Management Unit
 */

#ifndef EPS_HAL_H
#define EPS_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef int32_t HAL_Status_t;
#define HAL_OK             ((HAL_Status_t) 0)
#define HAL_ERR_NOT_INIT   ((HAL_Status_t)-4)

typedef struct {
    double  bus_voltage_V;
    double  bus_current_A;
    double  soc_pct;
    double  pack_temp_C;
    uint8_t cell_balance_active;
    uint8_t fault_flags;
} EpsHAL_BmuTlm_t;

HAL_Status_t EpsHAL_BMU_Init(void);
HAL_Status_t EpsHAL_BMU_Read(EpsHAL_BmuTlm_t *tlm);
HAL_Status_t EpsHAL_BMU_SetLoadShed(uint8_t level);

/* SOC drain for sim — called by EPS app to model consumption */
HAL_Status_t EpsHAL_DrainSOC(double delta_pct);

#endif /* EPS_HAL_H */
