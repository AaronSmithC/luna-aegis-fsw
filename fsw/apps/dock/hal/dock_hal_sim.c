/**
 * @file dock_hal_sim.c
 * @brief DOCK HAL — Desktop Simulation
 */

#include "dock_hal.h"
#include <stdlib.h>
#include <math.h>

static struct {
    DockHAL_State_t state;
    bool            initialized;
} dock_sim = {0};

static double dock_noise(double s)
{
    return ((double)rand() / RAND_MAX - 0.5) * 2.0 * s;
}

HAL_Status_t DockHAL_Init(void)
{
    dock_sim.state       = DOCK_HAL_RETRACTED;
    dock_sim.initialized = true;
    return HAL_OK;
}

HAL_Status_t DockHAL_Command(DockHAL_State_t cmd)
{
    if (!dock_sim.initialized) return HAL_ERR_NOT_INIT;
    dock_sim.state = cmd;
    return HAL_OK;
}

HAL_Status_t DockHAL_Read(DockHAL_Tlm_t *tlm)
{
    if (!dock_sim.initialized) return HAL_ERR_NOT_INIT;
    tlm->state = (uint8_t)dock_sim.state;

    tlm->seal_press_kPa   = (dock_sim.state >= DOCK_HAL_SOFT_DOCK)
                             ? 101.3 + dock_noise(0.5) : 0.0;
    tlm->tunnel_press_kPa = (dock_sim.state == DOCK_HAL_HARD_DOCK)
                             ? 101.3 + dock_noise(0.2) : 0.0;
    tlm->delta_p_kPa      = fabs(tlm->seal_press_kPa -
                                  tlm->tunnel_press_kPa);
    tlm->latch_engaged    = (dock_sim.state == DOCK_HAL_HARD_DOCK)
                             ? 12 : 0;
    return HAL_OK;
}
