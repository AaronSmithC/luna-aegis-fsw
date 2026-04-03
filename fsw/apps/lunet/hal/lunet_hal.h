/**
 * @file lunet_hal.h
 * @brief LUNET App Hardware Abstraction — Beacon Receiver
 *
 * Read-only interface to LUNET beacon radio receiver.
 * No actuators — LUNET is a passive sensor from the
 * vehicle's perspective.
 */

#ifndef LUNET_HAL_H
#define LUNET_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef int32_t HAL_Status_t;
#define HAL_OK             ((HAL_Status_t) 0)
#define HAL_ERR_NOT_INIT   ((HAL_Status_t)-4)

typedef struct {
    double  range_m;
    double  bearing_deg;
    double  elevation_deg;
    double  signal_quality;       /* 0.0–1.0                         */
    uint8_t beacon_id;
    uint8_t lock;                 /* 0=no lock, 1=locked             */
} LunetHAL_BeaconRaw_t;

HAL_Status_t LunetHAL_Init(void);
HAL_Status_t LunetHAL_ReadBeacon(LunetHAL_BeaconRaw_t *data);

#endif /* LUNET_HAL_H */
