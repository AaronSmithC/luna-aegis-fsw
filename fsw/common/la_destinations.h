/**
 * @file la_destinations.h
 * @brief Luna-Aegis Short Hopper — Destination Table
 *
 * Defines all known mission destinations with lunar coordinates,
 * type (surface/orbital), associated LUNET node IDs, and
 * approximate Δv requirements.
 *
 * Coordinates use a simplified south-pole-centric frame:
 *   lat_deg:  Latitude (negative = south)
 *   lon_deg:  Longitude (east positive)
 *   alt_m:    Altitude above mean lunar radius (1737.4 km)
 *             Surface sites: 0 (on terrain)
 *             Orbital: LLO altitude (~100 km)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#ifndef LA_DESTINATIONS_H
#define LA_DESTINATIONS_H

#include <stdint.h>

/* ── Destination Types ───────────────────────────────────── */

#define LA_DEST_TYPE_SURFACE   0
#define LA_DEST_TYPE_ORBITAL   1

/* ── Destination Entry ───────────────────────────────────── */

typedef struct {
    uint8_t     id;               /* Unique destination ID            */
    uint8_t     type;             /* LA_DEST_TYPE_SURFACE/ORBITAL     */
    uint8_t     lunet_node_id;    /* LUNET beacon/refuel node (0=none)*/
    uint8_t     has_refuel;       /* 1 = ISRU refuel available        */
    double      lat_deg;          /* Latitude (deg, south negative)   */
    double      lon_deg;          /* Longitude (deg, east positive)   */
    double      alt_m;            /* Altitude above mean radius (m)   */
    double      approx_dv_mps;   /* Approximate Δv from reference    */
    const char *name;             /* Human-readable name              */
} LA_Destination_t;

/* ── Moon Constants ──────────────────────────────────────── */

#define LA_MOON_RADIUS_M       1737400.0   /* Mean lunar radius (m)  */
#define LA_MOON_G_MPS2         1.625       /* Surface gravity (m/s²) */
#define LA_LLO_ALT_M           100000.0    /* Low lunar orbit (m)    */

/* ── Destination Table ───────────────────────────────────── */

/*
 * South pole region destinations based on the Short Hopper's
 * 1,500–2,000 km one-way range.  All surface sites are within
 * ~30° of the south pole.  Approximate Δv values assume
 * one-way hop with ISRU refuel at destination.
 *
 * Reference point: Shackleton Crater rim (primary base).
 */

#define LA_DEST_COUNT  8

static const LA_Destination_t LA_DEST_TABLE[LA_DEST_COUNT] = {

    /* ── ID 0: Home Base — Shackleton Crater Rim ── */
    {
        .id = 0, .type = LA_DEST_TYPE_SURFACE,
        .lunet_node_id = 1, .has_refuel = 1,
        .lat_deg = -89.90, .lon_deg = 0.0, .alt_m = 0.0,
        .approx_dv_mps = 0.0,
        .name = "SHACKLETON BASE"
    },

    /* ── ID 1: Aegis Station — Low Lunar Orbit ── */
    {
        .id = 1, .type = LA_DEST_TYPE_ORBITAL,
        .lunet_node_id = 8, .has_refuel = 1,
        .lat_deg = -89.90, .lon_deg = 0.0, .alt_m = LA_LLO_ALT_M,
        .approx_dv_mps = 1680.0,
        .name = "AEGIS STATION"
    },

    /* ── ID 2: Shoemaker Crater — PSR Ice Mining ── */
    {
        .id = 2, .type = LA_DEST_TYPE_SURFACE,
        .lunet_node_id = 2, .has_refuel = 1,
        .lat_deg = -88.10, .lon_deg = 44.9, .alt_m = 0.0,
        .approx_dv_mps = 320.0,
        .name = "SHOEMAKER ISRU"
    },

    /* ── ID 3: Malapert Mountain — Comms Relay ── */
    {
        .id = 3, .type = LA_DEST_TYPE_SURFACE,
        .lunet_node_id = 3, .has_refuel = 0,
        .lat_deg = -86.00, .lon_deg = 0.0, .alt_m = 5000.0,
        .approx_dv_mps = 480.0,
        .name = "MALAPERT RELAY"
    },

    /* ── ID 4: Amundsen Crater — Science Outpost ── */
    {
        .id = 4, .type = LA_DEST_TYPE_SURFACE,
        .lunet_node_id = 4, .has_refuel = 1,
        .lat_deg = -84.50, .lon_deg = -84.3, .alt_m = 0.0,
        .approx_dv_mps = 720.0,
        .name = "AMUNDSEN SCIENCE"
    },

    /* ── ID 5: Cabeus Crater — Water Ice Depot ── */
    {
        .id = 5, .type = LA_DEST_TYPE_SURFACE,
        .lunet_node_id = 5, .has_refuel = 1,
        .lat_deg = -85.30, .lon_deg = -35.7, .alt_m = 0.0,
        .approx_dv_mps = 580.0,
        .name = "CABEUS ICE DEPOT"
    },

    /* ── ID 6: Connecting Ridge — Waypoint ── */
    {
        .id = 6, .type = LA_DEST_TYPE_SURFACE,
        .lunet_node_id = 6, .has_refuel = 0,
        .lat_deg = -87.50, .lon_deg = 22.0, .alt_m = 2000.0,
        .approx_dv_mps = 380.0,
        .name = "RIDGE WAYPOINT"
    },

    /* ── ID 7: De Gerlache Crater — Habitat Alpha ── */
    {
        .id = 7, .type = LA_DEST_TYPE_SURFACE,
        .lunet_node_id = 7, .has_refuel = 1,
        .lat_deg = -88.50, .lon_deg = -87.1, .alt_m = 0.0,
        .approx_dv_mps = 420.0,
        .name = "DE GERLACHE HAB"
    },
};

/* ── Helpers ─────────────────────────────────────────────── */

/**
 * Compute great-circle surface distance between two points
 * on the Moon using the Haversine formula.
 * Returns distance in meters.
 *
 * NOTE: Requires libm (sin, cos, sqrt, asin).
 * Link with -lm.  This is already linked for apps
 * that use la_hal or math.h.
 */
#include <math.h>

static inline double LA_Dest_SurfaceDistance_m(
    double lat1_deg, double lon1_deg,
    double lat2_deg, double lon2_deg)
{
    const double DEG2RAD = 3.14159265358979323846 / 180.0;
    double lat1 = lat1_deg * DEG2RAD;
    double lat2 = lat2_deg * DEG2RAD;
    double dlat = (lat2_deg - lat1_deg) * DEG2RAD;
    double dlon = (lon2_deg - lon1_deg) * DEG2RAD;

    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1) * cos(lat2) *
               sin(dlon / 2.0) * sin(dlon / 2.0);
    double c = 2.0 * asin(sqrt(a));
    return LA_MOON_RADIUS_M * c;
}

/**
 * Look up a destination by ID.  Returns NULL if not found.
 */
static inline const LA_Destination_t *LA_Dest_Lookup(uint8_t id)
{
    for (int i = 0; i < LA_DEST_COUNT; i++) {
        if (LA_DEST_TABLE[i].id == id) return &LA_DEST_TABLE[i];
    }
    return (void*)0;
}

#endif /* LA_DESTINATIONS_H */
