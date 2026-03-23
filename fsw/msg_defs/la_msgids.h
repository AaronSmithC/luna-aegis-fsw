/**
 * @file la_msgids.h
 * @brief Luna-Aegis Short Hopper — Master Message ID Registry
 *
 * Every packet on the cFE Software Bus is assigned a unique
 * CCSDS APID-derived message ID.  This file is the single
 * source of truth for all SB message routing.
 *
 * Convention:
 *   0x1800–0x18FF  Telemetry (TLM) packets
 *   0x1900–0x19FF  Command (CMD) packets
 *   0x1A00–0x1AFF  Internal wakeup / scheduling messages
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#ifndef LA_MSGIDS_H
#define LA_MSGIDS_H

/* ══════════════════════════════════════════════════════════
 * TIER 0 — CORE EXECUTIVE (cFE-provided, listed for reference)
 * ══════════════════════════════════════════════════════════ */

/* These are defined by cFE itself; shown here for completeness */

/* ══════════════════════════════════════════════════════════
 * TIER 1 — SCHEDULING & TELEMETRY
 * ══════════════════════════════════════════════════════════ */

#define LA_SCH_CMD_MID              0x1901
#define LA_SCH_HK_TLM_MID          0x1801
#define LA_SCH_WAKEUP_MID(slot)     (0x1A00 + (slot))  /* 0x1A01–0x1A30 */

#define LA_HK_CMD_MID               0x1902
#define LA_HK_HK_TLM_MID           0x1802
#define LA_HK_COMBINED_TLM_MID     0x1803

#define LA_CI_CMD_MID               0x1903
#define LA_CI_HK_TLM_MID           0x1804

#define LA_TO_CMD_MID               0x1904
#define LA_TO_HK_TLM_MID           0x1805

#define LA_DS_CMD_MID               0x1905
#define LA_DS_HK_TLM_MID           0x1806

#define LA_FM_CMD_MID               0x1906
#define LA_FM_HK_TLM_MID           0x1807

/* ══════════════════════════════════════════════════════════
 * TIER 2 — GN&C FLIGHT SOFTWARE
 * ══════════════════════════════════════════════════════════ */

#define LA_IMU_MGR_CMD_MID          0x1910
#define LA_IMU_MGR_HK_TLM_MID      0x1810
#define LA_IMU_DATA_TLM_MID        0x1811  /* q, omega — 40 Hz */

#define LA_ALT_MGR_CMD_MID          0x1912
#define LA_ALT_MGR_HK_TLM_MID      0x1812
#define LA_ALT_DATA_TLM_MID        0x1813  /* AGL, rate — 10 Hz */

#define LA_NAV_CMD_MID              0x1914
#define LA_NAV_HK_TLM_MID          0x1814
#define LA_NAV_STATE_TLM_MID       0x1815  /* pos, vel, att — 40 Hz */

#define LA_GDN_CMD_MID              0x1916
#define LA_GDN_HK_TLM_MID          0x1816
#define LA_GDN_CMD_TLM_MID         0x1817  /* thrust, gimbal cmd — 10 Hz */

#define LA_ACS_CMD_MID              0x1918
#define LA_ACS_HK_TLM_MID          0x1818
#define LA_ACS_ACTUATOR_CMD_MID    0x1819  /* gimbal + RCS cmds — 40 Hz */

#define LA_TRN_CMD_MID              0x191A
#define LA_TRN_HK_TLM_MID          0x181A
#define LA_TRN_HAZARD_TLM_MID     0x181B  /* hazard map — 2 Hz */
#define LA_TRN_POSFIX_TLM_MID     0x181C  /* terrain-matched fix — 2 Hz */

/* ══════════════════════════════════════════════════════════
 * TIER 3 — MISSION MANAGEMENT
 * ══════════════════════════════════════════════════════════ */

#define LA_MM_CMD_MID               0x1920
#define LA_MM_HK_TLM_MID           0x1820
#define LA_MM_PHASE_TLM_MID        0x1821  /* flight phase — 1 Hz */
#define LA_MM_ABORT_CMD_MID        0x1921

#define LA_PROP_CMD_MID             0x1922
#define LA_PROP_HK_TLM_MID         0x1822
#define LA_PROP_STATUS_TLM_MID     0x1823  /* engine state, prop remaining */

#define LA_LG_CMD_MID               0x1924
#define LA_LG_HK_TLM_MID           0x1824
#define LA_LG_STATUS_TLM_MID       0x1825  /* deploy state, loads */

#define LA_DOCK_CMD_MID             0x1926
#define LA_DOCK_HK_TLM_MID         0x1826
#define LA_DOCK_STATUS_TLM_MID     0x1827  /* collar state, pressure eq */

/* ══════════════════════════════════════════════════════════
 * TIER 4 — VEHICLE SYSTEMS
 * ══════════════════════════════════════════════════════════ */

#define LA_EPS_CMD_MID              0x1930
#define LA_EPS_HK_TLM_MID          0x1830
#define LA_EPS_STATUS_TLM_MID      0x1831  /* bus V, SOC, load shed */

#define LA_TCS_CMD_MID              0x1932
#define LA_TCS_HK_TLM_MID          0x1832
#define LA_TCS_STATUS_TLM_MID      0x1833  /* radiator temps, heater state */

#define LA_LSS_CMD_MID              0x1934
#define LA_LSS_HK_TLM_MID          0x1834
#define LA_LSS_STATUS_TLM_MID      0x1835  /* cabin P, O2, CO2, humidity */

#define LA_COMM_CMD_MID             0x1936
#define LA_COMM_HK_TLM_MID         0x1836
#define LA_COMM_LINK_TLM_MID       0x1837  /* link state, SNR, HGA angles */

#define LA_LUNET_CMD_MID            0x1938
#define LA_LUNET_HK_TLM_MID        0x1838
#define LA_LUNET_BEACON_TLM_MID    0x1839  /* beacon data from surface node */
#define LA_LUNET_DIAG_TLM_MID      0x183A  /* field diagnostics */

/* ══════════════════════════════════════════════════════════
 * TIER 5 — HEALTH & SAFETY
 * ══════════════════════════════════════════════════════════ */

#define LA_HS_CMD_MID               0x1940
#define LA_HS_HK_TLM_MID           0x1840
#define LA_HS_ALERT_TLM_MID        0x1841  /* watchdog, critical alerts */

#define LA_LC_CMD_MID               0x1942
#define LA_LC_HK_TLM_MID           0x1842
#define LA_LC_ACTION_TLM_MID       0x1843  /* actionpoint triggers */

#define LA_SC_CMD_MID               0x1944
#define LA_SC_HK_TLM_MID           0x1844
#define LA_SC_EXEC_CMD_MID         0x1945  /* stored command execution */

/* ══════════════════════════════════════════════════════════
 * WAKEUP SLOT ASSIGNMENTS (via SCH)
 * ══════════════════════════════════════════════════════════ */

/*
 * The SCH app sends wakeup messages on minor frame ticks.
 * Slot mapping (1 Hz base, subdivided):
 *
 *  Slot 0x01 — IMU_MGR     (40 Hz — every minor frame)
 *  Slot 0x02 — ACS         (40 Hz — every minor frame)
 *  Slot 0x03 — NAV         (40 Hz — every minor frame)
 *  Slot 0x04 — ALT_MGR     (10 Hz — every 4th minor frame)
 *  Slot 0x05 — GDN         (10 Hz — every 4th minor frame)
 *  Slot 0x06 — PROP        (10 Hz — every 4th minor frame)
 *  Slot 0x07 — TRN          (2 Hz — every 20th minor frame)
 *  Slot 0x08 — MM           (1 Hz — major frame only)
 *  Slot 0x09 — EPS          (1 Hz)
 *  Slot 0x0A — LSS          (1 Hz)
 *  Slot 0x0B — COMM         (1 Hz)
 *  Slot 0x0C — LUNET        (1 Hz)
 *  Slot 0x0D — LG           (1 Hz)
 *  Slot 0x0E — DOCK         (1 Hz)
 *  Slot 0x0F — HK           (1 Hz)
 *  Slot 0x10 — TO           (1 Hz — downlink cycle)
 *  Slot 0x11 — HS           (1 Hz)
 *  Slot 0x12 — LC           (1 Hz)
 *  Slot 0x13 — TCS          (0.1 Hz — every 10th major frame)
 */

#endif /* LA_MSGIDS_H */
