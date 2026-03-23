#!/usr/bin/env python3
"""Patch TO_LAB and SCH_LAB tables for Luna-Aegis integration."""
import re, sys, os

def patch_to_lab(path):
    if not os.path.exists(path):
        print(f"  WARN: {path} not found")
        return
    with open(path) as f:
        txt = f.read()
    if "LA_MM_PHASE" in txt:
        print("  TO_LAB already patched")
        return
    new_txt = '''#include "cfe_tbl_filedef.h"
#include "cfe_sb_api_typedefs.h"
#include "to_lab_tbl.h"
#include "cfe_msgids.h"
#include "to_lab_msgids.h"

TO_LAB_Subs_t Subscriptions = {
    .Subs = {
        /* cFE Core Telemetry */
        {CFE_SB_MSGID_WRAP_VALUE(CFE_ES_HK_TLM_MID),             {0, 0}, 4},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_EVS_HK_TLM_MID),            {0, 0}, 4},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_SB_HK_TLM_MID),             {0, 0}, 4},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TBL_HK_TLM_MID),            {0, 0}, 4},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TIME_HK_TLM_MID),           {0, 0}, 4},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_EVS_LONG_EVENT_MSG_MID),     {0, 0}, 32},
        /* Luna-Aegis Short Hopper Telemetry */
        {CFE_SB_MSGID_WRAP_VALUE(0x1821), {0, 0}, 4},  /* LA_MM_PHASE_TLM     */
        {CFE_SB_MSGID_WRAP_VALUE(0x1815), {0, 0}, 4},  /* LA_NAV_STATE_TLM    */
        {CFE_SB_MSGID_WRAP_VALUE(0x1823), {0, 0}, 4},  /* LA_PROP_STATUS_TLM  */
        {CFE_SB_MSGID_WRAP_VALUE(0x1831), {0, 0}, 4},  /* LA_EPS_STATUS_TLM   */
        {CFE_SB_MSGID_WRAP_VALUE(0x1835), {0, 0}, 4},  /* LA_LSS_STATUS_TLM   */
        {CFE_SB_MSGID_WRAP_VALUE(0x1841), {0, 0}, 4},  /* LA_HS_ALERT_TLM     */
        {CFE_SB_MSGID_WRAP_VALUE(0x1837), {0, 0}, 4},  /* LA_COMM_LINK_TLM    */
        {CFE_SB_MSGID_WRAP_VALUE(0x1825), {0, 0}, 4},  /* LA_LG_STATUS_TLM    */
        {CFE_SB_MSGID_WRAP_VALUE(0x1827), {0, 0}, 4},  /* LA_DOCK_STATUS_TLM  */
        {CFE_SB_MSGID_WRAP_VALUE(0x1833), {0, 0}, 4},  /* LA_TCS_STATUS_TLM   */
        {CFE_SB_MSGID_WRAP_VALUE(0x1839), {0, 0}, 4},  /* LA_LUNET_BEACON_TLM */
        {CFE_SB_MSGID_WRAP_VALUE(0x181B), {0, 0}, 4},  /* LA_TRN_HAZARD_TLM   */
        {CFE_SB_MSGID_WRAP_VALUE(0x1811), {0, 0}, 4},  /* LA_IMU_DATA_TLM     */
        {CFE_SB_MSGID_WRAP_VALUE(0x1813), {0, 0}, 4},  /* LA_ALT_DATA_TLM     */
        {CFE_SB_MSGID_WRAP_VALUE(0x1817), {0, 0}, 4},  /* LA_GDN_CMD_TLM      */
        {CFE_SB_MSGID_WRAP_VALUE(0x1819), {0, 0}, 4},  /* LA_ACS_ACTUATOR_TLM */
        {CFE_SB_MSGID_WRAP_VALUE(0x1843), {0, 0}, 4},  /* LA_LC_ACTION_TLM    */
        {CFE_SB_MSGID_RESERVED, {0, 0}, 0}
    }
};

CFE_TBL_FILEDEF(Subscriptions, TO_LAB.Subscriptions, TO Lab Sub Tbl, to_lab_sub.tbl)
'''
    with open(path, 'w') as f:
        f.write(new_txt)
    print("  TO_LAB patched with cFE core + 17 Luna-Aegis TLM MIDs")


def patch_sch_lab(path):
    if not os.path.exists(path):
        print(f"  WARN: {path} not found")
        return
    with open(path) as f:
        txt = f.read()
    if "LA_IMU" in txt:
        print("  SCH_LAB already patched")
        return
    # Write a complete clean table file
    new_txt = '''#include "cfe_tbl_filedef.h"
#include "sch_lab_tbl.h"
#include "cfe_sb_api_typedefs.h"
#include "cfe_msgids.h"

SCH_LAB_ScheduleTable_t Schedule = {
    .TickRate = 100,
    .Config   = {
        /* cFE Core HK requests at 1 Hz */
        {CFE_SB_MSGID_WRAP_VALUE(CFE_ES_SEND_HK_MID),  100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_EVS_SEND_HK_MID), 100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_SB_SEND_HK_MID),  100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TBL_SEND_HK_MID), 100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TIME_SEND_HK_MID),100, 0},
        /* Luna-Aegis 1 Hz wakeups */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A08), 100, 0},  /* MM      */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A09), 100, 0},  /* EPS     */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A0A), 100, 0},  /* LSS     */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A0B), 100, 0},  /* COMM    */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A0C), 100, 0},  /* LUNET   */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A0D), 100, 0},  /* LG      */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A0E), 100, 0},  /* DOCK    */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A11), 100, 0},  /* HS + SC */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A12), 100, 0},  /* LC      */
        /* Luna-Aegis 10 Hz wakeups */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A04), 10,  0},  /* ALT_MGR */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A05), 10,  0},  /* GDN     */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A06), 10,  0},  /* PROP    */
        /* Luna-Aegis 2 Hz */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A07), 50,  0},  /* TRN     */
        /* Luna-Aegis 0.1 Hz */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A13),1000, 0},  /* TCS     */
        /* Luna-Aegis 10 Hz (IMU/NAV/ACS) */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A01), 10,  0},  /* IMU_MGR */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A02), 10,  0},  /* ACS     */
        {CFE_SB_MSGID_WRAP_VALUE(0x1A03), 10,  0},  /* NAV     */
        {CFE_SB_MSGID_RESERVED, 0, 0},
    }
};

CFE_TBL_FILEDEF(Schedule, SCH_LAB.Schedule, Schedule Lab MsgID Table, sch_lab_table.tbl)
'''
    with open(path, 'w') as f:
        f.write(new_txt)
    print("  SCH_LAB patched with cFE HK + 18 Luna-Aegis wakeups")


if __name__ == "__main__":
    cfs_dir = sys.argv[1]
    patch_to_lab(f"{cfs_dir}/apps/to_lab/fsw/tables/to_lab_sub.c")
    patch_sch_lab(f"{cfs_dir}/apps/sch_lab/fsw/tables/sch_lab_table.c")
