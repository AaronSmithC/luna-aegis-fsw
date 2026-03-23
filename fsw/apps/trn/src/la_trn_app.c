/**
 * @file la_trn_app.c
 * @brief Luna-Aegis Terrain-Relative Navigation (TRN) Application
 *
 * Terrain matching for absolute position fix and hazard detection
 * in the landing zone.  Uses altimeter data + nav state to compare
 * against onboard terrain database.
 *
 * Subscribes: ALT_DataPkt (10 Hz), NAV_StatePkt (40 Hz),
 *             SCH Wakeup (2 Hz)
 * Publishes:  TRN_HazardPkt (2 Hz), TRN_PosFix (2 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include <string.h>
#include <math.h>

#define TRN_PIPE_DEPTH   16
#define TRN_PIPE_NAME    "TRN_PIPE"

/* Hazard thresholds */
#define TRN_MAX_SLOPE_DEG     15.0   /* Max safe landing slope */
#define TRN_ROUGHNESS_THRESH  0.5    /* m — surface roughness limit */
#define TRN_SHADOW_FLAG       0x01

/* Terrain DB (placeholder: flat south-pole reference) */
#define TRN_REF_ALT_M         0.0
#define TRN_MATCH_RADIUS_M    500.0  /* Correlation search radius */

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    LA_ALT_DataPkt_t   LastAlt;
    LA_NAV_StatePkt_t  LastNav;
    bool               AltValid;
    bool               NavValid;

    LA_TRN_HazardPkt_t HazardPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} TRN_AppData_t;

static TRN_AppData_t TRN;

static CFE_Status_t TRN_Init(void)
{
    memset(&TRN, 0, sizeof(TRN));
    TRN.RunStatus = CFE_ES_RunStatus_APP_RUN;

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&TRN.CmdPipe, TRN_PIPE_DEPTH, TRN_PIPE_NAME);

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_TRN_CMD_MID),          TRN.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_ALT_DATA_TLM_MID),     TRN.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_NAV_STATE_TLM_MID),    TRN.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x07)), TRN.CmdPipe);

    CFE_MSG_Init(&TRN.HazardPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_TRN_HAZARD_TLM_MID), sizeof(LA_TRN_HazardPkt_t));

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "TRN: Terrain-relative nav initialized @ 2 Hz");
    return CFE_SUCCESS;
}

static void TRN_Process(void)
{
    /*
     * Terrain matching algorithm (simplified):
     * 1. Take nav position estimate + altimeter reading
     * 2. Compare altimeter AGL against terrain DB at nav position
     * 3. Compute position correction from altitude residual
     * 4. Scan local terrain for hazards (slope, roughness)
     *
     * Full implementation would use DEM correlation, possibly
     * lidar point cloud matching.
     */

    bool fix_valid = false;
    double fix_x = 0.0, fix_y = 0.0, fix_z = 0.0;
    double fix_uncert = 999.0;
    double max_slope = 0.0;
    double hazard_dist = 9999.0;
    uint8_t hazard_count = 0;

    if (TRN.NavValid && TRN.AltValid) {
        /* Current nav position */
        double nav_x = TRN.LastNav.Pos_MCMF_m.x;
        double nav_y = TRN.LastNav.Pos_MCMF_m.y;
        double nav_z = TRN.LastNav.Pos_MCMF_m.z;

        /* Expected terrain altitude at this position (from DB) */
        double terrain_alt = TRN_REF_ALT_M; /* Flat terrain placeholder */

        /* Measured AGL */
        double meas_agl = TRN.LastAlt.AGL_m;

        /* Position fix: if AGL doesn't match expected terrain,
         * correct z-position.  Lateral correction requires
         * slope information from the terrain DB. */
        double expected_agl = nav_z - terrain_alt;
        double z_correction = meas_agl - expected_agl;

        fix_x = nav_x;  /* No lateral correction in simplified model */
        fix_y = nav_y;
        fix_z = nav_z + z_correction;

        fix_uncert = TRN.LastAlt.LidarValid ? 2.0 : 10.0;
        fix_valid = true;

        /* Hazard assessment from terrain slope */
        max_slope = fabs(TRN.LastAlt.TerrainSlope_deg);
        if (max_slope > TRN_MAX_SLOPE_DEG) {
            hazard_count++;
            hazard_dist = 0.0; /* Hazard at current position */
            CFE_EVS_SendEvent(10, CFE_EVS_EventType_ERROR,
                              "TRN: Slope hazard %.1f deg at landing site",
                              max_slope);
        } else {
            hazard_dist = TRN_MATCH_RADIUS_M; /* Clear to edge of scan */
        }
    }

    TRN.HazardPkt.PosFix_MCMF_m.x = fix_x;
    TRN.HazardPkt.PosFix_MCMF_m.y = fix_y;
    TRN.HazardPkt.PosFix_MCMF_m.z = fix_z;
    TRN.HazardPkt.FixUncert_m     = fix_uncert;
    TRN.HazardPkt.MaxSlope_deg    = max_slope;
    TRN.HazardPkt.HazardDist_m    = hazard_dist;
    TRN.HazardPkt.HazardCount     = hazard_count;
    TRN.HazardPkt.FixValid        = fix_valid ? 1 : 0;

    CFE_SB_TransmitMsg(&TRN.HazardPkt.TlmHdr.Msg, true);
}

void TRN_AppMain(void)
{
    CFE_Status_t     status;
    CFE_SB_Buffer_t *SBBufPtr;
    CFE_SB_MsgId_t   MsgId;

    if (TRN_Init() != CFE_SUCCESS) { TRN.RunStatus = CFE_ES_RunStatus_APP_ERROR; }

    while (CFE_ES_RunLoop(&TRN.RunStatus) == true) {
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, TRN.CmdPipe, CFE_SB_PEND_FOREVER);
        if (status != CFE_SUCCESS) continue;
        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x07):
            TRN_Process();
            TRN.CycleCount++;
            break;
        case LA_ALT_DATA_TLM_MID:
            memcpy(&TRN.LastAlt, SBBufPtr, sizeof(LA_ALT_DataPkt_t));
            TRN.AltValid = true;
            break;
        case LA_NAV_STATE_TLM_MID:
            memcpy(&TRN.LastNav, SBBufPtr, sizeof(LA_NAV_StatePkt_t));
            TRN.NavValid = true;
            break;
        case LA_TRN_CMD_MID:
            TRN.CmdCount++;
            break;
        default:
            break;
        }
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
