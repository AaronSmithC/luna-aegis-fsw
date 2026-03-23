/**
 * @file la_prop_app.c
 * @brief Luna-Aegis Propulsion Manager (PROP) Application
 *
 * Engine start/stop sequencing, throttle execution, propellant
 * tracking, and ISRU refueling coordination.
 *
 * Subscribes: GDN_CmdPkt (10 Hz), SCH Wakeup (10 Hz),
 *             MM_PhasePkt (for enable/disable)
 * Publishes:  PROP_StatusPkt (10 Hz)
 *
 * Design Authority: Aegis Station Infrastructure LLC
 * ITAR/EAR-Free Baseline
 */

#include "../../cfe_hdr/cfe.h"
#include "../../msg_defs/la_msgids.h"
#include "../../msg_defs/la_msg_structs.h"
#include "../../hal/inc/la_hal.h"
#include <string.h>
#include <math.h>

#define PROP_PIPE_DEPTH    32
#define PROP_PIPE_NAME     "PROP_PIPE"
#define PROP_DT            0.1   /* 10 Hz */

/* Engine startup sequence timing */
#define PROP_CHILL_TIME_S    3.0
#define PROP_IGNITION_TIME_S 0.5

/* Propellant consumption model */
#define PROP_ISP_S           440.0
#define PROP_G0              9.80665
#define PROP_MR_NOMINAL      6.0    /* O/F mass ratio for LOX/LH2 */

/* Command function codes */
#define PROP_NOOP_CC         0
#define PROP_RESET_CC        1
#define PROP_ARM_CC          2
#define PROP_DISARM_CC       3
#define PROP_START_CC        4
#define PROP_SHUTDOWN_CC     5

typedef enum {
    PROP_SEQ_IDLE     = 0,
    PROP_SEQ_ARMED    = 1,
    PROP_SEQ_CHILL    = 2,
    PROP_SEQ_IGNITION = 3,
    PROP_SEQ_RUNNING  = 4,
    PROP_SEQ_SHUTDOWN = 5,
} PROP_Sequence_t;

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;
    uint32_t         RunStatus;

    /* Engine sequence state */
    PROP_Sequence_t  Sequence;
    double           SeqTimer_s;

    /* Cached inputs */
    LA_GDN_CmdPkt_t LastGdnCmd;
    LA_MM_PhasePkt_t LastPhase;
    bool             GdnCmdValid;
    bool             PhaseValid;

    /* Prop tracking */
    double           OxMass_kg;
    double           FuelMass_kg;

    /* Output */
    LA_PROP_StatusPkt_t StatusPkt;

    uint32_t CycleCount;
    uint16_t CmdCount;
    uint16_t ErrCount;
} PROP_AppData_t;

static PROP_AppData_t PROP;

static CFE_Status_t PROP_Init(void)
{
    memset(&PROP, 0, sizeof(PROP));
    PROP.RunStatus = CFE_ES_RunStatus_APP_RUN;
    PROP.Sequence  = PROP_SEQ_IDLE;

    /* Initial propellant load */
    PROP.OxMass_kg   = 2062.5;  /* 75% of 2750 kg */
    PROP.FuelMass_kg = 687.5;   /* 25% of 2750 kg */

    CFE_EVS_Register(NULL, 0, 0);
    CFE_SB_CreatePipe(&PROP.CmdPipe, PROP_PIPE_DEPTH, PROP_PIPE_NAME);

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_PROP_CMD_MID),          PROP.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_GDN_CMD_TLM_MID),       PROP.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_MM_PHASE_TLM_MID),      PROP.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_SCH_WAKEUP_MID(0x06)),  PROP.CmdPipe); /* 10 Hz */

    CFE_MSG_Init(&PROP.StatusPkt.TlmHdr.Msg, 
                 CFE_SB_ValueToMsgId(LA_PROP_STATUS_TLM_MID),
                 sizeof(LA_PROP_StatusPkt_t));

    HAL_Engine_Init();
    HAL_CryoTank_Init();

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "PROP: Propulsion Manager initialized — IDLE");
    return CFE_SUCCESS;
}

static void PROP_UpdateSequence(void)
{
    PROP.SeqTimer_s += PROP_DT;

    switch (PROP.Sequence) {
    case PROP_SEQ_IDLE:
    case PROP_SEQ_ARMED:
        /* Waiting for command */
        break;

    case PROP_SEQ_CHILL:
        /* Engine chill-down: precool turbopumps */
        HAL_Engine_Command(HAL_ENGINE_CHILL, 0.0);
        if (PROP.SeqTimer_s >= PROP_CHILL_TIME_S) {
            PROP.Sequence   = PROP_SEQ_IGNITION;
            PROP.SeqTimer_s = 0.0;
            CFE_EVS_SendEvent(10, CFE_EVS_EventType_INFORMATION,
                              "PROP: Chill complete — IGNITION");
        }
        break;

    case PROP_SEQ_IGNITION:
        HAL_Engine_Command(HAL_ENGINE_IGNITION, 0.0);
        if (PROP.SeqTimer_s >= PROP_IGNITION_TIME_S) {
            /* Check chamber pressure for successful ignition */
            HAL_Engine_Tlm_t eng_tlm;
            HAL_Engine_Read(&eng_tlm);
            if (eng_tlm.chamber_press_kPa > 500.0) {
                PROP.Sequence   = PROP_SEQ_RUNNING;
                PROP.SeqTimer_s = 0.0;
                HAL_Engine_Command(HAL_ENGINE_RUNNING, 100.0);
                CFE_EVS_SendEvent(11, CFE_EVS_EventType_INFORMATION,
                                  "PROP: Ignition confirmed — ENGINE RUNNING");
            } else {
                /* Ignition failure */
                PROP.Sequence = PROP_SEQ_SHUTDOWN;
                PROP.SeqTimer_s = 0.0;
                CFE_EVS_SendEvent(12, CFE_EVS_EventType_CRITICAL,
                                  "PROP: IGNITION FAILURE — Pc=%.1f kPa — SHUTDOWN",
                                  eng_tlm.chamber_press_kPa);
            }
        }
        break;

    case PROP_SEQ_RUNNING:
        {
            /* Use GDN throttle command if available, otherwise default 100% */
            double throttle = 100.0;
            double thrust   = 30000.0; /* default full thrust */
            if (PROP.GdnCmdValid) {
                throttle = PROP.LastGdnCmd.ThrottleCmd_pct;
                if (throttle < 0.0) throttle = 0.0;
                if (throttle > 100.0) throttle = 100.0;
                thrust = PROP.LastGdnCmd.ThrustCmd_N;
                if (thrust < 0.0) thrust = 0.0;
            }
            /* Engine is lit — enforce minimum idle thrust (10% throttle) */
            if (throttle < 10.0) throttle = 10.0;
            if (thrust < 3000.0) thrust = 3000.0;
            HAL_Engine_Command(HAL_ENGINE_RUNNING, throttle);

            /* Propellant consumption model */
            if (thrust > 0.0) {
                double mdot = thrust / (PROP_ISP_S * PROP_G0);
                double ox_dot  = mdot * PROP_MR_NOMINAL / (1.0 + PROP_MR_NOMINAL);
                double fuel_dot = mdot / (1.0 + PROP_MR_NOMINAL);

                PROP.OxMass_kg   -= ox_dot * PROP_DT;
                PROP.FuelMass_kg -= fuel_dot * PROP_DT;

                if (PROP.OxMass_kg < 0.0)   PROP.OxMass_kg = 0.0;
                if (PROP.FuelMass_kg < 0.0)  PROP.FuelMass_kg = 0.0;
            }
        }
        break;

    case PROP_SEQ_SHUTDOWN:
        HAL_Engine_Command(HAL_ENGINE_SHUTDOWN, 0.0);
        if (PROP.SeqTimer_s >= 2.0) {
            PROP.Sequence = PROP_SEQ_IDLE;
            HAL_Engine_Command(HAL_ENGINE_OFF, 0.0);
            CFE_EVS_SendEvent(13, CFE_EVS_EventType_INFORMATION,
                              "PROP: Engine shutdown complete");
        }
        break;
    }
}

static void PROP_ProcessCommand(const CFE_MSG_Message_t *MsgPtr)
{
    CFE_MSG_FcnCode_t fc;
    CFE_MSG_GetFcnCode(MsgPtr, &fc);

    switch (fc) {
    case PROP_NOOP_CC:
        PROP.CmdCount++;
        break;

    case PROP_RESET_CC:
        PROP.CmdCount = 0;
        PROP.ErrCount = 0;
        break;

    case PROP_ARM_CC:
        if (PROP.Sequence == PROP_SEQ_IDLE) {
            PROP.Sequence = PROP_SEQ_ARMED;
            PROP.CmdCount++;
            CFE_EVS_SendEvent(20, CFE_EVS_EventType_INFORMATION,
                              "PROP: Engine ARMED");
        } else {
            PROP.ErrCount++;
        }
        break;

    case PROP_DISARM_CC:
        if (PROP.Sequence == PROP_SEQ_ARMED) {
            PROP.Sequence = PROP_SEQ_IDLE;
            PROP.CmdCount++;
        } else {
            PROP.ErrCount++;
        }
        break;

    case PROP_START_CC:
        if (PROP.Sequence == PROP_SEQ_ARMED) {
            PROP.Sequence   = PROP_SEQ_CHILL;
            PROP.SeqTimer_s = 0.0;
            PROP.CmdCount++;
            CFE_EVS_SendEvent(21, CFE_EVS_EventType_INFORMATION,
                              "PROP: Engine start sequence initiated — CHILL");
        } else {
            PROP.ErrCount++;
            CFE_EVS_SendEvent(22, CFE_EVS_EventType_ERROR,
                              "PROP: START rejected — engine not armed");
        }
        break;

    case PROP_SHUTDOWN_CC:
        if (PROP.Sequence == PROP_SEQ_RUNNING ||
            PROP.Sequence == PROP_SEQ_CHILL   ||
            PROP.Sequence == PROP_SEQ_IGNITION ||
            PROP.Sequence == PROP_SEQ_ARMED)
        {
            PROP.Sequence   = PROP_SEQ_SHUTDOWN;
            PROP.SeqTimer_s = 0.0;
            PROP.CmdCount++;
            HAL_Engine_Command(HAL_ENGINE_SHUTDOWN, 0.0);
            CFE_EVS_SendEvent(23, CFE_EVS_EventType_INFORMATION,
                              "PROP: Commanded shutdown");
        } else if (PROP.Sequence == PROP_SEQ_IDLE) {
            /* Already idle — just ack */
            PROP.CmdCount++;
        } else {
            PROP.ErrCount++;
        }
        break;

    default:
        PROP.ErrCount++;
        break;
    }
}

static void PROP_SendStatusPkt(void)
{
    HAL_Engine_Tlm_t    eng;
    HAL_CryoTank_Tlm_t cryo;

    HAL_Engine_Read(&eng);
    HAL_CryoTank_Read(&cryo);

    PROP.StatusPkt.ChamberPress_kPa = eng.chamber_press_kPa;
    PROP.StatusPkt.MixtureRatio     = eng.mixture_ratio;
    PROP.StatusPkt.ThrustEst_N      = eng.thrust_est_N;
    PROP.StatusPkt.EngineState      = eng.state;
    PROP.StatusPkt.ValveStates      = eng.valve_positions;
    PROP.StatusPkt.OxTankPress_kPa  = cryo.ox_press_kPa;
    PROP.StatusPkt.FuelTankPress_kPa= cryo.fuel_press_kPa;
    PROP.StatusPkt.OxMass_kg        = PROP.OxMass_kg;
    PROP.StatusPkt.FuelMass_kg      = PROP.FuelMass_kg;

    CFE_SB_TransmitMsg(&PROP.StatusPkt.TlmHdr.Msg, true);
}

void PROP_AppMain(void)
{
    CFE_Status_t     status;
    CFE_SB_Buffer_t *SBBufPtr;
    CFE_SB_MsgId_t   MsgId;

    if (PROP_Init() != CFE_SUCCESS) {
        PROP.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&PROP.RunStatus) == true) {
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, PROP.CmdPipe,
                                      CFE_SB_PEND_FOREVER);
        if (status != CFE_SUCCESS) continue;

        CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

        switch (CFE_SB_MsgIdToValue(MsgId)) {
        case LA_SCH_WAKEUP_MID(0x06):
            PROP_UpdateSequence();
            PROP_SendStatusPkt();
            PROP.CycleCount++;
            break;

        case LA_GDN_CMD_TLM_MID:
            memcpy(&PROP.LastGdnCmd, SBBufPtr, sizeof(LA_GDN_CmdPkt_t));
            PROP.GdnCmdValid = true;
            break;

        case LA_MM_PHASE_TLM_MID:
            memcpy(&PROP.LastPhase, SBBufPtr, sizeof(LA_MM_PhasePkt_t));
            PROP.PhaseValid = true;
            /* Auto-shutdown on LANDED or SAFED */
            if (PROP.LastPhase.CurrentPhase == LA_PHASE_LANDED ||
                PROP.LastPhase.CurrentPhase == LA_PHASE_SAFED)
            {
                if (PROP.Sequence == PROP_SEQ_RUNNING) {
                    PROP.Sequence   = PROP_SEQ_SHUTDOWN;
                    PROP.SeqTimer_s = 0.0;
                }
            }
            break;

        case LA_PROP_CMD_MID:
            PROP_ProcessCommand(&SBBufPtr->Msg);
            break;

        default:
            break;
        }
    }

    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}
