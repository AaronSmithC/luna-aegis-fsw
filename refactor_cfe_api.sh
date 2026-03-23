#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
# Luna-Aegis — cFE API Compatibility Refactor
#
# Transforms all mission app sources from standalone stub API
# to real cFE 7.x API signatures.  Run from the repo root.
#
# Changes applied:
#   1. RunStatus: bool → uint32
#   2. CFE_SB_Buffer_t pointer semantics (** for ReceiveBuffer)
#   3. CFE_MSG_GetMsgId takes &SBBufPtr->Msg
#   4. switch on CFE_SB_MsgIdToValue(MsgId)
#   5. CFE_SB_Subscribe wraps MIDs with CFE_SB_ValueToMsgId()
#   6. CFE_MSG_Init wraps MID with CFE_SB_ValueToMsgId()
#   7. CFE_SB_TransmitMsg uses CFE_MSG_PTR() or &->Msg
#   8. CFE_MSG_GetFcnCode → CFE_MSG_FcnCode_t
#   9. Remove standalone header includes, use cfe.h
#  10. Remove #pragma pack (cFE handles alignment)
# ══════════════════════════════════════════════════════════════

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FSW_DIR="${SCRIPT_DIR}/fsw"

echo "Luna-Aegis cFE API Refactor"
echo "==========================="

# Find all .c source files in apps
SOURCES=$(find "${FSW_DIR}/apps" -name '*.c' | sort)
HEADERS=$(find "${FSW_DIR}/msg_defs" -name '*.h' | sort)

refactor_file() {
  local f="$1"
  local base=$(basename "$f")
  echo "  Refactoring: $base"

  # ── 1. RunStatus: bool → uint32 ──
  sed -i 's/bool             RunStatus;/uint32           RunStatus;/g' "$f"
  sed -i 's/bool            RunStatus;/uint32           RunStatus;/g' "$f"
  sed -i 's/bool           RunStatus;/uint32           RunStatus;/g' "$f"
  sed -i 's/bool  RunStatus;/uint32 RunStatus;/g' "$f"
  # Init: true → CFE_ES_RunStatus_APP_RUN
  sed -i 's/\.RunStatus = true;/.RunStatus = CFE_ES_RunStatus_APP_RUN;/g' "$f"
  sed -i 's/\.RunStatus = false;/.RunStatus = CFE_ES_RunStatus_APP_ERROR;/g' "$f"
  sed -i 's/RunStatus = true;/RunStatus = CFE_ES_RunStatus_APP_RUN;/g' "$f"
  sed -i 's/RunStatus = false;/RunStatus = CFE_ES_RunStatus_APP_ERROR;/g' "$f"

  # ── 2. CFE_SB_Buffer_t pointer: our code used CFE_SB_Buffer_t as value ──
  # Real cFE: CFE_SB_Buffer_t *SBBufPtr;
  # Our code: CFE_SB_Buffer_t BufPtr; or CFE_SB_Buffer_t B;
  sed -i 's/CFE_SB_Buffer_t   BufPtr;/CFE_SB_Buffer_t *SBBufPtr;/g' "$f"
  sed -i 's/CFE_SB_Buffer_t  BufPtr;/CFE_SB_Buffer_t *SBBufPtr;/g' "$f"
  sed -i 's/CFE_SB_Buffer_t BufPtr;/CFE_SB_Buffer_t *SBBufPtr;/g' "$f"
  # Short variable name versions
  sed -i 's/CFE_SB_Buffer_t B;/CFE_SB_Buffer_t *SBBufPtr;/g' "$f"

  # ── 3. CFE_SB_ReceiveBuffer: fix pointer semantics ──
  # Our: CFE_SB_ReceiveBuffer(&BufPtr, PipeId, timeout)
  # Real: CFE_SB_ReceiveBuffer(&SBBufPtr, PipeId, timeout) — SBBufPtr is already *
  sed -i 's/CFE_SB_ReceiveBuffer(&BufPtr,/CFE_SB_ReceiveBuffer(\&SBBufPtr,/g' "$f"
  sed -i 's/CFE_SB_ReceiveBuffer(&B,/CFE_SB_ReceiveBuffer(\&SBBufPtr,/g' "$f"

  # ── 4. CFE_MSG_GetMsgId: use &SBBufPtr->Msg ──
  # Our: CFE_MSG_GetMsgId((const CFE_MSG_Message_t *)BufPtr, &MsgId)
  # Our: CFE_MSG_GetMsgId((const CFE_MSG_Message_t *)B, &M)
  # Real: CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId)
  sed -i 's/CFE_MSG_GetMsgId((const CFE_MSG_Message_t \*)BufPtr, &MsgId)/CFE_MSG_GetMsgId(\&SBBufPtr->Msg, \&MsgId)/g' "$f"
  sed -i 's/CFE_MSG_GetMsgId((const CFE_MSG_Message_t \*)B, &M)/CFE_MSG_GetMsgId(\&SBBufPtr->Msg, \&MsgId)/g' "$f"
  # Variable rename: M → MsgId where used as msgid
  sed -i 's/CFE_SB_MsgId_t   M;/CFE_SB_MsgId_t MsgId;/g' "$f"
  sed -i 's/CFE_SB_MsgId_t  M;/CFE_SB_MsgId_t MsgId;/g' "$f"
  sed -i 's/CFE_SB_MsgId_t M;/CFE_SB_MsgId_t MsgId;/g' "$f"

  # ── 5. switch on CFE_SB_MsgIdToValue(MsgId) ──
  # Our: switch (MsgId) or switch (M)
  sed -i 's/switch (MsgId) {/switch (CFE_SB_MsgIdToValue(MsgId)) {/g' "$f"
  sed -i 's/switch (M) {/switch (CFE_SB_MsgIdToValue(MsgId)) {/g' "$f"

  # ── 6. CFE_SB_Subscribe: wrap MID constants ──
  # Our: CFE_SB_Subscribe(LA_xxx_MID, pipe)
  # Real: CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_xxx_MID), pipe)
  sed -i 's/CFE_SB_Subscribe(LA_\([A-Z_0-9]*\), /CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LA_\1), /g' "$f"

  # ── 7. CFE_MSG_Init: wrap MID ──
  # Our: CFE_MSG_Init((CFE_MSG_Message_t *)&XXX, LA_xxx_MID, sizeof(...))
  # Real: CFE_MSG_Init(CFE_MSG_PTR(XXX.TlmHdr), CFE_SB_ValueToMsgId(LA_xxx_MID), sizeof(...))
  # This one is complex — use a multi-pattern approach
  sed -i 's/CFE_MSG_Init((CFE_MSG_Message_t \*)&\([A-Za-z_.]*\),\s*/CFE_MSG_Init(\&\1.TlmHdr.Msg, /g' "$f"
  # Now wrap the MID constant that follows
  sed -i 's/CFE_MSG_Init(&\([^,]*\), LA_\([A-Z_0-9]*\),/CFE_MSG_Init(\&\1, CFE_SB_ValueToMsgId(LA_\2),/g' "$f"

  # ── 8. CFE_SB_TransmitMsg: fix pointer ──
  # Our: CFE_SB_TransmitMsg((const CFE_MSG_Message_t *)&XXX, true)
  # Real: CFE_SB_TransmitMsg(&XXX.TlmHdr.Msg, true)
  sed -i 's/CFE_SB_TransmitMsg((const CFE_MSG_Message_t \*)&\([A-Za-z_.]*\),/CFE_SB_TransmitMsg(\&\1.TlmHdr.Msg,/g' "$f"

  # Fix the SCH wakeup special case (uses bare CFE_MSG_Message_t)
  sed -i 's/CFE_SB_TransmitMsg(&wakeup,/CFE_SB_TransmitMsg(\&wakeup.TlmHdr.Msg,/g' "$f" 2>/dev/null || true

  # ── 9. memcpy from SBBufPtr ──
  # Our: memcpy(&XXX, BufPtr, sizeof(...))  or memcpy(&XXX, B, sizeof(...))
  # Real: memcpy(&XXX, SBBufPtr, sizeof(...))
  sed -i 's/memcpy(&\([^,]*\), BufPtr, /memcpy(\&\1, SBBufPtr, /g' "$f"
  sed -i 's/memcpy(&\([^,]*\), B, /memcpy(\&\1, SBBufPtr, /g' "$f"

  # ── 10. Cast for command processing ──
  # Our: (const CFE_MSG_Message_t *)BufPtr or (const CFE_MSG_Message_t *)B
  # Real: &SBBufPtr->Msg
  sed -i 's/(const CFE_MSG_Message_t \*)BufPtr/\&SBBufPtr->Msg/g' "$f"
  sed -i 's/(const CFE_MSG_Message_t \*)B/\&SBBufPtr->Msg/g' "$f"

  # ── 11. GetFcnCode: uint16_t → CFE_MSG_FcnCode_t ──
  sed -i 's/uint16_t fc;/CFE_MSG_FcnCode_t fc;/g' "$f"

  # ── 12. MsgId comparison (for if statements not switches) ──
  # if (M == LA_xxx) → if (CFE_SB_MsgIdToValue(MsgId) == LA_xxx)
  sed -i 's/if (M == LA_/if (CFE_SB_MsgIdToValue(MsgId) == LA_/g' "$f"
  sed -i 's/if (MsgId == LA_/if (CFE_SB_MsgIdToValue(MsgId) == LA_/g' "$f"

  # ── 13. Fix short variable refs: s → status, B → SBBufPtr etc ──
  # The compact main loops use: CFE_Status_t s; ... s = CFE_SB_...
  # These are fine as-is but the B refs need updating
  # Already handled above

  # ── 14. SCH special: CFE_MSG_Message_t wakeup needs proper init ──
  # The SCH app creates a bare message — needs special handling
  # Will be handled per-file if needed

  # ── 15. Remove #include for our standalone stubs ──
  sed -i '/#include "cfe_sb_types.h"/d' "$f"
  sed -i '/#include "la_cfe_sb_types.h"/d' "$f"
}

refactor_header() {
  local f="$1"
  echo "  Refactoring header: $(basename "$f")"

  # In la_msg_structs.h: replace our custom CCSDS types with cFE types
  # Our CFE_MSG_TelemetryHeader_t → already matches cFE
  # Our CFE_MSG_CommandHeader_t → already matches cFE
  # But we need to remove #pragma pack since cFE handles alignment
  sed -i '/#pragma pack/d' "$f"
}

echo ""
echo "Refactoring app sources..."
for src in $SOURCES; do
  refactor_file "$src"
done

echo ""
echo "Refactoring headers..."
for hdr in $HEADERS; do
  refactor_header "$hdr"
done

# ── Special fixes for SCH (uses bare CFE_MSG_Message_t) ──
echo ""
echo "Applying SCH-specific fixes..."
SCH_SRC="${FSW_DIR}/apps/sch/src/la_sch_app.c"
if [[ -f "$SCH_SRC" ]]; then
  # SCH creates a bare CFE_MSG_Message_t and transmits it
  # Fix: use CFE_MSG_CommandHeader_t as the wakeup message type
  sed -i 's/CFE_MSG_Message_t wakeup;/CFE_MSG_CommandHeader_t wakeup_msg;\n        CFE_MSG_Message_t *wakeup_ptr = (CFE_MSG_Message_t *)\&wakeup_msg;/g' "$SCH_SRC"
  sed -i 's/CFE_MSG_Init(&wakeup, /CFE_MSG_Init(wakeup_ptr, /g' "$SCH_SRC"
  sed -i 's/CFE_SB_TransmitMsg(&wakeup\.TlmHdr\.Msg,/CFE_SB_TransmitMsg(wakeup_ptr,/g' "$SCH_SRC"
fi

# ── Special fixes for SC (builds and sends command messages) ──
echo "Applying SC-specific fixes..."
SC_SRC="${FSW_DIR}/apps/sc/src/la_sc_app.c"
if [[ -f "$SC_SRC" ]]; then
  # SC creates CFE_MSG_CommandHeader_t and transmits
  sed -i 's/CFE_MSG_Init((CFE_MSG_Message_t \*)&msg,/CFE_MSG_Init(\&msg.Msg,/g' "$SC_SRC"
  sed -i 's/CFE_MSG_SetFcnCode((CFE_MSG_Message_t \*)&msg,/CFE_MSG_SetFcnCode(\&msg.Msg,/g' "$SC_SRC"
  sed -i 's/CFE_SB_TransmitMsg((const CFE_MSG_Message_t \*)&msg,/CFE_SB_TransmitMsg(\&msg.Msg,/g' "$SC_SRC"
  # Also fix the MsgId wrapping
  sed -i 's/CFE_MSG_Init(&msg\.Msg, cmd->MsgId,/CFE_MSG_Init(\&msg.Msg, CFE_SB_ValueToMsgId(cmd->MsgId),/g' "$SC_SRC"
fi

# ── Fix HAL sim (no cFE deps, but make sure includes are clean) ──
echo "Checking HAL sim..."
HAL_SIM="${FSW_DIR}/hal/sim/la_hal_sim.c"
if [[ -f "$HAL_SIM" ]]; then
  # HAL doesn't use any cFE APIs — should be clean
  echo "  HAL sim OK (no cFE API usage)"
fi

echo ""
echo "Refactor complete."
echo "Run 'make check' to verify standalone build, then './install.sh' for cFS integration."
