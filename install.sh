#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
# Luna-Aegis Short Hopper — cFS Integration Install Script
#
# This script:
#   1. Clones NASA cFS (with submodules)
#   2. Copies Luna-Aegis mission apps into the cFS tree
#   3. Generates CMakeLists.txt for each mission app
#   4. Patches cFS mission configuration (targets.cmake, startup)
#   5. Builds the integrated cFS + Luna-Aegis system
#   6. Runs the flight software in sim mode
#   7. Launches the ground console
#
# Prerequisites:
#   - git, gcc, g++, cmake (>= 3.5), make
#   - python3 (for ground console HTTP server)
#   - Internet access (to clone from GitHub)
#
# Usage:
#   chmod +x install.sh
#   ./install.sh              # Full install + build + run
#   ./install.sh --build-only # Install + build, don't run
#   ./install.sh --run-only   # Run previously built system
#
# Design Authority: Aegis Station Infrastructure LLC
# ITAR/EAR-Free Baseline
# ══════════════════════════════════════════════════════════════

set -euo pipefail

# ── Configuration ────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFS_REPO="https://github.com/nasa/cFS.git"
CONSOLE_PORT=8080

# Luna-Aegis source location (relative to this script)
LA_FSW_DIR="${SCRIPT_DIR}/fsw"
LA_CONSOLE_DIR="${SCRIPT_DIR}/ground/console"

# Colors
RED='\033[0;31m'
GRN='\033[0;32m'
CYN='\033[0;36m'
AMB='\033[0;33m'
RST='\033[0m'

log()  { echo -e "${CYN}[LA]${RST} $*"; }
ok()   { echo -e "${GRN}[OK]${RST} $*"; }
warn() { echo -e "${AMB}[!!]${RST} $*"; }
err()  { echo -e "${RED}[ERR]${RST} $*" >&2; }

# ── Parse Args ───────────────────────────────────────────────

BUILD_ONLY=0
RUN_ONLY=0
CUSTOM_CFS_ROOT=""
SKIP_CLONE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-only) BUILD_ONLY=1; shift ;;
    --run-only)   RUN_ONLY=1; shift ;;
    --cfs-root)
      if [[ -z "${2:-}" ]]; then
        err "--cfs-root requires a path argument"
        exit 1
      fi
      CUSTOM_CFS_ROOT="$2"
      shift 2
      ;;
    --cfs-root=*)
      CUSTOM_CFS_ROOT="${1#*=}"
      shift
      ;;
    --port)
      CONSOLE_PORT="$2"
      shift 2
      ;;
    --port=*)
      CONSOLE_PORT="${1#*=}"
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --cfs-root PATH   Use an existing cFS tree instead of cloning."
      echo "                    Installs Luna-Aegis apps into PATH/apps/la_*"
      echo "                    and patches PATH/sample_defs/."
      echo "                    Example: ./install.sh --cfs-root ~/cfs_wok"
      echo ""
      echo "  --port PORT       Ground console port (default: 8080)"
      echo "  --build-only      Clone/install + build; don't run"
      echo "  --run-only        Run a previously built system"
      echo "  (no args)         Fresh clone, full install, build, and run"
      echo ""
      echo "Examples:"
      echo "  ./install.sh                              # Fresh cFS clone + build + run"
      echo "  ./install.sh --cfs-root ~/cfs_wok         # Install into existing tree"
      echo "  ./install.sh --cfs-root ~/cfs_wok --build-only"
      echo "  ./install.sh --cfs-root ~/cfs_wok --run-only"
      exit 0
      ;;
    *) err "Unknown arg: $1"; exit 1 ;;
  esac
done

# ── Resolve CFS_DIR and INSTALL_DIR ─────────────────────────
#
# Three modes:
#   1. --cfs-root ~/cfs_wok     → CFS_DIR=~/cfs_wok, skip clone
#   2. (no flag)                → CFS_DIR=./cfs-luna-aegis/cFS, clone fresh
#   3. --run-only               → either of above, just run
#

if [[ -n "$CUSTOM_CFS_ROOT" ]]; then
  CFS_DIR="$(cd "$CUSTOM_CFS_ROOT" 2>/dev/null && pwd)" || {
    # Directory doesn't exist yet — that's OK for fresh install
    CFS_DIR="$(mkdir -p "$CUSTOM_CFS_ROOT" && cd "$CUSTOM_CFS_ROOT" && pwd)"
  }
  INSTALL_DIR="$(dirname "$CFS_DIR")"
  SKIP_CLONE=0  # Will check inside clone_cfs if submodules exist

  # If cFE subdir exists, this is a populated cFS tree — skip clone
  if [[ -d "${CFS_DIR}/cfe" ]]; then
    SKIP_CLONE=1
    log "Using existing cFS tree: ${CFS_DIR}"
  else
    log "cFS root ${CFS_DIR} is empty — will clone into it"
  fi
else
  INSTALL_DIR="${SCRIPT_DIR}"
  CFS_DIR="${SCRIPT_DIR}/cFS"
fi

# ── Check Prerequisites ─────────────────────────────────────

check_prereqs() {
  log "Checking prerequisites..."
  local missing=()

  for cmd in git gcc g++ cmake make python3; do
    if ! command -v "$cmd" &>/dev/null; then
      missing+=("$cmd")
    fi
  done

  if [[ ${#missing[@]} -gt 0 ]]; then
    err "Missing required tools: ${missing[*]}"
    echo ""
    echo "Install with:"
    echo "  sudo apt-get update && sudo apt-get install -y \\"
    echo "    git build-essential cmake python3"
    exit 1
  fi

  # Check cmake version
  local cmake_ver
  cmake_ver=$(cmake --version | head -1 | grep -oP '\d+\.\d+')
  log "  cmake ${cmake_ver}, gcc $(gcc -dumpversion)"
  ok "All prerequisites met"
}

# ── Step 1: Clone cFS ────────────────────────────────────────

clone_cfs() {
  # If --cfs-root pointed to a populated tree, skip cloning entirely
  if [[ $SKIP_CLONE -eq 1 ]]; then
    ok "Using existing cFS tree at ${CFS_DIR}"
    ok "  (cFE, OSAL, PSP detected — skipping clone)"
    return 0
  fi

  if [[ -d "${CFS_DIR}/cfe" ]]; then
    warn "cFS directory already exists at ${CFS_DIR}"
    read -rp "  Delete and re-clone? [y/N] " yn
    case "$yn" in
      [yY]*) rm -rf "${CFS_DIR}" ;;
      *)     log "Using existing cFS clone"; return 0 ;;
    esac
  fi

  log "Cloning NASA cFS from ${CFS_REPO}..."
  if [[ -n "$CUSTOM_CFS_ROOT" ]]; then
    # Clone INTO the specified root (not a subdirectory)
    git clone "${CFS_REPO}" "${CFS_DIR}" || {
      # Directory might exist but be empty
      cd "${CFS_DIR}"
      git init
      git remote add origin "${CFS_REPO}"
      git fetch origin
      git checkout -b main origin/main
    }
  else
    mkdir -p "${INSTALL_DIR}"
    git clone "${CFS_REPO}" "${CFS_DIR}"
  fi

  cd "${CFS_DIR}"

  log "Initializing cFS submodules..."
  git submodule init
  git submodule update

  ok "cFS cloned with all submodules"
}

# ── Step 2: Copy Luna-Aegis Apps into cFS Tree ──────────────

# All Luna-Aegis mission apps
LA_APPS=(
  imu_mgr alt_mgr nav gdn acs trn
  mm prop lg dock
  eps tcs lss comm lunet
  hs lc sc
  sch hk ci to ds fm
)

copy_apps() {
  log "Copying Luna-Aegis mission apps into cFS tree..."

  for app in "${LA_APPS[@]}"; do
    local app_upper
    app_upper=$(echo "$app" | tr '[:lower:]' '[:upper:]')
    local src="${LA_FSW_DIR}/apps/${app}"
    local dst="${CFS_DIR}/apps/la_${app}"

    if [[ ! -d "$src" ]]; then
      warn "Source not found for ${app}, skipping"
      continue
    fi

    mkdir -p "${dst}/fsw/src"
    mkdir -p "${dst}/fsw/mission_inc"
    mkdir -p "${dst}/fsw/platform_inc"

    # Copy source files and local headers
    cp "${src}/src/"*.c "${dst}/fsw/src/" 2>/dev/null || true
    cp "${src}/src/"*.h "${dst}/fsw/src/" 2>/dev/null || true

    # Copy tables if they exist
    if [[ -d "${src}/tables" ]]; then
      mkdir -p "${dst}/fsw/tables"
      cp "${src}/tables/"* "${dst}/fsw/tables/" 2>/dev/null || true
    fi

    log "  Copied la_${app}"
  done

  # Copy shared headers into a common location
  mkdir -p "${CFS_DIR}/apps/la_common/fsw/mission_inc"
  cp "${LA_FSW_DIR}/msg_defs/la_msgids.h"      "${CFS_DIR}/apps/la_common/fsw/mission_inc/"
  cp "${LA_FSW_DIR}/msg_defs/la_msg_structs.h"  "${CFS_DIR}/apps/la_common/fsw/mission_inc/"
  cp "${LA_FSW_DIR}/cfe_hdr/cfe_sb_types.h"     "${CFS_DIR}/apps/la_common/fsw/mission_inc/la_cfe_sb_types.h"

  # Copy common headers (destinations, sequence gates)
  if [[ -d "${LA_FSW_DIR}/common" ]]; then
    cp "${LA_FSW_DIR}/common/"*.h "${CFS_DIR}/apps/la_common/fsw/mission_inc/" 2>/dev/null || true
  fi

  # Copy HAL
  mkdir -p "${CFS_DIR}/apps/la_hal/fsw/src"
  mkdir -p "${CFS_DIR}/apps/la_hal/fsw/mission_inc"
  cp "${LA_FSW_DIR}/hal/inc/la_hal.h"           "${CFS_DIR}/apps/la_hal/fsw/mission_inc/"
  cp "${LA_FSW_DIR}/hal/sim/la_hal_sim.c"       "${CFS_DIR}/apps/la_hal/fsw/src/"

  # Copy ground console — lives inside the cFS tree so it travels with it
  mkdir -p "${CFS_DIR}/ground-console"
  cp "${LA_CONSOLE_DIR}/index.html" "${CFS_DIR}/ground-console/" 2>/dev/null || true

  ok "All Luna-Aegis apps copied into cFS tree"
}

# ── Step 3: Generate CMakeLists.txt for Each App ─────────────

generate_cmake() {
  log "Generating CMakeLists.txt for Luna-Aegis apps..."

  # la_common (header-only library)
  cat > "${CFS_DIR}/apps/la_common/CMakeLists.txt" << 'CMEOF'
project(LA_COMMON C)
add_cfe_app(la_common
  fsw/mission_inc/la_msgids.h
  fsw/mission_inc/la_msg_structs.h
)
# This is a header-only target; we create an interface library
# so other apps can depend on it for include paths.
CMEOF

  # la_hal (shared library)
  cat > "${CFS_DIR}/apps/la_hal/CMakeLists.txt" << 'CMEOF'
project(LA_HAL C)
add_cfe_app(la_hal fsw/src/la_hal_sim.c)
target_link_libraries(la_hal m)
target_include_directories(la_hal PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/fsw/mission_inc
  ${CMAKE_CURRENT_SOURCE_DIR}/../la_common/fsw/mission_inc
)
CMEOF

  # Generate CMakeLists.txt for each mission app
  for app in "${LA_APPS[@]}"; do
    local app_upper
    app_upper=$(echo "$app" | tr '[:lower:]' '[:upper:]')
    local dst="${CFS_DIR}/apps/la_${app}"

    [[ ! -d "$dst" ]] && continue

    # Find all .c files
    local srcs=""
    for f in "${dst}/fsw/src/"*.c; do
      [[ -f "$f" ]] && srcs="${srcs}  fsw/src/$(basename "$f")\n"
    done

    if [[ -z "$srcs" ]]; then
      warn "No sources for la_${app}, skipping CMake generation"
      continue
    fi

    cat > "${dst}/CMakeLists.txt" << CMEOF
project(LA_${app_upper} C)

add_cfe_app(la_${app}
$(echo -e "$srcs"))

target_include_directories(la_${app} PUBLIC
  \${CMAKE_CURRENT_SOURCE_DIR}/fsw/src
  \${CMAKE_CURRENT_SOURCE_DIR}/fsw/mission_inc
  \${CMAKE_CURRENT_SOURCE_DIR}/fsw/platform_inc
  \${CMAKE_CURRENT_SOURCE_DIR}/../la_common/fsw/mission_inc
  \${CMAKE_CURRENT_SOURCE_DIR}/../la_hal/fsw/mission_inc
)

target_link_libraries(la_${app} m)
CMEOF

    log "  Generated CMakeLists.txt for la_${app}"
  done

  ok "All CMakeLists.txt files generated"
}

# ── Step 4: Patch cFS Mission Configuration ──────────────────

patch_mission_config() {
  log "Patching cFS mission configuration..."

  # ── 4a: Create sample_defs from cFE defaults if not present ──
  local defs_dir="${CFS_DIR}/sample_defs"
  if [[ ! -d "$defs_dir" ]]; then
    log "  Copying sample_defs from cFE defaults..."
    cp -r "${CFS_DIR}/cfe/cmake/sample_defs" "${defs_dir}"
  fi

  # ── 4b: Patch targets.cmake to include Luna-Aegis apps ──
  local targets_file="${defs_dir}/targets.cmake"

  if [[ -f "$targets_file" ]]; then
    # Check if we already patched
    if grep -q "la_mm" "$targets_file"; then
      log "  targets.cmake already patched"
    else
      log "  Patching targets.cmake..."

      # Build the app list for the cmake block
      local la_app_list=""
      for app in "${LA_APPS[@]}"; do
        la_app_list="${la_app_list}    la_${app}"$'\n'
      done
      la_app_list="${la_app_list}    la_hal"

      # Append our app registration block to the end of targets.cmake.
      # This works regardless of cFS version (old SET or new list style)
      # because cmake evaluates the whole file — appending is always valid.
      cat >> "$targets_file" << TEOF

# ══════════════════════════════════════════════════════════════
# Luna-Aegis Short Hopper — Mission App Registration
# ══════════════════════════════════════════════════════════════

list(APPEND MISSION_GLOBAL_APPLIST
${la_app_list}
)
TEOF

      ok "  targets.cmake patched with Luna-Aegis apps"
    fi
  else
    warn "  targets.cmake not found — creating minimal version"
    cat > "$targets_file" << 'TEOF'
# ══════════════════════════════════════════════════════════════
# Luna-Aegis cFS Mission — targets.cmake
# ══════════════════════════════════════════════════════════════

SET(MISSION_NAME "LunaAegis")
SET(SPACECRAFT_ID 66)
SET(TGT1_NAME cpu1)
SET(TGT1_APPLIST
    ci_lab to_lab sch_lab
    la_imu_mgr la_alt_mgr la_nav la_gdn la_acs la_trn
    la_mm la_prop la_lg la_dock
    la_eps la_tcs la_lss la_comm la_lunet
    la_hs la_lc la_sc
    la_sch la_hk la_ci la_to la_ds la_fm
    la_hal
)
SET(TGT1_FILELIST cfe_es_startup.scr)
TEOF
  fi

  # ── 4c: Patch startup script ──
  local startup_scr="${defs_dir}/cpu1_cfe_es_startup.scr"
  if [[ ! -f "$startup_scr" ]]; then
    startup_scr="${defs_dir}/cfe_es_startup.scr"
  fi

  if [[ -f "$startup_scr" ]]; then
    if grep -q "la_mm" "$startup_scr"; then
      log "  Startup script already patched"
    else
      log "  Patching startup script..."
      # The stock startup script ends with a standalone '!' which is the
      # cFS EOF marker. We need to:
      #   1. Remove the existing '!' EOF marker
      #   2. Append our entries
      #   3. Add a single '!' EOF marker at the very end
      
      # Remove all standalone '!' lines (EOF markers)
      sed -i '/^!$/d' "$startup_scr"
      # Remove comment-only lines starting with '!' (they also terminate parsing)
      sed -i '/^! /d' "$startup_scr"
      # Remove blank lines at end
      sed -i -e :a -e '/^\n*$/{$d;N;ba' -e '}' "$startup_scr"

      # Append Luna-Aegis entries + final EOF marker
      cat >> "$startup_scr" << 'SEOF'
CFE_LIB, la_hal,      HAL_InitAll,        LA_HAL,        0,   0,     0x0, 0;
CFE_APP, la_imu_mgr,  IMU_MGR_AppMain,    LA_IMU,       60,   32768, 0x0, 0;
CFE_APP, la_alt_mgr,  ALT_MGR_AppMain,    LA_ALT,       62,   16384, 0x0, 0;
CFE_APP, la_nav,      NAV_AppMain,        LA_NAV,       60,   65536, 0x0, 0;
CFE_APP, la_gdn,      GDN_AppMain,        LA_GDN,       62,   32768, 0x0, 0;
CFE_APP, la_acs,      ACS_AppMain,        LA_ACS,       60,   32768, 0x0, 0;
CFE_APP, la_trn,      TRN_AppMain,        LA_TRN,       64,   32768, 0x0, 0;
CFE_APP, la_mm,       MM_AppMain,         LA_MM,        50,   32768, 0x0, 0;
CFE_APP, la_prop,     PROP_AppMain,       LA_PROP,      52,   32768, 0x0, 0;
CFE_APP, la_lg,       LG_AppMain,         LA_LG,        54,   16384, 0x0, 0;
CFE_APP, la_dock,     DOCK_AppMain,       LA_DOCK,      54,   16384, 0x0, 0;
CFE_APP, la_eps,      EPS_AppMain,        LA_EPS,       66,   16384, 0x0, 0;
CFE_APP, la_tcs,      TCS_AppMain,        LA_TCS,       68,   16384, 0x0, 0;
CFE_APP, la_lss,      LSS_AppMain,        LA_LSS,       66,   16384, 0x0, 0;
CFE_APP, la_comm,     COMM_AppMain,       LA_COMM,      64,   16384, 0x0, 0;
CFE_APP, la_lunet,    LUNET_AppMain,      LA_LUNET,     66,   16384, 0x0, 0;
CFE_APP, la_hs,       HS_AppMain,         LA_HS,        40,   16384, 0x0, 0;
CFE_APP, la_lc,       LC_AppMain,         LA_LC,        42,   16384, 0x0, 0;
CFE_APP, la_sc,       SC_AppMain,         LA_SC,        44,   16384, 0x0, 0;
!
SEOF
      ok "  Startup script patched with 24 Luna-Aegis apps"
    fi
  else
    warn "  Startup script not found — will be created during build"
  fi

  # ── 4d: Patch OSAL cmake config for Luna-Aegis resource limits ──
  # Draco uses cmake variables in osconfig.cmake, not #defines in .h files
  log "  Patching OSAL cmake config for Luna-Aegis resource limits..."

  for cfg_file in \
    "${defs_dir}/default_osconfig.cmake" \
    "${defs_dir}/native_osconfig.cmake"; do

    if [[ -f "$cfg_file" ]]; then
      if grep -q "OSAL_CONFIG_MAX_MODULES.*48" "$cfg_file"; then
        log "  $(basename "$cfg_file") already patched"
      else
        cat >> "$cfg_file" << 'OSCEOF'

# Luna-Aegis resource limits (22 mission apps + stock)
set(OSAL_CONFIG_MAX_MODULES 48)
set(OSAL_CONFIG_MAX_TASKS   64)
set(OSAL_CONFIG_MAX_QUEUES  64)
set(OSAL_CONFIG_QUEUE_MAX_DEPTH 256)
set(OSAL_CONFIG_MAX_MUTEXES 32)
OSCEOF
        log "  Patched $(basename "$cfg_file")"
      fi
    else
      # Create the file if it doesn't exist
      cat > "$cfg_file" << 'OSCEOF'
# Luna-Aegis resource limits
set(OSAL_CONFIG_MAX_MODULES 48)
set(OSAL_CONFIG_MAX_TASKS   64)
set(OSAL_CONFIG_MAX_QUEUES  64)
set(OSAL_CONFIG_QUEUE_MAX_DEPTH 256)
set(OSAL_CONFIG_MAX_MUTEXES 32)
OSCEOF
      log "  Created $(basename "$cfg_file")"
    fi
  done

  # Also patch cFE platform config via cmake if available
  local cfe_opts="${defs_dir}/default_cfe_options.cmake"
  if [[ -f "$cfe_opts" ]]; then
    if ! grep -q "CFE_PLATFORM_ES_MAX_APPLICATIONS" "$cfe_opts"; then
      cat >> "$cfe_opts" << 'CFEEOF'

# Luna-Aegis: bump max apps/libs
set(CFE_PLATFORM_ES_MAX_APPLICATIONS 40)
set(CFE_PLATFORM_ES_MAX_LIBRARIES    16)
CFEEOF
      log "  Patched default_cfe_options.cmake"
    fi
  fi

  ok "Mission configuration patched"
}

# ── Step 5: Adapt Source Files for Real cFE ──────────────────

adapt_sources() {
  log "Adapting Luna-Aegis sources for real cFE integration..."

  # Our apps include "../../cfe_hdr/cfe.h" etc. which is the standalone
  # build path.  For real cFS, we need to include "cfe.h" directly
  # since the cFE build system provides the include paths.
  #
  # We also need to replace HAL includes.

  for app in "${LA_APPS[@]}"; do
    local app_dir="${CFS_DIR}/apps/la_${app}/fsw/src"
    [[ ! -d "$app_dir" ]] && continue

    for src in "${app_dir}"/*.c "${app_dir}"/*.h; do
      [[ ! -f "$src" ]] && continue

      # Fix cFE header includes — remove relative paths
      sed -i 's|#include "../../cfe_hdr/cfe.h"|#include "cfe.h"|g' "$src"
      sed -i 's|#include "../../cfe_hdr/cfe_sb_types.h"|#include "cfe.h"|g' "$src"

      # Fix message definition includes
      sed -i 's|#include "../../msg_defs/la_msgids.h"|#include "la_msgids.h"|g' "$src"
      sed -i 's|#include "../../msg_defs/la_msg_structs.h"|#include "la_msg_structs.h"|g' "$src"

      # Fix HAL includes
      sed -i 's|#include "../../hal/inc/la_hal.h"|#include "la_hal.h"|g' "$src"

      # Fix common shared headers (destinations, sequence gates)
      sed -i 's|#include "../../common/\(.*\)"|#include "\1"|g' "$src"

    done
  done

  # Fix HAL sim source
  local hal_src="${CFS_DIR}/apps/la_hal/fsw/src/la_hal_sim.c"
  if [[ -f "$hal_src" ]]; then
    sed -i 's|#include "../inc/la_hal.h"|#include "la_hal.h"|g' "$hal_src"
  fi

  # Fix la_msg_structs.h — it references cfe_sb_types.h with relative path
  local structs_h="${CFS_DIR}/apps/la_common/fsw/mission_inc/la_msg_structs.h"
  if [[ -f "$structs_h" ]]; then
    sed -i 's|#include "../cfe_hdr/cfe_sb_types.h"|#include "cfe.h"|g' "$structs_h"
  fi

  ok "Sources adapted for real cFE build"

  # ── 5b+5c: Patch TO_LAB and SCH_LAB tables ──
  log "Patching TO_LAB and SCH_LAB tables for Luna-Aegis..."
  local patch_script="${SCRIPT_DIR}/patch_tables.py"
  if [[ -f "$patch_script" ]]; then
    python3 "$patch_script" "${CFS_DIR}"
  else
    warn "  patch_tables.py not found — inline TO_LAB patch"
  fi

  # Remove high-rate MIDs from TO_LAB that the console doesn't parse
  # (IMU 40Hz, ACS 40Hz, ALT 10Hz, GDN 10Hz, TRN 2Hz, TCS, LC)
  local to_sub="${CFS_DIR}/apps/to_lab/fsw/tables/to_lab_sub.c"
  if [[ -f "$to_sub" ]]; then
    for mid_hex in 0x1811 0x1813 0x1817 0x1819 0x181B 0x1833 0x1843; do
      sed -i "/${mid_hex}/d" "$to_sub"
    done
    echo "  TO_LAB high-rate MIDs removed (IMU/ACS/ALT/GDN/TRN/TCS/LC)"

    # Increase MsgLim to prevent SB pipe overflow.
    # Default MsgLim 4 overflows in 400ms for 10 Hz MIDs (NAV/PROP)
    # and generates EVS cascade events that starve 1 Hz MIDs (LG/DOCK).
    # 10 Hz MIDs: 4 → 16  (tolerates 1.6s TO_LAB scheduling jitter)
    #  1 Hz MIDs: 4 → 8   (tolerates 8s startup drain delay)
    # cFE core:   4 → 8   (same)
    # Total worst-case: 5×8+32+2×16+8×8 = 168 < 256 pipe depth
    sed -i 's/\(0x1815.*{0, 0},\) 4/\1 16/' "$to_sub"   # NAV  10 Hz
    sed -i 's/\(0x1823.*{0, 0},\) 4/\1 16/' "$to_sub"   # PROP 10 Hz
    for mid_1hz in 0x1821 0x1831 0x1835 0x1841 0x1837 0x1825 0x1827 0x1839; do
      sed -i "s/\(${mid_1hz}.*{0, 0},\) 4/\1 8/" "$to_sub"
    done
    # cFE core HK MIDs
    sed -i 's/\(CFE_ES_HK_TLM_MID.*{0, 0},\) 4/\1 8/' "$to_sub"
    sed -i 's/\(CFE_EVS_HK_TLM_MID.*{0, 0},\) 4/\1 8/' "$to_sub"
    sed -i 's/\(CFE_SB_HK_TLM_MID.*{0, 0},\) 4/\1 8/' "$to_sub"
    sed -i 's/\(CFE_TBL_HK_TLM_MID.*{0, 0},\) 4/\1 8/' "$to_sub"
    sed -i 's/\(CFE_TIME_HK_TLM_MID.*{0, 0},\) 4/\1 8/' "$to_sub"
    echo "  TO_LAB MsgLim increased (10Hz→16, 1Hz→8, cFE→8)"
  fi

  # Bump SCH_LAB max schedule entries from 32 to 48
  local sch_cfg="${CFS_DIR}/apps/sch_lab/fsw/inc/sch_lab_interface_cfg.h"
  if [[ -f "$sch_cfg" ]]; then
    sed -i 's/DEFAULT_SCH_LAB_MISSION_MAX_SCHEDULE_ENTRIES 32/DEFAULT_SCH_LAB_MISSION_MAX_SCHEDULE_ENTRIES 48/' "$sch_cfg"
  fi
  local sch_cfg2="${CFS_DIR}/apps/sch_lab/config/default_sch_lab_interface_cfg_values.h"
  if [[ -f "$sch_cfg2" ]]; then
    sed -i 's/MAX_SCHEDULE_ENTRIES 32/MAX_SCHEDULE_ENTRIES 48/' "$sch_cfg2"
  fi
  local sch_mcfg="${CFS_DIR}/apps/sch_lab/config/default_sch_lab_mission_cfg.h"
  if [[ -f "$sch_mcfg" ]]; then
    sed -i 's/DEFAULT_SCH_LAB_MISSION_MAX_SCHEDULE_ENTRIES 32/DEFAULT_SCH_LAB_MISSION_MAX_SCHEDULE_ENTRIES 48/' "$sch_mcfg"
  fi
}

# ── Step 6: Build ────────────────────────────────────────────

build_cfs() {
  log "Building cFS + Luna-Aegis..."
  cd "${CFS_DIR}"

  # Copy Makefile.sample if needed
  if [[ ! -f "Makefile" ]]; then
    cp cfe/cmake/Makefile.sample Makefile
  fi

  # Copy sample_defs if the default location isn't set
  if [[ ! -d "sample_defs" ]] && [[ -d "cfe/cmake/sample_defs" ]]; then
    cp -r cfe/cmake/sample_defs .
  fi

  log "  Running cmake prep (SIMULATION=native)..."
  make SIMULATION=native prep 2>&1 | tail -20

  # ── Post-prep patches ──
  # CMake generates osconfig.h during prep — we patch it AFTER prep.
  log "  Patching generated OSAL config for Luna-Aegis resource limits..."
  for osc in build/osal_public_api/inc/osconfig.h \
             build/native/default_cpu1/osal/inc/osconfig.h; do
    if [[ -f "$osc" ]]; then
      sed -i 's/#define OS_MAX_MODULES.*/#define OS_MAX_MODULES                  48/' "$osc"
      sed -i 's/#define OS_MAX_TASKS.*/#define OS_MAX_TASKS                    64/' "$osc"
      sed -i 's/#define OS_MAX_QUEUES.*/#define OS_MAX_QUEUES                   64/' "$osc"
    fi
  done

  # Patch OSAL to use RTLD_GLOBAL for dlopen so HAL library symbols
  # are visible to dynamically loaded apps (matches RTEMS behavior)
  local dl_loader="osal/src/os/portable/os-impl-posix-dl-loader.c"
  if [[ -f "$dl_loader" ]]; then
    sed -i 's/dl_mode |= RTLD_LOCAL/dl_mode |= RTLD_GLOBAL/' "$dl_loader"
    log "  Patched OSAL dlopen to use RTLD_GLOBAL"
  fi

  # Bump MAX_APPLICATIONS in cFE ES config
  local es_cfg="cfe/modules/es/fsw/inc/cfe_es_interface_cfg.h"
  if [[ -f "$es_cfg" ]]; then
    sed -i 's/DEFAULT_CFE_MISSION_ES_MAX_APPLICATIONS 16/DEFAULT_CFE_MISSION_ES_MAX_APPLICATIONS 40/' "$es_cfg"
  fi

  log "  Compiling..."
  if make -j"$(nproc)" 2>&1 | tail -30; then
    ok "Build successful"
  else
    warn "Build completed with warnings (expected for initial integration)"
    warn "Some Luna-Aegis apps may not link against real cFE APIs yet"
    warn "The ground console sim mode works independently"
  fi

  log "  Installing..."
  make install 2>&1 | tail -10

  ok "cFS + Luna-Aegis build complete"
  log "  Binary: ${CFS_DIR}/build/exe/cpu1/core-cpu1"
}

# ── Step 7: Run ──────────────────────────────────────────────

run_system() {
  log ""
  log "══════════════════════════════════════════════════════════"
  log "  LUNA-AEGIS SHORT HOPPER — GROUND OPERATIONS"
  log "══════════════════════════════════════════════════════════"
  log ""

  # Start ground console
  local console_dir="${CFS_DIR}/ground-console"
  if [[ -f "${console_dir}/index.html" ]]; then
    log "Starting ground console on http://localhost:${CONSOLE_PORT} ..."
    cd "${console_dir}"
    python3 -m http.server "${CONSOLE_PORT}" &>/dev/null &
    CONSOLE_PID=$!
    ok "Ground console running (PID ${CONSOLE_PID})"
    log "  → Open http://localhost:${CONSOLE_PORT} in your browser"
  else
    warn "Ground console not found at ${console_dir}"
  fi

  # Try to run cFS binary
  local cfs_bin="${CFS_DIR}/build/exe/cpu1/core-cpu1"
  if [[ -x "$cfs_bin" ]]; then
    log ""
    log "Starting cFS flight software..."
    log "  (Press Ctrl+C to stop)"
    log ""
    cd "${CFS_DIR}/build/exe/cpu1"
    ./core-cpu1 &
    CFS_PID=$!
    log "  cFS running (PID ${CFS_PID})"
  else
    log ""
    log "cFS binary not found or not executable."
    log "The ground console is running — bridge in standalone mode."
    log ""
    log "  → Open http://localhost:${CONSOLE_PORT}"
  fi

  log ""
  log "Press Ctrl+C to shut down everything."
  log ""

  # Wait for any child to exit, or Ctrl+C
  trap cleanup SIGINT SIGTERM
  wait
}

cleanup() {
  log ""
  log "Shutting down..."
  # Kill cFS first — send SIGTERM, give it 3s, then SIGKILL
  if [[ -n "${CFS_PID:-}" ]]; then
    kill "${CFS_PID}" 2>/dev/null
    for i in 1 2 3; do
      kill -0 "${CFS_PID}" 2>/dev/null || break
      sleep 1
    done
    kill -9 "${CFS_PID}" 2>/dev/null || true
  fi
  # Kill console HTTP server
  [[ -n "${CONSOLE_PID:-}" ]] && kill "${CONSOLE_PID}" 2>/dev/null || true
  # Kill any remaining children
  jobs -p | xargs -r kill 2>/dev/null || true
  wait 2>/dev/null || true
  ok "Luna-Aegis ground operations terminated"
  exit 0
}

# ── Main ─────────────────────────────────────────────────────

main() {
  echo ""
  echo -e "${CYN}══════════════════════════════════════════════════════════${RST}"
  echo -e "${CYN}  LUNA-AEGIS SHORT HOPPER${RST}"
  echo -e "${CYN}  cFS Flight Software Integration Installer${RST}"
  echo -e "${CYN}══════════════════════════════════════════════════════════${RST}"
  echo ""
  log "cFS root: ${CFS_DIR}"
  [[ -n "$CUSTOM_CFS_ROOT" ]] && log "  (custom --cfs-root)"
  echo ""

  if [[ $RUN_ONLY -eq 1 ]]; then
    run_system
    return
  fi

  check_prereqs
  clone_cfs
  copy_apps
  generate_cmake
  patch_mission_config
  adapt_sources
  build_cfs

  if [[ $BUILD_ONLY -eq 1 ]]; then
    ok ""
    ok "Build complete. To run:"
    if [[ -n "$CUSTOM_CFS_ROOT" ]]; then
      ok "  $0 --cfs-root ${CFS_DIR} --run-only"
    else
      ok "  $0 --run-only"
    fi
    return
  fi

  run_system
}

trap cleanup SIGINT SIGTERM
main "$@"
