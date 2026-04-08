> **⚠️ v1 — Archived.** This repository contains the original open-source release of the Short Hopper cFS stack. Active development has moved to a private v2 repository.

# Luna-Aegis Short Hopper — Flight Software

**Reusable Lunar Surface–Orbit Transfer Vehicle — cFS-Based Flight Software**

22 mission applications running on NASA's [core Flight System (cFS)](https://github.com/nasa/cFS) Draco framework. Real CCSDS telemetry over the Software Bus, live command uplink, and a browser-based ground console accessible over LAN.

**Design Authority:** Aegis Station Infrastructure LLC  
**Classification:** ITAR/EAR-Free Baseline  
**License:** Apache 2.0

---

## Quick Start

```bash
# Clone
git clone https://github.com/aaroncsmith/luna-aegis-fsw.git
cd luna-aegis-fsw

# Build (clones cFS, integrates apps, patches configs, compiles — ~3 min)
./install.sh

# Terminal 1: cFS flight software
cd cFS/build/exe/cpu1 && ./core-cpu1

# Terminal 2: Ground console + WebSocket bridge
cd ground && ./la_launch.sh

# Open http://localhost:8080/index.html
```

## What It Does

The Short Hopper is a single-stage VTOL lunar shuttle (~8,000 kg, LOX/LH₂, Isp 440 s) designed for crew and cargo transfer between low lunar orbit and south pole surface sites. The flight software manages autonomous mission profiles through a 9-state flight phase FSM:

```
PREFLIGHT → POWERED_ASC → COAST → POWERED_DES → HOVER → TERMINAL → LANDED
                                                                     ↓
                              ABORT ← (reachable from any powered phase)
                                ↓
                              SAFED
```

The Mission Manager (MM) orchestrates all subsystems — commanding engine start/stop, landing gear deploy, docking operations, and monitoring abort conditions — while 21 other apps handle navigation, guidance, attitude control, power, life support, comms, thermal management, health monitoring, and data storage.

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                    cFS Draco (cFE 7.0)              │
├──────────┬──────────┬──────────┬───────────┬────────┤
│  GN&C    │ Mission  │ Vehicle  │  Health   │ Infra  │
│ IMU_MGR  │ MM       │ EPS      │ HS        │ HK     │
│ ALT_MGR  │ PROP     │ TCS      │ LC        │ DS     │
│ NAV      │ LG       │ LSS      │ SC        │ FM     │
│ GDN      │ DOCK     │ COMM     │           │        │
│ ACS      │          │ LUNET    │           │        │
│ TRN      │          │          │           │        │
├──────────┴──────────┴──────────┴───────────┴────────┤
│              HAL — Hardware Abstraction Layer        │
│              (Simulation drivers in SIM_MODE)        │
└─────────────────────────────────────────────────────┘
          ↕ cFE Software Bus (CCSDS v1 packets)
┌─────────────────────────────────────────────────────┐
│  CI_LAB ← UDP:1234 ← la_bridge.py ← WS:8765       │
│  TO_LAB → UDP:2234 → la_bridge.py → WS:8765        │
│                     → Ground Console (any device)   │
└─────────────────────────────────────────────────────┘
```

### 22 Mission Applications

| Tier | App | Rate | Function |
|------|-----|------|----------|
| **GN&C** | IMU_MGR | 40 Hz | FOG/RLG + MEMS cross-check, source selection |
| | ALT_MGR | 10 Hz | Lidar/radar weighted fusion, terrain slope |
| | NAV | 40 Hz | 12-state EKF — position, velocity, attitude, bias |
| | GDN | 10 Hz | Gravity-turn ascent, powered descent PD, hover hold |
| | ACS | 40 Hz | PD attitude controller, gimbal/RCS mixing, deadband |
| | TRN | 2 Hz | Terrain-relative position fix, hazard detection |
| **Mission** | MM | 1 Hz | 9-state flight phase FSM, auto-abort, Δv tracking |
| | PROP | 10 Hz | Engine start sequence, throttle, propellant model |
| | LG | 1 Hz | Deploy/retract, per-leg load monitoring, touchdown detect |
| | DOCK | 1 Hz | 5-state docking FSM, seal monitoring, pressure equalization |
| **Vehicle** | EPS | 1 Hz | SOC tracking, bus voltage, 3-level load shedding |
| | TCS | 0.1 Hz | SSSRA radiator, 8-zone thermostat, Stefan-Boltzmann model |
| | LSS | 1 Hz | Cabin atmosphere (O₂/CO₂), consumables tracking |
| | COMM | 1 Hz | HGA pointing, SNR-based link state machine |
| | LUNET | 1 Hz | Beacon ranging, refuel port status |
| **Health** | HS | 1 Hz | Per-app watchdog, CPU/memory tracking |
| | LC | 1 Hz | 4 watchpoints with actionpoint triggers |
| | SC | 1 Hz | Time-tagged command sequences, abort safing |
| **Infra** | HK | 1 Hz | Housekeeping aggregation from 18 apps |
| | DS | 1 Hz | Data storage logging |
| | FM | 1 Hz | File manager with command dispatch |

### HAL — Hardware Abstraction Layer

Loaded as a cFS shared library (`CFE_LIB`). Abstracts 8 hardware controllers with simulation drivers for native Linux execution:

Engine Gimbal (LVDS), RCS Valve Drivers, Battery Management Unit (SPI), Cryo Tank Pressure Controller, Engine Controller (CAN), Landing Gear Actuators (CAN), Docking Collar Actuators (CAN), Heater Controllers, Antenna Gimbal (LVDS), IMU Sensors, Altimeters.

## Ground System

### Web Console — `ground/console/index.html`

Real-time mission control connected to cFS via the WebSocket bridge. Blueprint aesthetic (IBM Plex Mono, navy/cyan/amber palette).

- **8 telemetry panels** — Mission Manager, GN&C State, Propulsion, Electrical Power, Life Support, Comms/LUNET, Health & Safety, Landing Gear/Dock
- **Live command buttons** — START MISSION, ABORT, MANUAL/AUTO mode, engine ARM/START/SHUTDOWN, gear DEPLOY/RETRACT, DOCK MATE/DEMATE
- **Connection status** — WebSocket indicator, telemetry packet counter, staleness detection (panels grey out after 5 s without updates)
- **Event log** — command acknowledgements, bridge status, errors
- **LAN accessible** — open from any device on the network

### WebSocket Bridge — `ground/la_bridge.py`

Bridges cFS UDP telemetry to browser-friendly JSON over WebSocket:

```
cFS TO_LAB → UDP:2234 → la_bridge.py → WS:8765 → Browser Console
Browser Console → WS:8765 → la_bridge.py → UDP:1234 → cFS CI_LAB
```

Parses 9 telemetry packet types (MM, NAV, PROP, EPS, LSS, HS, COMM, LG, DOCK) from CCSDS binary into JSON. Routes console commands as properly formatted CCSDS command packets to CI_LAB. Sends TO_LAB enable on each client connect.

### LAN Launcher — `ground/la_launch.sh`

Starts bridge + HTTP server bound to `0.0.0.0`, auto-detects LAN IP, prints access URL.

```bash
./la_launch.sh                     # Default ports (WS:8765, HTTP:8080)
./la_launch.sh --http-port 9090    # Custom HTTP port
```

## Vehicle Specifications

| Parameter | Value |
|-----------|-------|
| Gross Wet Mass | ~8,000 kg |
| Dry Mass | ~5,250 kg |
| Propellant | ~2,750 kg LOX/LH₂ (ISRU-compatible) |
| Engine | Single gimbaled vacuum, ~30 kN, Isp ~440 s |
| Design Δv | 1,800 m/s (10% margin) |
| Range | 1,500–2,000 km surface-to-surface |
| Crew | 4 standard / 6 max |
| Cargo | Up to 1,000 kg |
| Turnaround | 24–48 hours with LUNET node support |
| Landing Precision | ±3 m (nominal) |
| Reusability | 5–10 sorties; indefinite with proactive maintenance |

## Repository Structure

```
luna-aegis-fsw/
├── install.sh                 # One-command cFS build + integration
├── Makefile                   # Standalone compile check (SIM_MODE)
├── fsw/
│   ├── apps/                  # 22 mission apps (C source)
│   │   ├── mm/src/            #   Mission Manager — flight phase FSM
│   │   ├── nav/src/           #   Navigation — 12-state EKF
│   │   ├── prop/src/          #   Propulsion — engine sequencing
│   │   ├── acs/src/           #   Attitude Control — gimbal + RCS
│   │   └── ...                #   18 more apps
│   ├── hal/                   # Hardware abstraction layer
│   │   ├── inc/la_hal.h       #   HAL API (8 HW controllers)
│   │   └── sim/la_hal_sim.c   #   Simulation drivers
│   ├── cfe_hdr/               # cFE API stubs (standalone build only)
│   └── msg_defs/
│       ├── la_msgids.h        # Master MID registry
│       └── la_msg_structs.h   # All packet structures
├── ground/
│   ├── console/index.html     # Web-based ground console
│   ├── la_bridge.py           # WebSocket-to-UDP bridge
│   └── la_launch.sh           # LAN launcher (bridge + HTTP server)
├── docs/                      # Architecture, ICD, message dictionary
└── tools/                     # Command generation, telemetry decode
```

## Building

### Prerequisites

- Linux (Ubuntu 22.04+ / Pop!_OS / Debian)
- GCC 11+, CMake 3.20+, Git
- Python 3.10+ with `websockets` (`pip install websockets`)

### Full cFS Integration Build

```bash
./install.sh
```

This clones NASA cFS with all submodules, copies 22 apps + HAL into the cFS tree, generates CMakeLists.txt for each app, patches platform configuration (OSAL limits, TO_LAB subscription table, startup script), adapts source includes, builds, and launches.

Flags:
- `--cfs-root PATH` — use existing cFS tree instead of cloning
- `--build-only` — build without launching
- `--run-only` — launch existing build
- `--port PORT` — console HTTP port (default 8080)

### Standalone Compile Check

```bash
make check    # Compiles all 22 apps in SIM_MODE — no cFS dependency
```

## Running

```bash
# Terminal 1: cFS flight software
cd cFS/build/exe/cpu1 && ./core-cpu1

# Terminal 2: Ground console + bridge
cd ground && ./la_launch.sh

# Open in browser
http://localhost:8080/index.html
```

All telemetry panels show `---` until cFS is running and producing telemetry. Command buttons are live — they send real CCSDS packets through the bridge to CI_LAB.

To stop cFS cleanly:
```bash
killall -9 core-cpu1 2>/dev/null
```

## cFS Integration Details

The install script patches several cFS configuration files to support 22+ Luna-Aegis apps:

| File | Change |
|------|--------|
| `default_osconfig.h` | `OS_MAX_TASKS` → 64, `OS_MAX_MODULES` → 48, `OS_MAX_QUEUES` → 64 |
| `default_cfe_platform_cfg.h` | `ES_MAX_APPLICATIONS` → 40, `SB_MAX_PIPE_DEPTH` → 64 |
| `to_lab_sub_table.h` | Added 18 Luna-Aegis telemetry MIDs |
| `cfe_es_startup.scr` | HAL as `CFE_LIB`, 22 apps as `CFE_APP`, clean EOF marker |
| `targets.cmake` | All `la_*` apps added to `TGT1_APPLIST` |

All Luna-Aegis apps link against `-lm` for math functions used in GN&C and thermal calculations.

## Message Architecture

Telemetry MIDs: `0x1800–0x1843` (published by apps → TO_LAB → UDP → bridge → console)  
Command MIDs: `0x1900–0x1926` (console → bridge → UDP → CI_LAB → apps)  
Wakeup MIDs: `0x1A00–0x1A13` (SCH_LAB scheduler ticks)

All packets use `CFE_MSG_TelemetryHeader_t` / `CFE_MSG_CommandHeader_t` with Draco-era opaque `CFE_SB_MsgId_t` types. MID constants are wrapped with `CFE_SB_ValueToMsgId()` / `CFE_SB_MsgIdToValue()` at all usage points for real cFE compatibility.

## Contact

Aaron Smith — engage@aegisstation.com  
Aegis Station Infrastructure LLC
