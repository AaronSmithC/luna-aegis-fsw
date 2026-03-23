# Luna-Aegis Short Hopper — Flight Software

**Reusable Lunar Surface–Orbit Transfer Vehicle — cFS-Based Flight Software**

22 mission applications running on NASA's [core Flight System (cFS)](https://github.com/nasa/cFS) Draco framework. Fully autonomous sortie missions with real CCSDS telemetry, inter-app commanding, and a browser-based ground console accessible over LAN.

**Design Authority:** Aegis Station Infrastructure LLC  
**Classification:** ITAR/EAR-Free Baseline  
**License:** Apache 2.0

---

## Quick Start

```bash
# Build (clones cFS, integrates apps, compiles — ~3 min)
./install.sh

# Terminal 2: Ground console + bridge
cd ground && ./la_launch.sh --http-port 9090

# Open http://localhost:9090 — pick a destination, press LAUNCH
```

## What It Does

The Short Hopper is a single-stage VTOL lunar shuttle (~8,000 kg, LOX/LH₂, Isp 440s) designed for crew/cargo transfer between low lunar orbit and south pole surface sites. The flight software runs autonomous mission profiles:

```
PREFLIGHT → engine ARM+START → POWERED_ASC (15s burn) → COAST (20s) →
re-ignition → POWERED_DES (15s burn) → HOVER (8s, gear deploy) →
TERMINAL (6s) → LANDED → engine SHUTDOWN
```

The Mission Manager (MM) orchestrates all subsystems — commanding propulsion start/stop, landing gear deploy/retract, and monitoring abort conditions — while 21 other apps handle GN&C, power, life support, comms, thermal, health monitoring, and data management.

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                    cFS Draco (cFE 7.0)              │
├──────────┬──────────┬──────────┬───────────┬────────┤
│  Tier 1  │  Tier 2  │  Tier 3  │  Tier 4   │Tier 5 │
│ SCH HK   │ IMU NAV  │ MM PROP  │ EPS TCS   │ HS LC │
│ CI TO    │ ALT GDN  │ LG DOCK  │ LSS COMM  │ SC    │
│ DS FM    │ ACS TRN  │          │ LUNET     │       │
├──────────┴──────────┴──────────┴───────────┴────────┤
│              HAL (Hardware Abstraction)              │
│              Simulation Drivers (SIM_MODE)           │
└─────────────────────────────────────────────────────┘
          ↕ cFE Software Bus (CCSDS packets)
┌─────────────────────────────────────────────────────┐
│  TO_LAB → UDP:2234 → la_bridge.py → WS:8765        │
│                    → Web Console (any LAN device)   │
│                    → Terminal Commander (backup)     │
└─────────────────────────────────────────────────────┘
```

### 22 Mission Applications

| Tier | Apps | Function |
|------|------|----------|
| **GN&C** | IMU_MGR, ALT_MGR, NAV, GDN, ACS, TRN | Inertial/altimetry fusion, 12-state EKF, guidance, attitude control, terrain-relative nav |
| **Mission** | MM, PROP, LG, DOCK | Flight phase FSM, engine sequencing with propellant model, landing gear, docking collar |
| **Vehicle** | EPS, TCS, LSS, COMM, LUNET | Power/battery, thermal (SSSRA), life support, comms link management, beacon network |
| **Health** | HS, LC, SC | Watchdog monitoring, limit checking, stored command sequences |
| **Infra** | HK, DS, FM | Housekeeping aggregation, data storage, file management |

## Ground System

### Web Console (`ground/console/index.html`)

Browser-based mission control with destination selector and real-time telemetry:

- **Destination picker** — 10 named south pole sites
- **LAUNCH / ABORT** — single-button mission control
- **8 telemetry panels** — propulsion, navigation, power, life support, comms, health, gear/dock
- **Event log** — all phase transitions, engine events, command acknowledgements
- Accessible from any device on the LAN

### Terminal Commander (`ground/la_commander.py`)

Engineering backup console with extended operations:

```
hopper> hop 500              # Execute a single hop
hopper> campaign 5           # 5-hop south pole survey
hopper> survey Shackleton    # Named site survey with docking
hopper> status               # Full telemetry snapshot
hopper> fuel                 # Propellant budget
```

### WebSocket Bridge (`ground/la_bridge.py`)

Bridges cFS TO_LAB/CI_LAB (UDP/CCSDS) to WebSocket (JSON) for browser and terminal clients.

## Vehicle Specifications

| Parameter | Value |
|-----------|-------|
| Gross Wet Mass | ~8,000 kg |
| Dry Mass | ~5,250 kg |
| Propellant | ~2,750 kg LOX/LH₂ (ISRU-compatible) |
| Engine | Single gimbaled vacuum, ~30 kN, Isp ~440s |
| Design Δv | 1,800 m/s (10% margin) |
| Range | 1,500–2,000 km surface-to-surface |
| Crew | 4 standard / 6 max |
| Cargo | Up to 1,000 kg |
| Turnaround | 24–48 hours |

## Repository Structure

```
luna-aegis-fsw/
├── install.sh                 # One-command cFS build + integration
├── Makefile                   # Standalone build (SIM_MODE)
├── fsw/
│   ├── apps/                  # 22 mission apps (C source)
│   │   ├── mm/src/            # Mission Manager FSM
│   │   ├── prop/src/          # Propulsion sequencing
│   │   ├── nav/src/           # Navigation EKF
│   │   └── ...
│   ├── hal/                   # Hardware abstraction layer
│   │   ├── inc/la_hal.h       # HAL API (8 HW controllers)
│   │   └── sim/la_hal_sim.c   # Simulation drivers
│   ├── cfe_hdr/               # cFE API stubs (standalone build)
│   └── msg_defs/              # Message IDs + packet structures
├── ground/
│   ├── console/index.html     # Web-based ground console
│   ├── la_bridge.py           # WebSocket-to-UDP bridge
│   ├── la_commander.py        # Terminal mission commander
│   └── la_launch.sh           # LAN launcher (bridge + HTTP)
└── docs/                      # Architecture documentation
```

## Building

### Prerequisites

- Linux (Ubuntu 22.04+ / Pop!_OS recommended)
- GCC 11+, CMake 3.20+, Git
- Python 3.10+ with `websockets` package

### Full cFS Integration Build

```bash
./install.sh
```

This clones NASA cFS, copies all 22 apps, patches configuration (OSAL limits, TO_LAB subscriptions, SCH_LAB wakeups), builds, and optionally launches.

### Standalone Compile Check

```bash
make check    # Compiles all apps in SIM_MODE (no cFS dependency)
```

## Running

```bash
# Terminal 1: cFS flight software
cd cFS/build/exe/cpu1 && ./core-cpu1

# Terminal 2: Ground console + bridge
cd ground && ./la_launch.sh --http-port 9090

# Terminal 3 (optional): Terminal commander
cd ground && python3 la_commander.py
```

**Important:** Always kill old cFS instances before starting new ones:
```bash
killall -9 core-cpu1 2>/dev/null
```

## Contact

Aaron Smith — engage@aegisstation.com
