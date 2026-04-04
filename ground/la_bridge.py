#!/usr/bin/env python3
"""
Luna-Aegis Short Hopper — WebSocket-to-UDP Bridge
===================================================

Bridges the browser-based Ground Console to the cFS CI_LAB/TO_LAB interface.

  Browser (WebSocket :8765)  ←→  Bridge  ←→  cFS (UDP CI_LAB :1234 / TO_LAB :2234)

Commands flow:   Browser → WS → Bridge → UDP → cFS CI_LAB
Telemetry flows: cFS TO_LAB → UDP → Bridge → WS → Browser

On each client connect the bridge sends a TO_LAB enable command so cFS
starts streaming telemetry to this machine.

Usage:
  python3 la_bridge.py [--cfs-host 127.0.0.1] [--ci-port 1234] [--tlm-port 2234] [--ws-port 8765]

Requirements:
  pip install websockets
"""

import asyncio
import json
import socket
import struct
import argparse
import time
import sys

try:
    import websockets
except ImportError:
    print("Installing websockets...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "websockets", "--break-system-packages"])
    import websockets


# ──────────────────────────────────────────────────────────
# cFS Message IDs (must match la_msgids.h)
# ──────────────────────────────────────────────────────────

# Telemetry MIDs (from SB → TO_LAB → UDP → Bridge → Browser)
LA_IMU_DATA_TLM_MID       = 0x1811
LA_ALT_DATA_TLM_MID       = 0x1813
LA_NAV_STATE_TLM_MID      = 0x1815
LA_GDN_CMD_TLM_MID        = 0x1817
LA_ACS_ACTUATOR_CMD_MID   = 0x1819
LA_TRN_HAZARD_TLM_MID     = 0x181B
LA_MM_PHASE_TLM_MID       = 0x1821
LA_PROP_STATUS_TLM_MID    = 0x1823
LA_LG_STATUS_TLM_MID      = 0x1825
LA_DOCK_STATUS_TLM_MID    = 0x1827
LA_EPS_STATUS_TLM_MID     = 0x1831
LA_TCS_STATUS_TLM_MID     = 0x1833
LA_LSS_STATUS_TLM_MID     = 0x1835
LA_COMM_LINK_TLM_MID      = 0x1837
LA_LUNET_BEACON_TLM_MID   = 0x1839
LA_HS_ALERT_TLM_MID       = 0x1841
LA_LC_ACTION_TLM_MID      = 0x1843
LA_HK_COMBINED_TLM_MID    = 0x1803

# Command MIDs (Browser → Bridge → UDP → cFS CI_LAB)
LA_MM_CMD_MID              = 0x1920
LA_MM_ABORT_CMD_MID        = 0x1921
LA_PROP_CMD_MID            = 0x1922
LA_LG_CMD_MID              = 0x1924
LA_DOCK_CMD_MID            = 0x1926
LA_NAV_CMD_MID             = 0x1914
LA_LUNET_CMD_MID_CMD       = 0x1938

# TO_LAB
TO_LAB_CMD_MID             = 0x1880

# Command function codes
MM_CC = {"NOOP": 0, "RESET": 1, "START_MISSION": 2, "ABORT": 3, "MANUAL": 4, "AUTO": 5, "SET_DOCK": 6, "SET_SURFACE": 7}
PROP_CC = {"NOOP": 0, "RESET": 1, "ARM": 2, "DISARM": 3, "START": 4, "SHUTDOWN": 5}
LG_CC = {"NOOP": 0, "RESET": 1, "DEPLOY": 2, "RETRACT": 3}
DOCK_CC = {"NOOP": 0, "RESET": 1, "MATE": 2, "DEMATE": 3, "HARD": 4}
NAV_CC = {"NOOP": 0, "RESET": 1, "SET_DEST": 2}
LUNET_CC = {"NOOP": 0, "RESET": 1, "REFUEL_START": 2, "AUTO_REFUEL": 3}

# Enums
PHASES = {0: "PREFLIGHT", 1: "POWERED_ASC", 2: "COAST", 3: "POWERED_DES",
          4: "HOVER", 5: "TERMINAL", 6: "LANDED", 7: "ABORT", 8: "SAFED",
          9: "RENDEZVOUS", 10: "DOCKING", 11: "DOCKED", 12: "UNDOCKING"}
MISSION_TYPES = {0: "SURFACE", 1: "DOCKING"}
NAV_MODES = {0: "PROPAGATE", 1: "IMU+ALT", 2: "FULL_FUSION"}
ENGINE_STATES = {0: "OFF", 1: "CHILL", 2: "IGNITION", 3: "RUNNING", 4: "SHUTDOWN", 5: "FAULT"}
GEAR_STATES = {0: "STOWED", 1: "DEPLOYING", 2: "DEPLOYED", 3: "RETRACTING", 4: "FAULT"}
DOCK_STATES = {0: "RETRACTED", 1: "EXTENDING", 2: "SOFT_DOCK", 3: "HARD_DOCK", 4: "FAULT"}
LINK_STATES = {0: "NO_LINK", 1: "UHF", 2: "S-BAND", 3: "HGA"}


# ──────────────────────────────────────────────────────────
# CCSDS Packet Construction (Draco format)
# ──────────────────────────────────────────────────────────
_cmd_seq = 0

def build_ccsds_cmd(mid, fc, payload=b""):
    global _cmd_seq
    _cmd_seq = (_cmd_seq + 1) & 0x3FFF
    total = 8 + len(payload)
    length = total - 7
    hdr = struct.pack(">HHH", mid, 0xC000 | _cmd_seq, length)
    sec = struct.pack("BB", fc, 0)
    return hdr + sec + payload


def build_to_lab_enable(ip_str="127.0.0.1"):
    payload_str = f"16:{ip_str}"
    payload = payload_str.encode("ascii").ljust(16, b"\x00")
    return build_ccsds_cmd(TO_LAB_CMD_MID, 6, payload)


# ──────────────────────────────────────────────────────────
# CCSDS Telemetry Decoding
# Draco TLM: 16 bytes header (6 primary + 2 extended + 8 secondary)
# ──────────────────────────────────────────────────────────
TLM_HDR_LEN = 16

def decode_tlm_header(data):
    if len(data) < TLM_HDR_LEN:
        return None
    stream_id, sequence, length = struct.unpack(">HHH", data[0:6])
    mid = stream_id
    seq_count = sequence & 0x3FFF
    payload = data[TLM_HDR_LEN:]
    return mid, seq_count, payload


# ──────────────────────────────────────────────────────────
# Telemetry Parsers (match la_msg_structs.h)
# ──────────────────────────────────────────────────────────

def parse_mm_phase(payload):
    """LA_MM_PhasePkt_t (Rev C): B B B B B x  d d d I = 36 bytes
       CurrentPhase, PreviousPhase, AbortReason, MissionMode,
       MissionType, Padding, MET_s, DvRemaining, PropRemaining,
       PhaseTimer_s
    """
    if len(payload) < 36:
        return None
    try:
        phase, prev, abort_r, mode, mtype, _pad = struct.unpack_from("BBBBBB", payload, 0)
        met, dv_remain, prop_remain = struct.unpack_from("<ddd", payload, 8)
        phase_timer = struct.unpack_from("<I", payload, 32)[0]
        return {
            "type": "mm_phase",
            "phase": phase, "phaseName": PHASES.get(phase, str(phase)),
            "previousPhase": prev, "abortReason": abort_r,
            "missionMode": mode,
            "missionType": mtype, "missionTypeName": MISSION_TYPES.get(mtype, str(mtype)),
            "met": round(met, 1), "dvRemaining": round(dv_remain, 1),
            "propRemaining": round(prop_remain, 1),
            "phaseTimer": phase_timer,
        }
    except struct.error:
        return None


def parse_nav_state(payload):
    """LA_NAV_StatePkt_t: 3d + 3d + 4d + 3d + 3d + d + BBBx = ~140 bytes (Rev D)"""
    if len(payload) < 136:
        return None
    try:
        pos = struct.unpack_from("<3d", payload, 0)
        vel = struct.unpack_from("<3d", payload, 24)
        att = struct.unpack_from("<4d", payload, 48)  # quaternion
        ang = struct.unpack_from("<3d", payload, 80)
        pos_u, vel_u, att_u = struct.unpack_from("<3d", payload, 104)
        dist_to_dest, = struct.unpack_from("<d", payload, 128)
        nav_mode, nav_health, dest_id = struct.unpack_from("BBB", payload, 136)
        return {
            "type": "nav_state",
            "posX": round(pos[0], 2), "posY": round(pos[1], 2), "posZ": round(pos[2], 2),
            "velX": round(vel[0], 3), "velY": round(vel[1], 3), "velZ": round(vel[2], 3),
            "attQ0": round(att[0], 4), "attQ1": round(att[1], 4),
            "attQ2": round(att[2], 4), "attQ3": round(att[3], 4),
            "posUncert": round(pos_u, 2), "velUncert": round(vel_u, 3),
            "distToDest": round(dist_to_dest, 1),
            "destId": dest_id if dest_id != 0xFF else None,
            "navMode": nav_mode, "navModeName": NAV_MODES.get(nav_mode, str(nav_mode)),
            "navHealth": nav_health,
        }
    except struct.error:
        return None


def parse_prop_status(payload):
    """LA_PROP_StatusPkt_t: 7d + BB xx = ~60 bytes"""
    if len(payload) < 60:
        return None
    try:
        pc, ox_p, fuel_p, ox_m, fuel_m, mr, thrust_est = struct.unpack_from("<7d", payload, 0)
        eng_state, valve_states = struct.unpack_from("BB", payload, 56)
        return {
            "type": "prop_status",
            "chamberPress": round(pc, 1), "oxTankPress": round(ox_p, 1),
            "fuelTankPress": round(fuel_p, 1),
            "oxMass": round(ox_m, 1), "fuelMass": round(fuel_m, 1),
            "mixtureRatio": round(mr, 2), "thrustEst": round(thrust_est, 1),
            "engineState": eng_state, "engineStateName": ENGINE_STATES.get(eng_state, str(eng_state)),
            "valveStates": valve_states,
        }
    except struct.error:
        return None


def parse_eps_status(payload):
    """LA_EPS_StatusPkt_t: 5d + BB xx = ~44 bytes"""
    if len(payload) < 42:
        return None
    try:
        bus_v, bus_a, soc, batt_t, solar = struct.unpack_from("<5d", payload, 0)
        shed, health = struct.unpack_from("BB", payload, 40)
        return {
            "type": "eps_status",
            "busVoltage": round(bus_v, 1), "busCurrent": round(bus_a, 1),
            "battSOC": round(soc, 1), "battTemp": round(batt_t, 1),
            "solarInput": round(solar, 1),
            "loadShedLevel": shed, "battHealth": health,
        }
    except struct.error:
        return None


def parse_lss_status(payload):
    """LA_LSS_StatusPkt_t: 7d + BB xx = ~60 bytes"""
    if len(payload) < 58:
        return None
    try:
        cp, o2, co2, hum, ct, o2r, scrub = struct.unpack_from("<7d", payload, 0)
        crew, mode = struct.unpack_from("BB", payload, 56)
        return {
            "type": "lss_status",
            "cabinPress": round(cp, 1), "ppO2": round(o2, 2),
            "ppCO2": round(co2, 3), "humidity": round(hum, 1),
            "cabinTemp": round(ct, 1),
            "o2Remaining": round(o2r, 1), "scrubberPct": round(scrub, 1),
            "crewCount": crew, "lssMode": mode,
        }
    except struct.error:
        return None


def parse_hs_alert(payload):
    """LA_HS_AlertPkt_t: II HH BB xx = ~12 bytes"""
    if len(payload) < 12:
        return None
    try:
        cpu, mem = struct.unpack_from("<II", payload, 0)
        wd_mask, crit = struct.unpack_from("<HH", payload, 8)
        redund, reboot = struct.unpack_from("BB", payload, 12)
        return {
            "type": "hs_alert",
            "cpuLoad": round(cpu / 100.0, 1), "memFree": mem,
            "watchdogMask": f"0x{wd_mask:04X}",
            "critEvents": crit,
            "redundState": redund, "rebootCount": reboot,
        }
    except struct.error:
        return None


def parse_comm_link(payload):
    """LA_COMM_LinkPkt_t: 4d + BB xx"""
    if len(payload) < 34:
        return None
    try:
        up_snr, down_snr, hga_az, hga_el = struct.unpack_from("<4d", payload, 0)
        link, rate = struct.unpack_from("BB", payload, 32)
        return {
            "type": "comm_link",
            "uplinkSNR": round(up_snr, 1), "downlinkSNR": round(down_snr, 1),
            "hgaAz": round(hga_az, 1), "hgaEl": round(hga_el, 1),
            "linkState": link, "linkStateName": LINK_STATES.get(link, str(link)),
            "dataRate": rate,
        }
    except struct.error:
        return None


def parse_lg_status(payload):
    if len(payload) < 40:
        return None
    try:
        deploy = struct.unpack_from("B", payload, 0)[0]
        legs = struct.unpack_from("4B", payload, 1)
        td = struct.unpack_from("B", payload, 5)[0]
        loads = struct.unpack_from("<4d", payload, 8)
        return {
            "type": "lg_status",
            "gearState": deploy, "gearStateName": GEAR_STATES.get(deploy, str(deploy)),
            "legStatus": list(legs), "touchdown": td,
            "legLoads": [round(l, 0) for l in loads],
        }
    except struct.error:
        return None


def parse_dock_status(payload):
    if len(payload) < 20:
        return None
    try:
        collar, seal_ok, latch, target = struct.unpack_from("BBBB", payload, 0)
        tunnel_p, delta_p = struct.unpack_from("<2d", payload, 4)
        return {
            "type": "dock_status",
            "collarState": collar, "collarStateName": DOCK_STATES.get(collar, str(collar)),
            "sealOK": seal_ok, "latchState": latch, "mateTarget": target,
            "tunnelPress": round(tunnel_p, 2), "deltaP": round(delta_p, 3),
        }
    except struct.error:
        return None


REFUEL_STATES = {0: "IDLE", 1: "HANDSHAKE", 2: "CART_MATE", 3: "PROP_XFER", 4: "RECHARGE", 5: "COMPLETE"}


def parse_lunet_beacon(payload):
    """LA_LUNET_BeaconPkt_t: 3d + d + d + BBBx"""
    if len(payload) < 42:
        return None
    try:
        bx, by, bz = struct.unpack_from("<3d", payload, 0)
        rng, bearing = struct.unpack_from("<2d", payload, 24)
        beacon_id, sig_q, refuel = struct.unpack_from("BBB", payload, 40)
        return {
            "type": "lunet_beacon",
            "beaconId": beacon_id,
            "range": round(rng, 1),
            "signalQuality": sig_q,
            "refuelState": refuel,
            "refuelStateName": REFUEL_STATES.get(refuel, str(refuel)),
        }
    except struct.error:
        return None


# Parser registry
TLM_PARSERS = {
    LA_MM_PHASE_TLM_MID:     parse_mm_phase,
    LA_NAV_STATE_TLM_MID:    parse_nav_state,
    LA_PROP_STATUS_TLM_MID:  parse_prop_status,
    LA_EPS_STATUS_TLM_MID:   parse_eps_status,
    LA_LSS_STATUS_TLM_MID:   parse_lss_status,
    LA_HS_ALERT_TLM_MID:     parse_hs_alert,
    LA_COMM_LINK_TLM_MID:    parse_comm_link,
    LA_LG_STATUS_TLM_MID:    parse_lg_status,
    LA_DOCK_STATUS_TLM_MID:  parse_dock_status,
    LA_LUNET_BEACON_TLM_MID: parse_lunet_beacon,
}


# ──────────────────────────────────────────────────────────
# WebSocket-UDP Bridge
# ──────────────────────────────────────────────────────────

class LABridge:
    def __init__(self, cfs_host, ci_port, tlm_port, ws_port):
        self.cfs_host = cfs_host
        self.ci_port = ci_port
        self.tlm_port = tlm_port
        self.ws_port = ws_port
        self.clients = set()
        self.running = True
        self.tlm_count = 0
        self.unknown_mids = set()
        self.nav_decimator = 0   # Downsample NAV from 40 Hz to 2 Hz
        self.prop_decimator = 0  # Downsample PROP from 10 Hz to 2 Hz

        # Latest-value table: keyed by telemetry type string.
        # tlm_recv updates the value; tlm_broadcast sweeps dirty entries.
        # Guarantees every MID type gets delivered — no FIFO priority inversion.
        self._latest = {}      # type_str -> JSON string
        self._dirty = set()    # set of type_str updated since last broadcast
        self._lock = asyncio.Lock()

        # Diagnostic: track last-seen time for LG/DOCK to detect gaps
        self._diag_lg_last = 0.0
        self._diag_dock_last = 0.0

        # UDP sockets
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tlm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tlm_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tlm_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)  # 1 MB
        self.tlm_sock.bind(("0.0.0.0", self.tlm_port))
        self.tlm_sock.setblocking(False)

    async def start(self):
        print(f"  CMD  → cFS CI_LAB at {self.cfs_host}:{self.ci_port}")
        print(f"  TLM  ← listening on UDP :{self.tlm_port}")
        print(f"  WS   ← serving on ws://0.0.0.0:{self.ws_port}")
        print()

        async with websockets.serve(self.ws_handler, "0.0.0.0", self.ws_port):
            await asyncio.gather(self.tlm_recv(), self.tlm_broadcast())

    async def ws_handler(self, websocket, path=None):
        self.clients.add(websocket)
        addr = websocket.remote_address
        print(f"  ● Client connected: {addr[0]}:{addr[1]}")

        # Send TO_LAB enable so cFS starts streaming TLM to us
        enable_pkt = build_to_lab_enable("127.0.0.1")
        self.cmd_sock.sendto(enable_pkt, (self.cfs_host, self.ci_port))
        print(f"  → TO_LAB enable sent")

        # Send welcome
        welcome = {
            "type": "bridge_status",
            "status": "connected",
            "cfsHost": self.cfs_host,
            "ciPort": self.ci_port,
            "tlmPort": self.tlm_port,
            "tlmCount": self.tlm_count,
        }
        await websocket.send(json.dumps(welcome))

        try:
            async for message in websocket:
                await self.handle_cmd(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            print(f"  ○ Client disconnected: {addr[0]}:{addr[1]}")

    async def handle_cmd(self, websocket, message):
        try:
            msg = json.loads(message)
            cmd_type = msg.get("cmd", "").upper()
            params = msg.get("params", {})
            label = cmd_type
            pkt = None

            # Mission Manager commands
            if cmd_type in MM_CC:
                pkt = build_ccsds_cmd(LA_MM_CMD_MID, MM_CC[cmd_type])
            elif cmd_type == "MM_ABORT":
                pkt = build_ccsds_cmd(LA_MM_ABORT_CMD_MID, 0)

            # Propulsion commands
            elif cmd_type in ("PROP_" + k for k in PROP_CC):
                cc_key = cmd_type.replace("PROP_", "")
                pkt = build_ccsds_cmd(LA_PROP_CMD_MID, PROP_CC[cc_key])

            # Landing gear commands
            elif cmd_type in ("LG_" + k for k in LG_CC):
                cc_key = cmd_type.replace("LG_", "")
                pkt = build_ccsds_cmd(LA_LG_CMD_MID, LG_CC[cc_key])

            # Docking commands
            elif cmd_type in ("DOCK_" + k for k in DOCK_CC):
                cc_key = cmd_type.replace("DOCK_", "")
                pkt = build_ccsds_cmd(LA_DOCK_CMD_MID, DOCK_CC[cc_key])

            # NAV commands
            elif cmd_type == "NAV_SET_DEST":
                dest_id = params.get("destId", 0)
                payload = struct.pack("B", dest_id)
                pkt = build_ccsds_cmd(LA_NAV_CMD_MID, NAV_CC["SET_DEST"], payload)
                label = f"NAV_SET_DEST({dest_id})"

            # LUNET commands
            elif cmd_type == "LUNET_REFUEL_START":
                pkt = build_ccsds_cmd(LA_LUNET_CMD_MID_CMD, LUNET_CC["REFUEL_START"])
            elif cmd_type == "LUNET_AUTO_REFUEL":
                pkt = build_ccsds_cmd(LA_LUNET_CMD_MID_CMD, LUNET_CC["AUTO_REFUEL"])

            else:
                await websocket.send(json.dumps({
                    "type": "error",
                    "message": f"Unknown command: {cmd_type}"
                }))
                return

            if pkt:
                self.cmd_sock.sendto(pkt, (self.cfs_host, self.ci_port))
                ack = {
                    "type": "cmdAck",
                    "cmd": label,
                    "bytes": len(pkt),
                    "timestamp": time.time(),
                }
                await websocket.send(json.dumps(ack))
                print(f"  → {label} ({len(pkt)} bytes)")

        except Exception as e:
            err = {"type": "error", "message": str(e)}
            await websocket.send(json.dumps(err))
            print(f"  ✗ Error: {e}")

    async def tlm_recv(self):
        """Read UDP as fast as possible, parse, and store latest value per type."""
        loop = asyncio.get_event_loop()
        while self.running:
            try:
                data = await loop.sock_recv(self.tlm_sock, 4096)
                if not data or len(data) < TLM_HDR_LEN:
                    continue

                self.tlm_count += 1
                hdr = decode_tlm_header(data)
                if hdr is None:
                    continue
                mid, seq_count, payload = hdr

                if self.tlm_count <= 5:
                    print(f"  ◄ TLM #{self.tlm_count}: MID=0x{mid:04X} len={len(data)}")

                # Downsample high-rate MIDs — console can't use more than ~2 Hz
                if mid == LA_NAV_STATE_TLM_MID:
                    self.nav_decimator += 1
                    if self.nav_decimator % 20 != 0:  # 40 Hz → 2 Hz
                        continue
                elif mid == LA_PROP_STATUS_TLM_MID:
                    self.prop_decimator += 1
                    if self.prop_decimator % 5 != 0:  # 10 Hz → 2 Hz
                        continue

                parser = TLM_PARSERS.get(mid)
                if parser:
                    parsed = parser(payload)
                    if parsed:
                        parsed["timestamp"] = time.time()
                        msg = json.dumps(parsed)
                        tlm_type = parsed["type"]
                    else:
                        continue
                else:
                    if mid not in self.unknown_mids:
                        self.unknown_mids.add(mid)
                        print(f"  · MID 0x{mid:04X} (no parser, len={len(data)})")
                    continue  # Drop unparsed MIDs entirely — they have no console handler

                # Diagnostic: log LG/DOCK arrivals with gap detection
                now = time.time()
                if mid == LA_LG_STATUS_TLM_MID:
                    gap = now - self._diag_lg_last if self._diag_lg_last else 0
                    self._diag_lg_last = now
                    state = parsed.get("gearStateName", "?")
                    flag = " *** GAP" if gap > 2.0 else ""
                    print(f"  ◄ LG  seq={seq_count:5d} state={state:10s} gap={gap:.1f}s{flag}")
                elif mid == LA_DOCK_STATUS_TLM_MID:
                    gap = now - self._diag_dock_last if self._diag_dock_last else 0
                    self._diag_dock_last = now
                    state = parsed.get("collarStateName", "?")
                    flag = " *** GAP" if gap > 2.0 else ""
                    print(f"  ◄ DOCK seq={seq_count:5d} state={state:10s} gap={gap:.1f}s{flag}")

                # Store latest value — overwrites previous, never drops other types
                async with self._lock:
                    self._latest[tlm_type] = msg
                    self._dirty.add(tlm_type)

            except BlockingIOError:
                await asyncio.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"  ✗ TLM recv error: {e}")
                    await asyncio.sleep(0.1)

    async def tlm_broadcast(self):
        """Sweep latest-value table at 10 Hz, push all dirty entries to clients."""
        while self.running:
            try:
                await asyncio.sleep(0.1)  # 10 Hz sweep rate
                if not self.clients or not self._dirty:
                    continue

                # Snapshot and clear dirty set under lock
                async with self._lock:
                    to_send = [self._latest[t] for t in self._dirty]
                    self._dirty.clear()

                # Send all dirty entries to all clients concurrently
                dead = set()
                for ws in list(self.clients):
                    try:
                        for msg in to_send:
                            await ws.send(msg)
                    except websockets.exceptions.ConnectionClosed:
                        dead.add(ws)
                self.clients -= dead

            except Exception as e:
                if self.running:
                    print(f"  ✗ TLM broadcast error: {e}")
                    await asyncio.sleep(0.1)


# ──────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Luna-Aegis Short Hopper — WebSocket-to-UDP Bridge",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
  Browser (WS :8765)  ←→  Bridge  ←→  cFS (CI_LAB :1234 / TO_LAB :2234)
        """
    )
    parser.add_argument("--cfs-host", default="127.0.0.1")
    parser.add_argument("--ci-port", type=int, default=1234)
    parser.add_argument("--tlm-port", type=int, default=2234)
    parser.add_argument("--ws-port", type=int, default=8765)
    args = parser.parse_args()

    print()
    print("  ╔══════════════════════════════════════════════╗")
    print("  ║  Luna-Aegis · Short Hopper · WS-UDP Bridge  ║")
    print("  ╚══════════════════════════════════════════════╝")
    print()

    bridge = LABridge(args.cfs_host, args.ci_port, args.tlm_port, args.ws_port)
    try:
        asyncio.run(bridge.start())
    except KeyboardInterrupt:
        print("\n  Shutting down.")


if __name__ == "__main__":
    main()
