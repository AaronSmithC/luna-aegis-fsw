#!/usr/bin/env python3
"""
Luna-Aegis Short Hopper — Terminal Mission Commander
=====================================================

Interactive terminal console for commanding missions and viewing telemetry.
Sends CCSDS commands directly to cFS CI_LAB over UDP.
Receives telemetry from TO_LAB for real-time status display.

Works alongside the web console — both see the same live telemetry.

Usage:
  python3 la_commander.py [--cfs-host 127.0.0.1] [--ci-port 1234] [--tlm-port 2234]

Commands:
  start / go         Start autonomous mission
  abort              Emergency abort
  manual / auto      Toggle mission mode
  arm / ignite / shutdown   Engine commands
  deploy / retract   Landing gear
  mate / demate      Docking collar
  status / s         Show current telemetry snapshot
  hop <km>           Plan and execute a hop of specified range
  campaign <sites>   Execute multi-hop campaign (e.g. "campaign 3" for 3 hops)
  survey <name>      Named site survey with full telemetry logging
  wait <phase>       Block until a specific flight phase
  log                Show mission log
  fuel               Show propellant budget
  help / ?           Show commands
  quit / q           Exit
"""

import asyncio
import json
import socket
import struct
import sys
import time
import os
import readline
import threading
from datetime import datetime

# ── CCSDS Construction (same as bridge) ──

_cmd_seq = 0
def build_cmd(mid, fc, payload=b""):
    global _cmd_seq
    _cmd_seq = (_cmd_seq + 1) & 0x3FFF
    total = 8 + len(payload)
    length = total - 7
    hdr = struct.pack(">HHH", mid, 0xC000 | _cmd_seq, length)
    sec = struct.pack("BB", fc, 0)
    return hdr + sec + payload

# MIDs
LA_MM_CMD_MID    = 0x1920
LA_PROP_CMD_MID  = 0x1922
LA_LG_CMD_MID    = 0x1924
LA_DOCK_CMD_MID  = 0x1926
TO_LAB_CMD_MID   = 0x1880

# TLM MIDs
LA_MM_PHASE_TLM_MID    = 0x1821
LA_PROP_STATUS_TLM_MID = 0x1823
LA_EPS_STATUS_TLM_MID  = 0x1831
LA_LSS_STATUS_TLM_MID  = 0x1835
LA_COMM_LINK_TLM_MID   = 0x1837
LA_LG_STATUS_TLM_MID   = 0x1825
LA_DOCK_STATUS_TLM_MID = 0x1827
LA_NAV_STATE_TLM_MID   = 0x1815

# Command codes
MM_START = 2; MM_ABORT = 3; MM_MANUAL = 4; MM_AUTO = 5
PROP_ARM = 2; PROP_START = 4; PROP_SHUTDOWN = 5
LG_DEPLOY = 2; LG_RETRACT = 3
DOCK_MATE = 2; DOCK_DEMATE = 3

PHASES = {0:"PREFLIGHT", 1:"POWERED_ASC", 2:"COAST", 3:"POWERED_DES",
          4:"HOVER", 5:"TERMINAL", 6:"LANDED", 7:"ABORT", 8:"SAFED"}
ENGINE = {0:"OFF", 1:"CHILL", 2:"IGNITION", 3:"RUNNING", 4:"SHUTDOWN", 5:"FAULT"}
GEAR = {0:"STOWED", 1:"DEPLOYING", 2:"DEPLOYED", 3:"RETRACTING"}
DOCK = {0:"RETRACTED", 1:"EXTENDING", 2:"SOFT_DOCK", 3:"HARD_DOCK"}

TLM_HDR_LEN = 16

# ── Colors ──
C  = "\033[0;36m"   # cyan
G  = "\033[0;32m"   # green
Y  = "\033[0;33m"   # yellow/amber
R  = "\033[0;31m"   # red
M  = "\033[0;35m"   # magenta
W  = "\033[0;37m"   # white
D  = "\033[0;90m"   # dim
B  = "\033[1m"      # bold
RST = "\033[0m"

# ── Telemetry State ──
class TlmState:
    def __init__(self):
        self.phase = 0
        self.phase_name = "---"
        self.met = 0.0
        self.dv_remain = 0.0
        self.prop_remain = 0.0
        self.abort_reason = 0
        self.mode = 0
        self.engine_state = 0
        self.engine_name = "---"
        self.ox_mass = 0.0
        self.fuel_mass = 0.0
        self.thrust = 0.0
        self.chamber_pc = 0.0
        self.bus_v = 0.0
        self.batt_soc = 0.0
        self.cabin_press = 0.0
        self.pp_o2 = 0.0
        self.pp_co2 = 0.0
        self.gear_state = 0
        self.gear_name = "---"
        self.dock_state = 0
        self.dock_name = "---"
        self.link_name = "---"
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.tlm_count = 0
        self.last_update = 0
        self.initial_prop = 2750.0
        self.lock = threading.Lock()

    def update_mm(self, payload):
        if len(payload) < 28:
            return
        with self.lock:
            self.phase, prev, self.abort_reason, self.mode = struct.unpack_from("BBBB", payload, 0)
            self.met, self.dv_remain, self.prop_remain = struct.unpack_from("<ddd", payload, 4)
            self.phase_name = PHASES.get(self.phase, "?")
            self.last_update = time.time()

    def update_prop(self, payload):
        if len(payload) < 58:
            return
        with self.lock:
            vals = struct.unpack_from("<7d", payload, 0)
            self.chamber_pc = vals[0]
            self.ox_mass = vals[3]
            self.fuel_mass = vals[4]
            self.thrust = vals[6]
            self.engine_state = struct.unpack_from("B", payload, 56)[0]
            self.engine_name = ENGINE.get(self.engine_state, "?")

    def update_eps(self, payload):
        if len(payload) < 42:
            return
        with self.lock:
            vals = struct.unpack_from("<5d", payload, 0)
            self.bus_v = vals[0]
            self.batt_soc = vals[2]

    def update_lss(self, payload):
        if len(payload) < 56:
            return
        with self.lock:
            vals = struct.unpack_from("<7d", payload, 0)
            self.cabin_press = vals[0]
            self.pp_o2 = vals[1]
            self.pp_co2 = vals[2]

    def update_lg(self, payload):
        if len(payload) < 6:
            return
        with self.lock:
            self.gear_state = struct.unpack_from("B", payload, 0)[0]
            self.gear_name = GEAR.get(self.gear_state, "?")

    def update_dock(self, payload):
        if len(payload) < 4:
            return
        with self.lock:
            self.dock_state = struct.unpack_from("B", payload, 0)[0]
            self.dock_name = DOCK.get(self.dock_state, "?")

    def update_nav(self, payload):
        if len(payload) < 48:
            return
        with self.lock:
            p = struct.unpack_from("<3d", payload, 0)
            v = struct.unpack_from("<3d", payload, 24)
            self.pos = [p[0], p[1], p[2]]
            self.vel = [v[0], v[1], v[2]]

    def update_comm(self, payload):
        if len(payload) < 34:
            return
        with self.lock:
            link = struct.unpack_from("B", payload, 32)[0]
            names = {0:"NO_LINK", 1:"UHF", 2:"S-BAND", 3:"HGA"}
            self.link_name = names.get(link, "?")

# ── Commander ──

class Commander:
    def __init__(self, cfs_host, ci_port, ws_port):
        self.cfs_host = cfs_host
        self.ci_port = ci_port
        self.ws_port = ws_port
        self.state = TlmState()
        self.mission_log = []
        self.running = True
        self.hop_count = 0
        self.total_prop_used = 0.0

        # UDP socket for commands only (no TLM binding)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, mid, fc, payload=b""):
        pkt = build_cmd(mid, fc, payload)
        self.cmd_sock.sendto(pkt, (self.cfs_host, self.ci_port))

    def send_to_enable(self):
        payload = "16:127.0.0.1".encode("ascii").ljust(16, b"\x00")
        self.send(TO_LAB_CMD_MID, 6, payload)

    def log(self, msg, color=W):
        ts = datetime.now().strftime("%H:%M:%S")
        self.mission_log.append((ts, msg))
        print(f"  {D}{ts}{RST}  {color}{msg}{RST}")

    def tlm_listener(self):
        """Connect to bridge WebSocket and receive parsed JSON telemetry."""
        import websockets.sync.client as ws_client

        ws_url = f"ws://127.0.0.1:{self.ws_port}"
        while self.running:
            try:
                with ws_client.connect(ws_url) as ws:
                    while self.running:
                        try:
                            raw = ws.recv(timeout=1.0)
                            msg = json.loads(raw)
                            self._handle_ws_msg(msg)
                        except TimeoutError:
                            continue
                        except Exception:
                            continue
            except Exception:
                # Reconnect after delay
                if self.running:
                    time.sleep(2)

    def _handle_ws_msg(self, msg):
        t = msg.get("type", "")
        s = self.state
        with s.lock:
            if t == "mm_phase":
                s.phase = msg.get("phase", 0)
                s.phase_name = msg.get("phaseName", "---")
                s.met = msg.get("met", 0)
                s.dv_remain = msg.get("dvRemaining", 0)
                s.prop_remain = msg.get("propRemaining", 0)
                s.abort_reason = msg.get("abortReason", 0)
                s.mode = msg.get("missionMode", 0)
            elif t == "prop_status":
                s.engine_state = msg.get("engineState", 0)
                s.engine_name = msg.get("engineStateName", "---")
                s.ox_mass = msg.get("oxMass", 0)
                s.fuel_mass = msg.get("fuelMass", 0)
                s.thrust = msg.get("thrustEst", 0)
                s.chamber_pc = msg.get("chamberPress", 0)
            elif t == "eps_status":
                s.bus_v = msg.get("busVoltage", 0)
                s.batt_soc = msg.get("battSOC", 0)
            elif t == "lss_status":
                s.cabin_press = msg.get("cabinPress", 0)
                s.pp_o2 = msg.get("ppO2", 0)
                s.pp_co2 = msg.get("ppCO2", 0)
            elif t == "lg_status":
                s.gear_state = msg.get("gearState", 0)
                s.gear_name = msg.get("gearStateName", "---")
            elif t == "dock_status":
                s.dock_state = msg.get("collarState", 0)
                s.dock_name = msg.get("collarStateName", "---")
            elif t == "nav_state":
                s.pos = [msg.get("posX", 0), msg.get("posY", 0), msg.get("posZ", 0)]
                s.vel = [msg.get("velX", 0), msg.get("velY", 0), msg.get("velZ", 0)]
            elif t == "comm_link":
                s.link_name = msg.get("linkStateName", "---")
            else:
                return  # skip non-telemetry messages
        s.tlm_count += 1
        s.last_update = time.time()

    def wait_phase(self, target, timeout=120):
        """Block until phase matches target (name or number). Returns True if reached."""
        if isinstance(target, str):
            target_num = None
            for k, v in PHASES.items():
                if v == target.upper():
                    target_num = k
                    break
            if target_num is None:
                self.log(f"Unknown phase: {target}", R)
                return False
        else:
            target_num = target

        start = time.time()
        while time.time() - start < timeout:
            if self.state.phase == target_num:
                return True
            if self.state.phase == 7: # ABORT
                return False
            time.sleep(0.2)
        self.log(f"Timeout waiting for {PHASES.get(target_num, '?')}", R)
        return False

    def wait_engine(self, target_state, timeout=30):
        start = time.time()
        while time.time() - start < timeout:
            if self.state.engine_state == target_state:
                return True
            time.sleep(0.1)
        return False

    def print_status(self):
        s = self.state
        with s.lock:
            phase_color = {0:D, 1:G, 2:C, 3:Y, 4:M, 5:R, 6:G, 7:R, 8:D}.get(s.phase, W)
            eng_color = G if s.engine_state == 3 else Y if s.engine_state in (1,2) else D
            prop_total = s.ox_mass + s.fuel_mass
            prop_pct = (prop_total / s.initial_prop * 100) if s.initial_prop > 0 else 0
            prop_bar = self._bar(prop_pct, 20)
            soc_bar = self._bar(s.batt_soc, 12)

            print()
            print(f"  {C}{'═'*60}{RST}")
            print(f"  {C}  LUNA-AEGIS SHORT HOPPER — STATUS{RST}")
            print(f"  {C}{'═'*60}{RST}")
            print(f"  {D}Phase:{RST}    {phase_color}{B}{s.phase_name}{RST}{'':>8}{D}MET:{RST} {Y}{self._fmt_met(s.met)}{RST}")
            print(f"  {D}Mode:{RST}     {'AUTO' if s.mode==0 else 'MANUAL' if s.mode==1 else 'ABORT-SEQ'}")
            if s.abort_reason:
                ar = {1:"PROPULSION", 2:"NAV LOST", 3:"COMMANDED"}.get(s.abort_reason, "?")
                print(f"  {D}Abort:{RST}    {R}{ar}{RST}")
            print(f"  {D}Δv:{RST}       {C}{s.dv_remain:.0f} m/s{RST}")
            print()
            print(f"  {D}Engine:{RST}   {eng_color}{s.engine_name}{RST}{'':>6}{D}Thrust:{RST} {G}{s.thrust/1000:.1f} kN{RST}{'':>4}{D}Pc:{RST} {W}{s.chamber_pc:.0f} kPa{RST}")
            print(f"  {D}Prop:{RST}     {C}{prop_total:.0f} kg{RST} ({prop_pct:.0f}%) {D}{prop_bar}{RST}")
            print(f"  {D}  LOX:{RST}    {s.ox_mass:.0f} kg{'':>8}{D}LH₂:{RST}  {s.fuel_mass:.0f} kg")
            print()
            print(f"  {D}Bus:{RST}      {C}{s.bus_v:.1f} V{RST}{'':>8}{D}SOC:{RST} {G}{s.batt_soc:.0f}%{RST} {D}{soc_bar}{RST}")
            print(f"  {D}Cabin:{RST}    {s.cabin_press:.1f} kPa{'':>4}{D}O₂:{RST} {s.pp_o2:.1f}{'':>4}{D}CO₂:{RST} {s.pp_co2:.2f} kPa")
            print(f"  {D}Gear:{RST}     {s.gear_name}{'':>10}{D}Dock:{RST} {s.dock_name}{'':>8}{D}Comm:{RST} {s.link_name}")
            print(f"  {D}Pos:{RST}      X={s.pos[0]:.0f}  Y={s.pos[1]:.0f}  Z={s.pos[2]:.0f} m")
            print(f"  {D}Vel:{RST}      X={s.vel[0]:.1f}  Y={s.vel[1]:.1f}  Z={s.vel[2]:.1f} m/s")
            print(f"  {C}{'═'*60}{RST}")
            print(f"  {D}TLM packets: {s.tlm_count}   Hops: {self.hop_count}   Prop used: {self.total_prop_used:.0f} kg{RST}")
            print()

    def print_fuel(self):
        s = self.state
        with s.lock:
            total = s.ox_mass + s.fuel_mass
            used = s.initial_prop - total
            pct = (total / s.initial_prop * 100) if s.initial_prop > 0 else 0
            dv = s.dv_remain

            print()
            print(f"  {C}{'─'*50}{RST}")
            print(f"  {C}  PROPELLANT BUDGET{RST}")
            print(f"  {C}{'─'*50}{RST}")
            print(f"  {D}Initial Load:{RST}  {s.initial_prop:.0f} kg")
            print(f"  {D}Remaining:{RST}     {G}{total:.0f} kg{RST} ({pct:.1f}%)")
            print(f"  {D}  LOX:{RST}         {s.ox_mass:.0f} kg")
            print(f"  {D}  LH₂:{RST}         {s.fuel_mass:.0f} kg")
            print(f"  {D}Used:{RST}          {R}{used:.0f} kg{RST}")
            print(f"  {D}Session used:{RST}  {self.total_prop_used:.0f} kg (across {self.hop_count} hops)")
            print(f"  {D}Δv remaining:{RST}  {C}{dv:.0f} m/s{RST}")
            est_hops = int(dv / 400) if dv > 400 else 0
            print(f"  {D}Est. hops:{RST}     ~{est_hops} (at ~400 m/s per short hop)")
            print(f"  {C}{'─'*50}{RST}")
            print()

    def execute_hop(self, label=""):
        """Execute one complete hop: start → wait for LANDED."""
        prop_before = self.state.ox_mass + self.state.fuel_mass
        dv = self.state.dv_remain

        # If Δv hasn't been computed yet (MM hasn't seen PROP tlm),
        # calculate it from prop mass directly
        if dv < 1.0 and prop_before > 100:
            m_wet = 5250.0 + prop_before
            m_dry = 5250.0
            import math
            dv = 440.0 * 9.80665 * math.log(m_wet / m_dry) if m_wet > m_dry else 0

        self.log(f"{'═'*50}", C)
        self.log(f"HOP {self.hop_count + 1}{': ' + label if label else ''}", C)
        self.log(f"{'═'*50}", C)
        self.log(f"Propellant: {prop_before:.0f} kg  Δv: {dv:.0f} m/s", D)

        if prop_before < 100:
            self.log("INSUFFICIENT PROPELLANT — cannot execute hop safely", R)
            return False

        self.log("Commanding START_MISSION", G)
        self.send(LA_MM_CMD_MID, MM_START)
        time.sleep(0.5)

        # Wait for POWERED_ASC
        self.log("Waiting for engine start...", D)
        if not self.wait_phase("POWERED_ASC", timeout=5):
            self.log("Failed to enter POWERED_ASC", R)
            return False
        self.log(f"POWERED_ASC — engine {self.state.engine_name}", G)

        # Wait for engine RUNNING
        if self.wait_engine(3, timeout=10):
            self.log(f"Engine RUNNING — thrust {self.state.thrust/1000:.1f} kN", G)
        else:
            self.log(f"Engine not running (state: {self.state.engine_name})", Y)

        # Wait through mission phases
        for target in ["COAST", "POWERED_DES", "HOVER", "TERMINAL", "LANDED"]:
            if self.state.phase == 7:  # ABORT
                self.log(f"ABORT detected — mission terminated", R)
                self.wait_phase("SAFED", timeout=30)
                return False
            if self.wait_phase(target, timeout=60):
                extra = ""
                if target == "COAST":
                    extra = f" — engine {self.state.engine_name}"
                elif target == "POWERED_DES":
                    extra = f" — re-ignition"
                elif target == "HOVER":
                    extra = f" — gear {self.state.gear_name}"
                elif target == "LANDED":
                    extra = f" — touchdown"
                self.log(f"{target}{extra}", G if target == "LANDED" else C)
            else:
                self.log(f"Timeout waiting for {target}", R)
                return False

        prop_after = self.state.ox_mass + self.state.fuel_mass
        used = prop_before - prop_after
        self.total_prop_used += used
        self.hop_count += 1

        self.log(f"HOP COMPLETE — used {used:.0f} kg propellant", G)
        self.log(f"Remaining: {prop_after:.0f} kg  Δv: {self.state.dv_remain:.0f} m/s", D)
        return True

    def cmd_hop(self, args):
        if args:
            try:
                dist_km = float(args)
            except ValueError:
                self.log("Usage: hop <distance_km>", Y)
                return
            label = f"{dist_km:.0f} km range hop"
        else:
            label = "standard hop"

        self.execute_hop(label)

    def cmd_campaign(self, args):
        try:
            num_hops = int(args) if args else 3
        except ValueError:
            self.log("Usage: campaign <number_of_hops>", Y)
            return

        if num_hops < 1 or num_hops > 20:
            self.log("Campaign must be 1-20 hops", Y)
            return

        sites = [
            "Shackleton Crater Rim",
            "Malapert Massif",
            "Connecting Ridge Alpha",
            "de Gerlache Crater",
            "Leibnitz Beta Plateau",
            "Sverdrup Rim Station",
            "Amundsen North",
            "Shoemaker Crater Floor",
            "Faustini Rim",
            "Cabeus Site",
            "Nobile Rim Station",
            "Scott Crater",
            "Haworth Crater Floor",
            "Slater Plain",
            "Idel'son Rim",
            "Wiechert Station",
            "Manzinus Ridge",
            "Bhabha Crater",
            "Drygalski North",
            "South Pole Station Alpha",
        ]

        print()
        self.log(f"{'═'*50}", M)
        self.log(f"CAMPAIGN: {num_hops}-HOP SOUTH POLE SURVEY", M)
        self.log(f"{'═'*50}", M)

        prop_start = self.state.ox_mass + self.state.fuel_mass
        campaign_start = time.time()

        for i in range(num_hops):
            site = sites[i % len(sites)]
            print()
            self.log(f"{'─'*50}", D)
            self.log(f"Site {i+1}/{num_hops}: {site}", M)
            self.log(f"{'─'*50}", D)

            if not self.execute_hop(f"→ {site}"):
                self.log(f"Campaign terminated at hop {i+1}", R)
                break

            # Brief settle time between hops
            if i < num_hops - 1:
                self.log("Surface operations... (3s)", D)
                time.sleep(3)

        elapsed = time.time() - campaign_start
        prop_end = self.state.ox_mass + self.state.fuel_mass
        total_used = prop_start - prop_end

        print()
        self.log(f"{'═'*50}", M)
        self.log(f"CAMPAIGN SUMMARY", M)
        self.log(f"{'═'*50}", M)
        self.log(f"Hops completed: {self.hop_count}", W)
        self.log(f"Elapsed time: {elapsed:.0f} s ({elapsed/60:.1f} min)", W)
        self.log(f"Propellant used: {total_used:.0f} kg", W)
        self.log(f"Remaining: {prop_end:.0f} kg  Δv: {self.state.dv_remain:.0f} m/s", W)
        est = int(self.state.dv_remain / 400) if self.state.dv_remain > 400 else 0
        self.log(f"Estimated remaining hops: ~{est}", W)
        self.log(f"{'═'*50}", M)
        print()

    def cmd_survey(self, args):
        name = args if args else f"Site-{self.hop_count + 1:03d}"
        print()
        self.log(f"{'═'*50}", M)
        self.log(f"SITE SURVEY: {name}", M)
        self.log(f"{'═'*50}", M)

        self.log("Pre-flight status:", D)
        self.print_status()

        if self.execute_hop(f"Survey → {name}"):
            self.log("Post-landing survey complete:", D)
            self.print_status()
            self.log("Initiating docking sequence...", C)
            self.send(LA_DOCK_CMD_MID, DOCK_MATE)
            time.sleep(5)
            self.log(f"Dock state: {self.state.dock_name}", C)
            self.log(f"Survey of {name} complete", G)
        else:
            self.log(f"Survey of {name} failed", R)

    def print_log(self):
        print()
        print(f"  {C}  MISSION LOG ({len(self.mission_log)} entries){RST}")
        print(f"  {C}{'─'*50}{RST}")
        for ts, msg in self.mission_log[-30:]:
            print(f"  {D}{ts}{RST}  {msg}")
        print()

    def print_help(self):
        print(f"""
  {C}{'═'*60}{RST}
  {C}  LUNA-AEGIS MISSION COMMANDER — COMMANDS{RST}
  {C}{'═'*60}{RST}

  {G}Mission Control{RST}
    {B}start{RST} / {B}go{RST}           Start autonomous mission
    {B}abort{RST}                Emergency abort
    {B}manual{RST} / {B}auto{RST}        Toggle mission mode
    {B}wait{RST} <phase>         Wait for phase (e.g. "wait LANDED")

  {G}Subsystems{RST}
    {B}arm{RST}                  Arm engine
    {B}ignite{RST}               Start engine sequence
    {B}shutdown{RST}             Engine shutdown
    {B}deploy{RST} / {B}retract{RST}     Landing gear
    {B}mate{RST} / {B}demate{RST}        Docking collar

  {G}Operations{RST}
    {B}hop{RST} [km]             Execute a single hop
    {B}campaign{RST} <N>         Multi-hop campaign (N sites)
    {B}survey{RST} <name>        Named site survey with docking

  {G}Telemetry{RST}
    {B}status{RST} / {B}s{RST}           Show telemetry snapshot
    {B}fuel{RST} / {B}f{RST}             Propellant budget
    {B}log{RST} / {B}l{RST}             Mission log

  {B}quit{RST} / {B}q{RST}              Exit
  {C}{'═'*60}{RST}
""")

    def _bar(self, pct, width=20):
        filled = int(pct / 100 * width)
        filled = max(0, min(width, filled))
        if pct > 60:
            color = G
        elif pct > 25:
            color = Y
        else:
            color = R
        return f"{color}[{'█' * filled}{'░' * (width - filled)}]{RST}"

    def _fmt_met(self, s):
        h = int(s // 3600)
        m = int((s % 3600) // 60)
        sec = int(s % 60)
        return f"T+{h:02d}:{m:02d}:{sec:02d}"

    def run(self):
        # Start TLM listener thread
        t = threading.Thread(target=self.tlm_listener, daemon=True)
        t.start()

        print(f"""
  {C}╔══════════════════════════════════════════════════════╗{RST}
  {C}║  LUNA-AEGIS · SHORT HOPPER · MISSION COMMANDER      ║{RST}
  {C}╚══════════════════════════════════════════════════════╝{RST}

  {D}CMD:{RST}  {self.cfs_host}:{self.ci_port} (UDP)
  {D}TLM:{RST}  ws://127.0.0.1:{self.ws_port} (via bridge)
  {D}Type {B}help{RST}{D} for commands, {B}status{RST}{D} for telemetry{RST}
""")

        # Wait for first telemetry
        waited = 0
        while self.state.tlm_count == 0 and waited < 5:
            time.sleep(0.5)
            waited += 0.5

        if self.state.tlm_count > 0:
            self.log(f"Telemetry active — {self.state.tlm_count} packets received", G)
            self.log(f"Phase: {self.state.phase_name}  Engine: {self.state.engine_name}", D)
        else:
            self.log("No telemetry received — is cFS running?", Y)

        while self.running:
            try:
                raw = input(f"  {C}hopper>{RST} ").strip()
                if not raw:
                    continue

                parts = raw.split(None, 1)
                cmd = parts[0].lower()
                args = parts[1] if len(parts) > 1 else ""

                if cmd in ("quit", "q", "exit"):
                    self.running = False
                    break
                elif cmd in ("help", "?", "h"):
                    self.print_help()
                elif cmd in ("status", "s"):
                    self.print_status()
                elif cmd in ("fuel", "f"):
                    self.print_fuel()
                elif cmd in ("log", "l"):
                    self.print_log()
                elif cmd in ("start", "go"):
                    self.log("Commanding START_MISSION", G)
                    self.send(LA_MM_CMD_MID, MM_START)
                elif cmd == "abort":
                    self.log("Commanding ABORT", R)
                    self.send(LA_MM_CMD_MID, MM_ABORT)
                elif cmd == "manual":
                    self.log("Commanding MANUAL mode", Y)
                    self.send(LA_MM_CMD_MID, MM_MANUAL)
                elif cmd == "auto":
                    self.log("Commanding AUTO mode", G)
                    self.send(LA_MM_CMD_MID, MM_AUTO)
                elif cmd == "arm":
                    self.log("Commanding PROP ARM", Y)
                    self.send(LA_PROP_CMD_MID, PROP_ARM)
                elif cmd in ("ignite", "start_engine"):
                    self.log("Commanding PROP START", G)
                    self.send(LA_PROP_CMD_MID, PROP_START)
                elif cmd == "shutdown":
                    self.log("Commanding PROP SHUTDOWN", R)
                    self.send(LA_PROP_CMD_MID, PROP_SHUTDOWN)
                elif cmd == "deploy":
                    self.log("Commanding LG DEPLOY", G)
                    self.send(LA_LG_CMD_MID, LG_DEPLOY)
                elif cmd == "retract":
                    self.log("Commanding LG RETRACT", Y)
                    self.send(LA_LG_CMD_MID, LG_RETRACT)
                elif cmd == "mate":
                    self.log("Commanding DOCK MATE", C)
                    self.send(LA_DOCK_CMD_MID, DOCK_MATE)
                elif cmd == "demate":
                    self.log("Commanding DOCK DEMATE", Y)
                    self.send(LA_DOCK_CMD_MID, DOCK_DEMATE)
                elif cmd == "hop":
                    self.cmd_hop(args)
                elif cmd == "campaign":
                    self.cmd_campaign(args)
                elif cmd == "survey":
                    self.cmd_survey(args)
                elif cmd == "wait":
                    if args:
                        self.log(f"Waiting for phase {args.upper()}...", D)
                        if self.wait_phase(args.upper()):
                            self.log(f"Phase {args.upper()} reached", G)
                        else:
                            self.log(f"Did not reach phase {args.upper()}", R)
                    else:
                        self.log("Usage: wait <PHASE_NAME>", Y)
                else:
                    self.log(f"Unknown command: {cmd} — type 'help'", Y)

            except (KeyboardInterrupt, EOFError):
                print()
                self.running = False
                break

        print(f"\n  {D}Mission Commander terminated.{RST}\n")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Luna-Aegis Terminal Mission Commander")
    parser.add_argument("--cfs-host", default="127.0.0.1")
    parser.add_argument("--ci-port", type=int, default=1234)
    parser.add_argument("--ws-port", type=int, default=8765,
                        help="Bridge WebSocket port (default: 8765)")
    args = parser.parse_args()

    cmd = Commander(args.cfs_host, args.ci_port, args.ws_port)
    cmd.run()


if __name__ == "__main__":
    main()
