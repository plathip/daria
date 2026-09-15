"""Fire Console, the operator's interface for the firefighting drone.

One-action UI over ArduPilot: a live SIGNAL/ARMING readiness panel, the
estate map with editable factory pins, click a factory -> the drone arms,
flies out, returns, lands and disarms by itself; a RETURN HOME NOW abort
appears while airborne; everything is narrated into the scrolling log.
(Advanced verbs, dispatch/goto/nudge/hold/drop, remain API-only.)

Usage:
  python fire_console.py --sim               # boots SITL over the estate (demo)
  python fire_console.py --link COM4         # real drone over USB
  python fire_console.py --link COM7         # e.g. Bluetooth bridge COM port
Then open  http://127.0.0.1:8008  in a browser.

Manual FLYING always stays on the radio (mode switch overrides everything).
This console covers the point-and-click side of operations.
"""
import argparse
import json
import math
import os
import sys
import queue
import re
import subprocess
import threading
import time

from flask import Flask, Response, jsonify, request
from pymavlink import mavutil

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESTATE_PATH = os.path.join(ROOT, 'console', 'site.json')          # launch pad + factory pins, made on first run
LEGACY_ESTATE_PATH = os.path.join(ROOT, 'missions', 'factories.json')
SITL_EXE = os.path.join(ROOT, 'tools', 'sitl', 'ArduCopter.exe')
SITL_DEFAULTS = os.path.join(ROOT, 'tools', 'sitl', 'copter-defaults.parm')
SIM_DIR = os.path.join(ROOT, 'sim')

CRUISE_ALT = 25.0
RELEASE_CH = 6          # payload release servo: pad silkscreened S5 = output 6
RELEASE_DROP_PWM = 2500  # full drop swing, user-approved on the bench (SERVO6_MAX 2500)
RELEASE_LOCK_PWM = 1500

SETTINGS_FILE = os.path.join(ROOT, 'console', 'settings.json')
DEFAULT_SETTINGS = {
    'site': None,                       # {'name','lat','lon','zoom'} - where the map opens
    'board': 'speedybeef4v5',           # ArduPilot board id (firmware folder name)
    'link': 'auto',                     # auto | ble | usb | radio  (auto: USB cable when plugged in, else Bluetooth)
    'ble_name': 'F405',                 # Bluetooth name fragment the bridge looks for
    'ble_addr': '',                     # optional: the drone's Bluetooth address (modules that advertise no name)
    'release': {'ch': 6, 'drop_pwm': 2500, 'lock_pwm': 1500},
    'cruise_alt': 25,
}
SETTINGS = {}


def apply_settings():
    """Push the editable settings into the module globals the flight code reads."""
    global CRUISE_ALT, RELEASE_CH, RELEASE_DROP_PWM, RELEASE_LOCK_PWM
    r = SETTINGS.get('release') or {}
    RELEASE_CH = int(r.get('ch', 6))
    RELEASE_DROP_PWM = int(r.get('drop_pwm', 2500))
    RELEASE_LOCK_PWM = int(r.get('lock_pwm', 1500))
    CRUISE_ALT = float(SETTINGS.get('cruise_alt', 25))


def load_settings():
    global SETTINGS
    SETTINGS = json.loads(json.dumps(DEFAULT_SETTINGS))
    try:
        with open(SETTINGS_FILE, encoding='utf-8') as f:
            saved = json.load(f)
        for k, v in saved.items():
            if isinstance(v, dict) and isinstance(SETTINGS.get(k), dict):
                SETTINGS[k].update(v)
            else:
                SETTINGS[k] = v
    except FileNotFoundError:
        pass
    apply_settings()


def save_settings():
    with open(SETTINGS_FILE, 'w', encoding='utf-8') as f:
        json.dump(SETTINGS, f, indent=2)


load_settings()
EARTH_R = 6378137.0

def _load_estate():
    """Site file: launch pad (map fallback, SITL home) and the factory pins placed in Operations."""
    for p in (ESTATE_PATH, LEGACY_ESTATE_PATH):
        if os.path.exists(p):
            with open(p, encoding='utf-8') as f:
                return json.load(f)
    return {'launch_pad': None, 'factories': []}


ESTATE = _load_estate()


def usb_board_port():
    """COM port of an ArduPilot board on the USB cable, or None."""
    try:
        from serial.tools import list_ports
        return next((p.device for p in list_ports.comports() if p.vid == 0x1209), None)
    except Exception:
        return None


def resolve_link():
    """(link, baud, kind) from the settings - evaluated at every (re)connect, so a changed setting or a
    cable plugged in or pulled later is picked up without restarting. kind: bluetooth | usb | radio.
    auto (default): the USB cable whenever an ArduPilot board is on it (bench), else Bluetooth (field)."""
    from serial.tools import list_ports
    mode = SETTINGS.get('link', 'auto')
    if mode == 'auto':
        port = usb_board_port()
        if port:
            return port, 115200, 'usb'
        return 'tcp:127.0.0.1:5770', 115200, 'bluetooth'
    if mode == 'ble':
        return 'tcp:127.0.0.1:5770', 115200, 'bluetooth'          # fc/bt_bridge.py
    if mode == 'radio':                                            # SiK-style telemetry radio on USB
        port = next((p.device for p in list_ports.comports()
                     if p.vid in (0x10C4, 0x0403, 0x1A86, 0x067B)), None)
        return port, 57600, 'radio'
    return usb_board_port(), 115200, 'usb'


class _LinkReleased(Exception):
    """Raised inside the link loop to drop the connection on purpose (USB tool, changed settings)."""


def latlon_add_m(lat, lon, north_m, east_m):
    nlat = lat + math.degrees(north_m / EARTH_R)
    nlon = lon + math.degrees(east_m / (EARTH_R * math.cos(math.radians(lat))))
    return nlat, nlon


class DroneLink(threading.Thread):
    """Owns the MAVLink connection; executes queued commands; keeps state."""

    daemon = True

    def __init__(self, link, baud=115200, sim=False, resolver=None):
        super().__init__()
        self.link = link
        self.baud = baud
        self.sim = sim
        self.resolver = resolver      # None = fixed --link; else called at every (re)connect
        self.paused = False           # a USB tool needs the port: release it until cleared
        self.relink = False           # settings changed: drop and reconnect with the new link
        self.cmds = queue.Queue()
        self.state = {'connected': False, 'lat': None, 'lon': None,
                      'alt': 0.0, 'heading': 0, 'groundspeed': 0.0,
                      'mode': '-', 'armed': False, 'batt_v': 0.0,
                      'sats': 0, 'fix': 0, 'msgs': [], 'busy': '',
                      'sensors': {}, 'prearm_msgs': [], 'rc': [], 'rc_count': 0,
                      'magcal': None, 'fc_info': {}, 'params': {},
                      # bench/setup telemetry (all additive, read-only)
                      'attitude': None,   # {'roll','pitch','yaw'} deg
                      'vibe': None,       # {'x','y','z','clip'}
                      'ekf': None,        # {'flags','vel','pos_h','pos_v','mag'}
                      'batt_a': None,     # battery current, A
                      'rc_fs': None,      # True while the FC reports "Radio Failsafe"
                      'motor_test': False,  # armed only because a bench motor test is running
                      'link_kind': 'sim' if sim else ('custom' if link else None),
                      'link': link}         # bluetooth / usb / radio and the port or url in use
        self.m = None
        self.release_state = 'unknown'   # locked / open / unknown (dropper)
        self._prearm_seen = {}   # objection text -> last time the FC said it
        self._last_hb = 0.0      # last vehicle heartbeat, for silence detection
        self._last_gcs_hb = 0.0  # last heartbeat WE sent (GCS-failsafe cadence)
        self._last_ack = None    # (command, result) of the newest COMMAND_ACK
        self._param_wait = {}    # PARAM_VALUEs seen since the last request
        self.motor_test_until = 0.0   # ArduPilot ARMS the motors for a bench motor test: until when

    # ------------- helpers -------------
    def note(self, text):
        self.state['msgs'] = ([f'{time.strftime("%H:%M:%S")}  {text}']
                              + self.state['msgs'])[:200]

    def _gcs_hb_tick(self):
        # our GCS heartbeat must keep flowing in EVERY blocking loop, with
        # FS_GCS_ENABLE on, the FC failsafes on our silence
        if time.time() - self._last_gcs_hb > 1.0:
            try:
                self.m.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
            self._last_gcs_hb = time.time()

    def pump(self, seconds=0.0):
        self._gcs_hb_tick()
        # objections the FC hasn't repeated within 25 s count as resolved
        now = time.time()
        self._prearm_seen = {k: v for k, v in self._prearm_seen.items()
                             if now - v < 25.0}
        self.state['prearm_msgs'] = [k for k, _ in sorted(
            self._prearm_seen.items(), key=lambda kv: -kv[1])][:6]
        t0 = time.time()
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                if time.time() - t0 >= seconds:
                    return
                time.sleep(0.02)
                continue
            if (msg.get_srcSystem() != self.m.target_system
                    or msg.get_srcComponent() != 1):
                continue     # only the autopilot itself, never another GCS
            t = msg.get_type()
            s = self.state
            if t == 'HEARTBEAT':
                self._last_hb = time.time()
                if not s['connected']:
                    s['connected'] = True
                    self.note('telemetry restored')
                s['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                s['motor_test'] = bool(s['armed'] and time.time() < self.motor_test_until)
                s['mode'] = mavutil.mode_string_v10(msg)
            elif t == 'GLOBAL_POSITION_INT':
                # (0,0) means "no GPS fix yet", never treat Null Island as real
                if abs(msg.lat) > 10000 or abs(msg.lon) > 10000:
                    s['lat'] = msg.lat / 1e7
                    s['lon'] = msg.lon / 1e7
                s['alt'] = msg.relative_alt / 1000.0
                s['heading'] = msg.hdg / 100.0 if msg.hdg != 65535 else 0
            elif t == 'VFR_HUD':
                s['groundspeed'] = msg.groundspeed
            elif t == 'ATTITUDE':
                yaw = math.degrees(msg.yaw) % 360.0
                s['attitude'] = {'roll': round(math.degrees(msg.roll), 1),
                                 'pitch': round(math.degrees(msg.pitch), 1),
                                 'yaw': round(yaw, 1)}
            elif t == 'VIBRATION':
                s['vibe'] = {'x': round(msg.vibration_x, 1), 'y': round(msg.vibration_y, 1),
                             'z': round(msg.vibration_z, 1),
                             'clip': int(msg.clipping_0 + msg.clipping_1 + msg.clipping_2)}
            elif t == 'EKF_STATUS_REPORT':
                s['ekf'] = {'flags': int(msg.flags), 'vel': round(msg.velocity_variance, 2),
                            'pos_h': round(msg.pos_horiz_variance, 2),
                            'pos_v': round(msg.pos_vert_variance, 2),
                            'mag': round(msg.compass_variance, 2)}
            elif t == 'SERVO_OUTPUT_RAW':
                self._release_raw = getattr(msg, 'servo%d_raw' % RELEASE_CH, 0)
                # a fresh-booted FC outputs TRIM (1500) on the pin, which looks
                # exactly like LOCK, so telemetry may only ever prove 'open'
                # (drop PWM seen); 'locked' comes solely from a deliberate
                # LOCK action by the operator
                if self._release_raw == RELEASE_DROP_PWM:
                    self.release_state = 'open'
                s['release'] = self.release_state
                s['servo_pwm'] = self._release_raw
            elif t == 'SYS_STATUS':
                if msg.voltage_battery != 65535:
                    s['batt_v'] = msg.voltage_battery / 1000.0
                if msg.current_battery != -1:
                    s['batt_a'] = round(msg.current_battery / 100.0, 1)
                mv, en, hl = (mavutil.mavlink,
                              msg.onboard_control_sensors_enabled,
                              msg.onboard_control_sensors_health)
                bits = {'gyro': mv.MAV_SYS_STATUS_SENSOR_3D_GYRO,
                        'accel': mv.MAV_SYS_STATUS_SENSOR_3D_ACCEL,
                        'compass': mv.MAV_SYS_STATUS_SENSOR_3D_MAG,
                        'gps': mv.MAV_SYS_STATUS_SENSOR_GPS,
                        'rc': mv.MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
                        'position': mv.MAV_SYS_STATUS_AHRS,
                        'battery': mv.MAV_SYS_STATUS_SENSOR_BATTERY,
                        'prearm': mv.MAV_SYS_STATUS_PREARM_CHECK}
                s['sensors'] = {k: (bool(hl & b) if (en & b) else None)
                                for k, b in bits.items()}
                if s['sensors'].get('prearm'):
                    s['prearm_msgs'] = []
                    self._prearm_seen = {}
            elif t == 'GPS_RAW_INT':
                s['sats'] = msg.satellites_visible
                s['fix'] = msg.fix_type
            elif t == 'COMMAND_ACK':
                self._last_ack = (msg.command, msg.result)
            elif t == 'RC_CHANNELS':
                s['rc'] = [getattr(msg, 'chan%d_raw' % i) for i in range(1, 17)]
                s['rc_count'] = msg.chancount
            elif t == 'PARAM_VALUE':
                name = msg.param_id.strip('\x00')
                s['params'][name] = msg.param_value
                self._param_wait[name] = msg.param_value
            elif t == 'MAG_CAL_PROGRESS':
                s['magcal'] = {'pct': msg.completion_pct, 'status': 'running', 'report': None}
            elif t == 'MAG_CAL_REPORT':
                ok = msg.cal_status == 4
                s['magcal'] = {'pct': 100 if ok else (s['magcal'] or {}).get('pct', 0),
                               'status': 'success' if ok else 'failed',
                               'report': ('fitness %.1f (lower is better, under 30 is good)' % msg.fitness)
                               if ok else 'status %d' % msg.cal_status}
                self.note('compass calibration %s' % ('SUCCESS - reboot the drone to use it'
                          if ok else 'FAILED - move away from metal and retry'))
            elif t == 'STATUSTEXT':
                self.note(msg.text)
                if msg.text.startswith(('PreArm:', 'Arm:')):
                    self._prearm_seen[msg.text.split(':', 1)[1].strip()] = time.time()
                low = msg.text.lower()
                if 'radio failsafe' in low:          # "Radio Failsafe" / "Radio Failsafe Cleared"
                    s['rc_fs'] = 'cleared' not in low
                txt = msg.text.strip()
                if txt.startswith(('ArduCopter V', 'ArduPlane V', 'ArduRover V')):
                    s['fc_info']['fw'] = txt
                else:
                    mb = re.match(r'^([A-Za-z][A-Za-z0-9_\-]{2,}) [0-9A-Fa-f]{8} [0-9A-Fa-f]{8}', txt)
                    if mb:
                        s['fc_info']['board'] = mb.group(1)

    def wait_until(self, cond, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump(0.1)
            if cond():
                return True
        return False

    def set_mode(self, name, timeout=8):
        # resend until the aircraft is SEEN in the mode, BLE eats packets
        want = self.m.mode_mapping()[name]
        t0, last_send = time.time(), 0.0
        while time.time() - t0 < timeout:
            if time.time() - last_send > 2.0:
                self.m.set_mode(want)
                last_send = time.time()
            self.pump(0.2)
            if self.state['mode'] == name:
                return True
        return False

    def goto(self, lat, lon, alt):
        self.m.mav.set_position_target_global_int_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000, int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0)

    def _dist_m(self, lat, lon):
        s = self.state
        if s['lat'] is None:
            return 1e9
        dn = math.radians(lat - s['lat']) * EARTH_R
        de = math.radians(lon - s['lon']) * EARTH_R * math.cos(math.radians(s['lat']))
        return math.hypot(dn, de)

    def _abort_click(self):
        """During a mission, service RTL/HOLD clicks immediately; note the rest."""
        try:
            cmd = self.cmds.get_nowait()
        except queue.Empty:
            return False
        if cmd[0] in ('rtl', 'hold'):
            self.note(f'mission interrupted by operator {cmd[0].upper()}')
            getattr(self, 'do_' + cmd[0])(*cmd[1:])
            return True
        self.note(f'{cmd[0]} ignored: a mission is running (the return switch on the radio always works)')
        return False

    def _link_alive(self):
        if time.time() - self._last_hb <= 5.0:
            return True
        if self.state['connected']:
            self.state['connected'] = False
            self.note('TELEMETRY LOST: drone off or out of range')
        return False

    def _mission_interrupt(self):
        """True if the mission must stop: operator abort click or dead link."""
        if self._abort_click():
            return True
        if not self._link_alive():
            self.note('mission stands down: no telemetry. The aircraft continues '
                      'on its own (position target held, its failsafes and your '
                      'radio still work). Regain link or take over on the radio.')
            return True
        return False

    def _ensure_param(self, name, want):
        """Set an FC parameter if it differs; verified by readback."""
        for _ in range(4):
            self._gcs_hb_tick()
            self.m.mav.param_request_read_send(self.m.target_system, 1,
                                               name.encode(), -1)
            pv = self.m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if pv is None:
                continue
            pid = pv.param_id if isinstance(pv.param_id, str) else pv.param_id.decode()
            if pid.rstrip('\x00') != name:
                continue
            if abs(pv.param_value - want) < 1e-3:
                return True
            self.m.mav.param_set_send(self.m.target_system, 1, name.encode(),
                                      want, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            self.note(f'set {name} = {want:g}')
        return False

    def _upload_fc_mission(self, fac):
        """Upload takeoff -> factory(hold 5 s) -> DROP BALL -> RTL as an
        ONBOARD mission: the aircraft flies AND drops even with no link."""
        m = self.m
        MT = mavutil.mavlink.MAV_MISSION_TYPE_MISSION

        def send_item(seq, cmd, p1, x, y, z, p2=0):
            frame = (mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT if seq
                     else mavutil.mavlink.MAV_FRAME_GLOBAL_INT)
            m.mav.mission_item_int_send(m.target_system, 1, seq, frame, cmd,
                                        0, 1, p1, p2, 0, 0, x, y, z, MT)

        m.mav.mission_count_send(m.target_system, 1, 7, MT)
        t0 = time.time()
        while time.time() - t0 < 20:
            self._gcs_hb_tick()
            msg = m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT',
                                     'MISSION_ACK'], blocking=True, timeout=3)
            if (msg is None or msg.get_srcSystem() != m.target_system
                    or msg.get_srcComponent() != 1):
                continue
            if msg.get_type() == 'MISSION_ACK':
                return msg.type == 0
            seq = msg.seq
            if seq == 0:      # home placeholder, ignored by the FC
                send_item(0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0)
            elif seq == 1:
                send_item(1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                          0, 0, 0, CRUISE_ALT)
            elif seq == 2:    # param1 = hold seconds at the factory
                send_item(2, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 5,
                          int(fac['lat'] * 1e7), int(fac['lon'] * 1e7), CRUISE_ALT)
            elif seq == 3:   # the aircraft ITSELF releases the ball here
                # DO_SET_SERVO: param1 = output number, param2 = PWM
                send_item(3, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                          RELEASE_CH, 0, 0, 0, p2=RELEASE_DROP_PWM)
            elif seq == 4:   # hover 3 s while the ball falls clear
                send_item(4, mavutil.mavlink.MAV_CMD_NAV_DELAY, 3, 0, 0, 0)
            elif seq == 5:   # cut servo pulses: silent for the trip home
                send_item(5, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                          RELEASE_CH, 0, 0, 0, p2=0)
            elif seq == 6:
                send_item(6, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                          0, 0, 0, 0)
        return False

    def _verify_drop_item(self):
        """Read mission item 3 back from the FC and insist it is
        DO_SET_SERVO(RELEASE_CH -> RELEASE_DROP_PWM). Guards against an
        upload that 'succeeded' with the wrong numbers in it."""
        m = self.m
        for _ in range(3):
            m.mav.mission_request_int_send(m.target_system, 1, 3)
            t0 = time.time()
            while time.time() - t0 < 3:
                self._gcs_hb_tick()
                it = m.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=1)
                if (it is None or it.get_srcSystem() != m.target_system
                        or it.get_srcComponent() != 1 or it.seq != 3):
                    continue
                ok = (it.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO
                      and int(it.param1) == RELEASE_CH
                      and int(it.param2) == RELEASE_DROP_PWM)
                self.note('drop step verified on aircraft: servo %d -> %d us %s'
                          % (int(it.param1), int(it.param2), 'OK' if ok else '*** WRONG ***'))
                return ok
        self.note('drop step could not be read back from the aircraft')
        return False

    # ------------- command handlers -------------
    def do_mission(self, factory_id):
        """One click: the WHOLE flight is uploaded into the aircraft and flown
        onboard (AUTO), Bluetooth dropouts cannot stop it."""
        with ESTATE_LOCK:
            fac = next((dict(x) for x in ESTATE['factories']
                        if x['id'] == factory_id), None)
        if fac is None:
            self.note(f'unknown factory {factory_id}')
            return
        s = self.state
        if not self._link_alive():
            self.note('MISSION CANCELLED: no telemetry from the drone')
            return
        if s['armed'] and s['mode'] not in ('GUIDED', 'AUTO'):
            self.note(f'MISSION REFUSED: aircraft is under manual control '
                      f'({s["mode"]}). Land and disarm, or hand over by '
                      'switching the radio to GUIDED (mode switch low).')
            return
        self.note(f'=== MISSION {factory_id}: arm, fly out, return, land ===')
        s['busy'] = f'mission {factory_id}'
        try:
            # the aircraft needs these to fly an AUTO takeoff hands-off
            self._ensure_param('AUTO_OPTIONS', 3)
            self.note('uploading the flight into the aircraft ...')
            if not (self._upload_fc_mission(fac) or self._upload_fc_mission(fac)):
                self.note('MISSION CANCELLED: onboard mission upload failed twice. Click again')
                return
            self.note('onboard mission stored (takeoff -> factory -> DROP -> return)')
            # never trust the upload: read the drop step BACK from the aircraft
            if not self._verify_drop_item():
                self.note('MISSION CANCELLED: the drop step on the aircraft is wrong '
                          'and it would come home with the ball. Click again.')
                return
            self.pump(1.5)   # the blocking phases ate heartbeats, take fresh ones
                             # before any link-health judgement
            # BLACKOUT-PROOF ORDER: enter AUTO first, THEN arm. With
            # AUTO_OPTIONS=3 the takeoff item runs the instant arming
            # completes, nothing must cross the link after motor start
            # (motor start reliably kills the BLE link for several seconds).
            if s['mode'] != 'AUTO' and not self.set_mode('AUTO'):
                self.note('MISSION CANCELLED: AUTO refused. Check the readiness panel')
                return
            if not s['armed']:
                self.note('arming. The mission starts the moment arming completes ...')
                t0 = time.time()
                while not s['armed']:
                    if self._mission_interrupt():
                        return
                    if s['mode'] != 'AUTO':
                        self.note(f'MISSION CANCELLED: mode changed to {s["mode"]} while arming')
                        return
                    if time.time() - t0 > 60:
                        self.note('MISSION CANCELLED: arming refused. See the log for the PreArm reason')
                        return
                    self.m.mav.command_long_send(
                        self.m.target_system, self.m.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                        1, 0, 0, 0, 0, 0, 0)
                    self.pump(2.0)
                self.note('armed')
            self.note('*** ONBOARD MISSION RUNNING: the aircraft flies it to the '
                      'end even if Bluetooth drops. Radio always overrides. ***')

            # from here we only WATCH. Link loss is reported, never fatal.
            t0, last_lost_note, last_dist_note = time.time(), 0.0, 0.0
            max_alt, armed_at, start_nudge = 0.0, time.time(), 0.0
            while time.time() - t0 < 900:
                if self._abort_click():
                    return
                if not self._link_alive():
                    if time.time() - last_lost_note > 30:
                        self.note('link down. The mission continues onboard, watching '
                                  'for the aircraft to come back into range')
                        last_lost_note = time.time()
                    self.pump(1.0)
                    continue
                max_alt = max(max_alt, s['alt'])
                if not s['armed']:
                    if max_alt > 5.0:
                        self.note('=== MISSION COMPLETE: landed and disarmed ===')
                        # the mission always releases at the factory: whatever
                        # telemetry we caught, the dropper is open now
                        self.release_state = 'open'
                    else:
                        self.note(f'aircraft disarmed WITHOUT flying (max {max_alt:.1f} m) '
                                  '. The mission did not run; check the log and click again')
                    return
                # armed in AUTO but not climbing: nudge the mission engine
                # (MISSION_START resumes, safe to repeat)
                if (s['mode'] == 'AUTO' and max_alt < 1.0
                        and time.time() - armed_at > 8.0
                        and time.time() - start_nudge > 5.0):
                    self.m.mav.command_long_send(
                        self.m.target_system, self.m.target_component,
                        mavutil.mavlink.MAV_CMD_MISSION_START, 0,
                        0, 0, 0, 0, 0, 0, 0)
                    start_nudge = time.time()
                if s['mode'] not in ('AUTO', 'RTL', 'LAND'):
                    self.note(f'pilot took over ({s["mode"]}). Mission stands down')
                    return
                if time.time() - last_dist_note > 10 and s['lat'] is not None:
                    d = self._dist_m(fac['lat'], fac['lon'])
                    self.note(f'watching: {s["mode"]}, {d:.0f} m from target, '
                              f'{s["alt"]:.0f} m alt')
                    last_dist_note = time.time()
                self.pump(0.5)
            self.note('mission watch ended after 15 min. Check the aircraft')
        finally:
            s['busy'] = ''

    def do_dispatch(self, factory_id):
        with ESTATE_LOCK:
            fac = next((dict(x) for x in ESTATE['factories']
                        if x['id'] == factory_id), None)
        if fac is None:
            self.note(f'unknown factory {factory_id}')
            return
        self.state['busy'] = f'dispatching to {factory_id}'
        if self.state['mode'] != 'GUIDED' and not self.set_mode('GUIDED'):
            self.note('GUIDED refused. Dispatch cancelled')
            self.state['busy'] = ''
            return
        if not self.state['armed']:
            self.note('arming...')
            t0 = time.time()
            while time.time() - t0 < 60 and not self.state['armed']:
                self.m.mav.command_long_send(
                    self.m.target_system, self.m.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1, 0, 0, 0, 0, 0, 0)
                self.pump(2.0)
            if not self.state['armed']:
                self.note('ARM FAILED (prearm?)')
                self.state['busy'] = ''
                return
        if self.state['alt'] < 3:
            self.note(f'takeoff to {CRUISE_ALT:.0f} m')
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, CRUISE_ALT)
            if not self.wait_until(lambda: self.state['alt'] > CRUISE_ALT * 0.9, 60):
                self.note('TAKEOFF FAILED. Dispatch cancelled (check the aircraft, it may still be armed)')
                self.state['busy'] = ''
                return
        self.note(f'en route to {factory_id}')
        self.goto(fac['lat'], fac['lon'], CRUISE_ALT)
        self.state['busy'] = ''

    def do_goto(self, lat, lon, alt):
        if not self.state['armed']:
            self.note('goto ignored: not armed/flying')
            return
        if self.state['mode'] != 'GUIDED':
            self.set_mode('GUIDED')
        self.note(f'flying to clicked point ({alt:.0f} m)')
        self.goto(lat, lon, alt)

    def do_nudge(self, dn, de, dd):
        s = self.state
        if not s['armed'] or s['lat'] is None:
            return
        if s['mode'] != 'GUIDED':
            self.set_mode('GUIDED')
        lat, lon = latlon_add_m(s['lat'], s['lon'], dn, de)
        alt = max(4.0, s['alt'] - dd)
        self.note(f'nudge n{dn:+.0f} e{de:+.0f} d{dd:+.0f}')
        self.goto(lat, lon, alt)

    def do_hold(self):
        s = self.state
        if s['armed'] and s['lat'] is not None:
            if s['mode'] != 'GUIDED':
                self.set_mode('GUIDED')
            self.goto(s['lat'], s['lon'], max(4.0, s['alt']))
            self.note('holding position')

    def do_rtl(self):
        if self.set_mode('RTL'):
            self.note('returning to launch')
        else:
            self.note('RTL commanded but NOT confirmed. Check the aircraft; '
                      'ch11 on the radio always works')

    def do_calgyro(self):
        """On-demand gyro calibration (fixes 'Gyros not calibrated' after a
        power-up in motion). Drone must be disarmed and PERFECTLY STILL."""
        if not self._link_alive():
            self.note('gyro cal: no telemetry from the drone')
            return
        if self.state['armed']:
            self.note('gyro cal REFUSED: aircraft is armed')
            return
        self.note('GYRO CALIBRATION: keep the drone perfectly still (about 10 s) ...')
        self._last_ack = None
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
            1, 0, 0, 0, 0, 0, 0)          # param1 = 1: gyro only
        t0 = time.time()
        while time.time() - t0 < 12:
            ack = self._last_ack
            if ack and ack[0] == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                self.note('gyro calibration %s'
                          % ('DONE' if ack[1] == 0 else f'REJECTED (result {ack[1]})'))
                break
            self.pump(0.4)
        else:
            self.note('gyro calibration: no acknowledgement. Check the log')
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     401, 0, 0, 0, 0, 0, 0, 0, 0)

    def _set_release(self, pwm, label):
        """Drive the release servo; verify for 3 s, then accept and stop.

        No capacitor on the servo rail, so we never hammer commands:
        at most 3 sends inside a 3 s window. The FC holds the PWM on
        the pin continuously either way, so after the window we accept
        the position and simply say what the pin read."""
        self._release_raw = None
        t0 = time.time()
        while time.time() - t0 < 3.0:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                RELEASE_CH, pwm, 0, 0, 0, 0, 0)
            self.pump(1.0)
            if self._release_raw == pwm:
                self.note('%s: servo %d confirmed at %d us' % (label, RELEASE_CH, pwm))
                return True
        self.note('%s: commanded %d us, pin read %s after 3 s; accepting position '
                  '(no-retry policy)' % (label, pwm, self._release_raw))
        return True

    def _relax_release(self):
        """Stop pulses so the servo goes limp and silent. The pin is held
        by the lug geometry, not servo torque, so idle buzz is pure waste."""
        self.pump(1.5)                       # let it finish reaching position
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            RELEASE_CH, 0, 0, 0, 0, 0, 0)
        self.note('release servo relaxed (silent; the mechanism holds the position)')

    def do_drop(self):
        self.note('>>> BALL RELEASE <<<')
        self._set_release(RELEASE_DROP_PWM, 'RELEASED')
        self.release_state = 'open'
        self._relax_release()

    def do_lock(self):
        self._set_release(RELEASE_LOCK_PWM, 'LOCKED')
        self.release_state = 'locked'
        self._relax_release()

    def do_servo(self, pwm):
        """Bench-only: park the release servo at any PWM (Setup page slider)."""
        if self.state['armed']:
            self.note('servo test refused: aircraft is armed')
            return
        self._set_release(int(pwm), 'SERVO')
        self._relax_release()            # 1.5 s to get there, then silent - the mechanism holds the pin

    MOTOR_IDX = {'A': 1, 'B': 2, 'C': 3, 'D': 4}   # ArduPilot motor-test order

    def do_motor_test(self, motor, pct, secs):
        """Bench-only, the same command Mission Planner's Motor Test uses:
        the FC spins one motor (or all four in A-B-C-D order) at a low
        percentage while DISARMED. The web page gates it behind an explicit
        props-off / props-on-1% acknowledgement."""
        if self.state['armed'] and time.time() > self.motor_test_until:
            self.note('motor test refused: aircraft is armed (and no motor test is running)')
            return
        seq = 'A>B>C>D' if motor == 'ALL' else motor
        idx = 1 if motor == 'ALL' else self.MOTOR_IDX[motor]
        count = 4 if motor == 'ALL' else 0
        self._last_ack = None
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
            idx, 0, float(pct), float(secs), count, 0, 0)
        # the FC reports ARMED while the test runs - remember until when that is expected
        self.motor_test_until = time.time() + float(secs) * (4 if motor == 'ALL' else 1) + 2.5
        self.note('motor test: %s at %g%% for %g s ...' % (seq, pct, secs))
        t0 = time.time()
        while time.time() - t0 < 3:
            self.pump(0.2)
            ack = self._last_ack
            if ack and ack[0] == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
                self.note('motor test %s' % ('ACCEPTED by the FC' if ack[1] == 0
                          else 'REFUSED (result %d) - read the FC message above' % ack[1]))
                return
        self.note('motor test: no acknowledgement from the FC')

    # ---- parameters (bench) ----
    BENCH_PARAM_PREFIXES = ('SERVO', 'RC', 'FLTMODE', 'FRAME_', 'MOT_', 'RCMAP_', 'BATT', 'COMPASS_',
                            'INS_', 'FS_', 'RTL_', 'FENCE_', 'ATC_', 'BRD_SAFETY', 'SERIAL', 'GPS',
                            'ARMING_', 'AUTO_OPTIONS', 'LAND_', 'WPNAV_', 'PILOT_', 'LOG_', 'OSD',
                            'VTX_', 'RALLY_', 'EK3_', 'AHRS_', 'RALLY', 'NTF_', 'AUTOTUNE_', 'ESC_')

    def _param_get(self, name, timeout=2.0):
        self._param_wait.pop(name, None)
        self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component,
                                           name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump(0.05)
            if name in self._param_wait:
                return self._param_wait[name]
        return None

    def _param_set(self, name, value):
        for _ in range(3):
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                      name.encode(), float(value),
                                      mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            self.pump(0.3)
            v = self._param_get(name)
            if v is not None and abs(v - float(value)) < 1e-3:
                return True
        return False

    def do_get_params(self, names):
        got = 0
        for n in names[:40]:
            if self._param_get(str(n)) is not None:
                got += 1
        self.note('read %d/%d parameters' % (got, len(names[:40])))

    def do_set_params(self, params):
        if self.state['armed'] and time.time() < self.motor_test_until:
            self.note('waiting for the motor test to finish before writing ...')
            while self.state['armed'] and time.time() < self.motor_test_until + 2.0:
                self.pump(0.2)
        if self.state['armed']:
            self.note('parameter write refused: aircraft is armed')
            return
        for name, value in params.items():
            if not str(name).startswith(self.BENCH_PARAM_PREFIXES):
                self.note('%s: not a setup parameter, skipped' % name)
                continue
            ok = self._param_set(name, value)
            self.note('%s = %s %s' % (name, value, '(verified)' if ok else '*** FAILED ***'))

    def do_reboot(self):
        if self.state['armed']:
            self.note('reboot refused: aircraft is armed')
            return
        self.note('rebooting the flight controller ...')
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                     1, 0, 0, 0, 0, 0, 0)
        self.pump(1.0)

    def do_banner(self):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SEND_BANNER, 0, 0, 0, 0, 0, 0, 0, 0)
        self.pump(1.5)

    # ---- calibrations (bench) ----
    def _cal_cmd(self, label, cmd, *params):
        if self.state['armed']:
            self.note('%s refused: aircraft is armed' % label)
            return
        self._last_ack = None
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)
        t0 = time.time()
        while time.time() - t0 < 4:
            self.pump(0.2)
            ack = self._last_ack
            if ack and ack[0] == cmd:
                self.note('%s: %s' % (label, 'accepted' if ack[1] == 0 else 'REFUSED (result %d)' % ack[1]))
                return
        self.note('%s: sent (no acknowledgement)' % label)

    def do_accelcal_start(self):
        self.state['params'].pop('_accelcal_step', None)
        self._cal_cmd('accelerometer calibration (6 positions)',
                      mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0)

    def do_accelcal_pos(self, pos):
        self._cal_cmd('position %d recorded' % int(pos),
                      mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS, int(pos))

    def do_levelcal(self):
        self._cal_cmd('level calibration', mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                      0, 0, 0, 0, 2, 0, 0)

    def do_magcal_start(self):
        self.state['magcal'] = {'pct': 0, 'status': 'running', 'report': None}
        self._cal_cmd('compass calibration started - rotate the drone slowly around every axis',
                      mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL, 0, 1, 1, 0, 0)

    def do_magcal_cancel(self):
        self._cal_cmd('compass calibration cancelled', mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL, 0)
        self.state['magcal'] = {'pct': 0, 'status': 'cancelled', 'report': None}

    def do_magcal_accept(self):
        self._cal_cmd('compass calibration accepted', mavutil.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL, 0)

    def do_magcal_yaw(self):
        self._cal_cmd('quick compass calibration (drone pointing NORTH)',
                      mavutil.mavlink.MAV_CMD_FIXED_MAG_CAL_YAW, 0, 0, 0, 0)

    def do_servo_off(self):
        if self.state['armed']:
            return
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            RELEASE_CH, 0, 0, 0, 0, 0, 0)
        self.note('release servo relaxed (signal off)')

    # ------------- main loop -------------
    def run(self):
        nolink_noted = None
        while True:
            try:
                if self.paused:
                    time.sleep(0.3)
                    continue
                if self.resolver:
                    link, baud, kind = self.resolver()
                    self.state['link_kind'], self.state['link'] = kind, link
                    if not link:
                        if nolink_noted != kind:
                            self.note('no %s link yet: %s' % (kind, {
                                'usb': 'plug the flight controller into USB',
                                'radio': 'plug the telemetry radio into USB'}.get(kind, '')))
                            nolink_noted = kind
                        time.sleep(3)
                        continue
                    nolink_noted = None
                    self.link, self.baud = link, baud
                self.m = mavutil.mavlink_connection(self.link, baud=self.baud,
                                                    source_system=255)
                hb = None
                for _ in range(10):
                    self.m.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    hb = self.m.wait_heartbeat(timeout=3)
                    if hb:
                        break
                if hb is None:
                    raise RuntimeError('no heartbeat')
                self.m.target_component = 1   # the autopilot, never a peripheral
                for msg_id, hz in ((33, 5), (74, 2), (24, 1), (1, 1), (36, 2), (65, 4),
                                   (30, 3), (241, 1), (193, 1)):
                    self.m.mav.command_long_send(
                        self.m.target_system, self.m.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
                self.m.mav.command_long_send(          # firmware + board name lines
                    self.m.target_system, self.m.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SEND_BANNER, 0, 0, 0, 0, 0, 0, 0, 0)
                self.state['connected'] = True
                self.note('link up')
                self._last_hb = time.time()
                last_hb = 0.0
                last_prearm_poll = 0.0
                last_cable_check = 0.0
                while True:
                    if self.paused or self.relink:
                        raise _LinkReleased('paused' if self.paused else 'relink')
                    if (self.resolver and time.time() - last_cable_check > 3.0
                            and not self.state['armed']):
                        last_cable_check = time.time()
                        kind, cable = self.state.get('link_kind'), usb_board_port()
                        if SETTINGS.get('link', 'auto') == 'auto' and kind == 'bluetooth' and cable:
                            self.note('USB cable plugged in: switching the bench link to the cable (%s)' % cable)
                            raise _LinkReleased('relink')
                        if kind == 'usb' and not cable:
                            self.note('USB cable pulled: switching to Bluetooth')
                            raise _LinkReleased('relink')
                    # bridge TCP can stay up while the drone is gone, judge the
                    # link by the vehicle's heartbeat, not the socket
                    if (self.state['connected']
                            and time.time() - self._last_hb > 5.0):
                        self.state['connected'] = False
                        self.note('TELEMETRY LOST: drone off or out of range')
                    # while grounded and not ready, ask the FC every 20 s to
                    # restate its prearm objections so the panel stays current
                    if (not self.state['armed']
                            and self.state['sensors'].get('prearm') is False
                            and time.time() - last_prearm_poll > 20.0):
                        self.m.mav.command_long_send(
                            self.m.target_system, self.m.target_component,
                            401, 0, 0, 0, 0, 0, 0, 0, 0)   # RUN_PREARM_CHECKS
                        last_prearm_poll = time.time()
                    try:
                        cmd = self.cmds.get_nowait()
                    except queue.Empty:
                        cmd = None
                    if cmd:
                        try:
                            getattr(self, 'do_' + cmd[0])(*cmd[1:])
                        except Exception as e:
                            self.note(f'command {cmd[0]} FAILED: {e}')
                            self.state['busy'] = ''
                    self.pump(0.1)
            except _LinkReleased as e:
                self.state['connected'] = False
                try:
                    self.m.close()
                except Exception:
                    pass
                self.m = None
                if str(e) == 'paused':
                    self.note('link released for the USB tool - it reconnects when the tool finishes')
                    while self.paused:
                        time.sleep(0.3)
                else:
                    self.relink = False
                    self.note('reconnecting with the new link settings ...')
            except Exception as e:
                self.state['connected'] = False
                try:
                    self.m.close()
                except Exception:
                    pass
                self.m = None
                self.note(f'link lost: {e}')
                time.sleep(3)


app = Flask(__name__)
drone = None  # set in main
DROP_ARM = {'until': 0.0}   # server-side two-step drop confirmation


PAGES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pages')


def _page(name):
    with open(os.path.join(PAGES, name), encoding='utf-8') as f:
        return Response(f.read(), mimetype='text/html')


@app.route('/')
def index():
    return _page('portal.html')


@app.route('/ops')
def ops_page():
    return _page('ops.html')


@app.route('/setup')
def setup_page():
    return _page('setup.html')


@app.route('/requirements')
def requirements_page():
    return _page('requirements.html')


@app.route('/api/env')
def api_env():
    """Read-only environment check for the Requirements/Setup pages."""
    import importlib
    import platform
    import socket
    pk = []
    for name, mod in (('pymavlink', 'pymavlink'), ('pyserial', 'serial'),
                      ('flask', 'flask'), ('bleak', 'bleak'),
                      ('pyusb', 'usb'), ('libusb-package', 'libusb_package')):
        try:
            m = importlib.import_module(mod)
            pk.append({'name': name, 'ok': True,
                       'version': getattr(m, '__version__', '?')})
        except Exception:
            pk.append({'name': name, 'ok': False, 'version': None})
    mp = None
    for c in (os.path.join(os.environ.get('ProgramFiles(x86)', ''), 'Mission Planner', 'MissionPlanner.exe'),
              os.path.join(os.environ.get('ProgramFiles', ''), 'Mission Planner', 'MissionPlanner.exe'),
              os.path.join(os.path.expanduser('~'), 'Documents', 'Mission Planner', 'MissionPlanner.exe'),
              os.path.join(ROOT, 'tools', 'MissionPlanner', 'MissionPlanner.exe')):
        if c and os.path.exists(c):
            mp = c
            break
    fc = {'kind': None, 'port': None}
    try:
        from serial.tools import list_ports
        for pinfo in list_ports.comports():
            if pinfo.vid == 0x1209:
                fc = {'kind': 'ardupilot', 'port': pinfo.device}
                break
            if pinfo.vid == 0x0483:
                fc = {'kind': 'stm32', 'port': pinfo.device}
        if fc['kind'] == 'stm32':
            sys.path.insert(0, os.path.join(ROOT, 'fc'))
            import bf_migrate
            import bf_msp
            info = bf_msp.probe(fc['port'])
            if info:
                fc.update(info)
                fc['kind'] = 'betaflight'
                boards, _ = copter_boards()
                ap, how = bf_migrate.ap_board_for(info.get('target') or '', boards)
                fc['ardupilot_board'] = ap
                fc['ardupilot_supported'] = bool(ap)
    except Exception:
        pass

    def can_connect(host, port, t=1.5):
        try:
            socket.create_connection((host, port), timeout=t).close()
            return True
        except Exception:
            return False
    py = sys.version_info
    return jsonify({
        'os': platform.system() + ' ' + platform.release(),
        'python': {'version': '%d.%d.%d' % py[:3], 'ok': py >= (3, 10)},
        'packages': pk,
        'mission_planner': mp,
        'fc': fc,
        'bridge': can_connect('127.0.0.1', 5770),
        'internet': can_connect('server.arcgisonline.com', 443, 2.5),
    })


# ---- one-click setup jobs: run a whitelisted fc/ tool over USB and stream its output ----
SETUP_TOOLS = {
    'backup_betaflight': ['fc/backup_betaflight.py'],
    'push_params': ['fc/push_params.py', 'fc/speedybee-f405v5-base.param'],
    'snapshot': ['fc/param_snapshot.py'],
    'rc_check': ['fc/rc_check.py', '--seconds', '8'],
    'rc_record': ['fc/rc_record.py'],
    'rc_write_cal': ['fc/rc_write_cal.py'],
    'bench_cal': ['fc/bench_cal.py'],
    'magcal': ['fc/field_magcal.py', '--no-confirm'],
    'prearm': ['fc/prearm_check.py'],
    'flash': ['fc/dfu_flash.py', 'fc/firmware/arducopter_with_bl.hex'],   # path fixed up per board below
    'servo_test': ['fc/servo_test.py'],
    'servo_open': ['fc/release.py', 'open'],
    'servo_close': ['fc/release.py', 'close'],
}
SETUP_JOB = {'lock': threading.Lock(), 'proc': None, 'tool': None,
             'lines': [], 'done': True, 'rc': None}


def _setup_reader(proc):
    try:
        for line in proc.stdout:
            SETUP_JOB['lines'].append(line.rstrip('\n'))
            if len(SETUP_JOB['lines']) > 600:
                del SETUP_JOB['lines'][:200]
        proc.wait()
        SETUP_JOB['rc'] = proc.returncode
    finally:
        SETUP_JOB['done'] = True
        drone.paused = False          # give the USB port back to the live link


@app.route('/api/setup/run', methods=['POST'])
def api_setup_run():
    if request.headers.get('Sec-Fetch-Site', '') == 'cross-site':
        return jsonify({'ok': False, 'err': 'cross-site request refused'}), 403
    tool = (request.get_json(force=True) or {}).get('tool')
    if tool not in SETUP_TOOLS:
        return jsonify({'ok': False, 'err': 'unknown tool'}), 400
    args = list(SETUP_TOOLS[tool])
    if tool == 'push_params':
        args[1] = ('fc/speedybee-f405v5-base.param'
                   if SETTINGS.get('board') == 'speedybeef4v5'
                   else 'fc/generic-copter-base.param')
    if tool == 'flash':
        board = re.sub(r'[^A-Za-z0-9_\-]', '', SETTINGS.get('board') or '')
        hexpath = os.path.join(ROOT, 'fc', 'firmware', board, 'arducopter_with_bl.hex')
        if not board or not os.path.exists(hexpath):
            return jsonify({'ok': False, 'err': 'no firmware downloaded for board "%s" - click "Download firmware" first' % board}), 400
        args[1] = os.path.relpath(hexpath, ROOT)
    return _start_job(tool, args)


def _start_job(label, args):
    with SETUP_JOB['lock']:
        if not SETUP_JOB['done']:
            return jsonify({'ok': False, 'err': 'another step is still running'}), 409
        if drone.state['armed']:
            return jsonify({'ok': False, 'err': 'aircraft is armed - USB tools are for the bench'}), 409
        if drone.state.get('link_kind') == 'usb' and drone.m is not None:
            # the tool needs the same COM port the live link holds: release it first
            drone.paused = True
            for _ in range(60):
                if drone.m is None:
                    break
                time.sleep(0.1)
        env = dict(os.environ, PYTHONIOENCODING='utf-8', PYTHONUNBUFFERED='1')
        try:
            proc = subprocess.Popen([sys.executable, '-u'] + args, cwd=ROOT,
                                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True,
                                    encoding='utf-8', errors='replace', env=env)
        except Exception as e:
            drone.paused = False
            return jsonify({'ok': False, 'err': 'could not start the tool: %s' % e}), 500
        SETUP_JOB.update(proc=proc, tool=label, done=False, rc=None,
                         lines=['$ python ' + ' '.join(args)])
        threading.Thread(target=_setup_reader, args=(proc,), daemon=True).start()
    return jsonify({'ok': True})


@app.route('/api/setup/status')
def api_setup_status():
    return jsonify({'tool': SETUP_JOB['tool'], 'lines': SETUP_JOB['lines'],
                    'done': SETUP_JOB['done'], 'rc': SETUP_JOB['rc']})


@app.route('/api/setup/stop', methods=['POST'])
def api_setup_stop():
    p = SETUP_JOB['proc']
    if p and not SETUP_JOB['done']:
        p.kill()
        SETUP_JOB['lines'].append('[stopped by operator]')
    return jsonify({'ok': True})


# ---- site / connection / board settings ----
@app.route('/api/settings')
def api_settings_get():
    return jsonify(SETTINGS)


@app.route('/api/settings', methods=['POST'])
def api_settings_post():
    if request.headers.get('Sec-Fetch-Site', '') == 'cross-site':
        return jsonify({'ok': False, 'err': 'cross-site request refused'}), 403
    d = request.get_json(force=True) or {}
    try:
        if 'site' in d:
            st = d['site']
            SETTINGS['site'] = None if st is None else {
                'name': str(st.get('name') or 'My site')[:80],
                'lat': float(st['lat']), 'lon': float(st['lon']),
                'zoom': max(3, min(19, int(st.get('zoom', 16))))}
        if 'board' in d:
            b = str(d['board']).strip()
            if not b.replace('_', '').replace('-', '').isalnum():
                raise ValueError('bad board id')
            SETTINGS['board'] = b
        relink = False
        if 'link' in d:
            if d['link'] not in ('auto', 'ble', 'usb', 'radio'):
                raise ValueError('bad link')
            if d['link'] != SETTINGS.get('link') and drone.state['armed']:
                return jsonify({'ok': False, 'err': 'aircraft is armed - change the link after landing'}), 409
            relink = d['link'] != SETTINGS.get('link')
            SETTINGS['link'] = d['link']
        if 'ble_name' in d:
            SETTINGS['ble_name'] = str(d['ble_name'])[:32]
        if 'ble_addr' in d:
            a = str(d['ble_addr'] or '').strip()
            if a and not re.fullmatch(r'[0-9A-Fa-f]{2}(:[0-9A-Fa-f]{2}){5}', a):
                raise ValueError('bad Bluetooth address')
            SETTINGS['ble_addr'] = a.upper()
        if 'release' in d:
            r = d['release']
            ch = int(r.get('ch', SETTINGS['release']['ch']))
            dp = int(r.get('drop_pwm', SETTINGS['release']['drop_pwm']))
            lp = int(r.get('lock_pwm', SETTINGS['release']['lock_pwm']))
            if not (1 <= ch <= 16 and 800 <= dp <= 2500 and 800 <= lp <= 2500 and dp != lp):
                raise ValueError('release values out of range')
            SETTINGS['release'] = {'ch': ch, 'drop_pwm': dp, 'lock_pwm': lp}
        if 'cruise_alt' in d:
            ca = float(d['cruise_alt'])
            if not 5 <= ca <= 120:
                raise ValueError('cruise altitude must be 5-120 m')
            SETTINGS['cruise_alt'] = ca
    except (KeyError, ValueError, TypeError) as e:
        return jsonify({'ok': False, 'err': str(e)}), 400
    apply_settings()
    save_settings()
    if relink and drone.resolver:
        drone.relink = True           # the link loop drops and reconnects with the new setting
    return jsonify({'ok': True, 'settings': SETTINGS})


@app.route('/api/geocode')
def api_geocode():
    """Place-name search (OpenStreetMap Nominatim) so a new site can be found by name."""
    import urllib.parse
    import urllib.request
    q = (request.args.get('q') or '').strip()
    if not q:
        return jsonify([])
    url = 'https://nominatim.openstreetmap.org/search?format=json&limit=6&q=' + urllib.parse.quote(q)
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'FireFightingDronePortal/1.0'})
        with urllib.request.urlopen(req, timeout=8) as r:
            hits = json.loads(r.read().decode('utf-8'))
        return jsonify([{'name': h.get('display_name', ''), 'lat': float(h['lat']), 'lon': float(h['lon'])}
                        for h in hits])
    except Exception as e:
        return jsonify({'error': 'search failed: %s' % e}), 502


TILE_DIR = os.path.join(ROOT, 'console', 'tilecache')


@app.route('/tiles/<int:z>/<int:x>/<int:y>')
def api_tile(z, x, y):
    """Satellite tiles via a disk cache: once a site has been viewed online, the
    map keeps working with no internet at the field."""
    import urllib.request
    if not (0 <= z <= 19):
        return Response(status=404)
    fp = os.path.join(TILE_DIR, str(z), str(x), '%d.jpg' % y)
    if not os.path.exists(fp):
        try:
            url = ('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/'
                   'MapServer/tile/%d/%d/%d' % (z, y, x))
            req = urllib.request.Request(url, headers={'User-Agent': 'FireFightingDronePortal/1.0'})
            with urllib.request.urlopen(req, timeout=10) as r:
                data = r.read()
            os.makedirs(os.path.dirname(fp), exist_ok=True)
            with open(fp, 'wb') as f:
                f.write(data)
        except Exception:
            return Response(status=504)
    with open(fp, 'rb') as f:
        return Response(f.read(), mimetype='image/jpeg',
                        headers={'Cache-Control': 'public, max-age=2592000'})


@app.route('/api/dfu')
def api_dfu():
    """Is a board sitting in STM32 DFU (bootloader) mode? Plus every serial port, for the flash step."""
    dfu = False
    try:
        out = subprocess.run(
            ['powershell', '-NoProfile', '-Command',
             "Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | "
             "Where-Object { $_.InstanceId -like 'USB\\VID_0483&PID_DF11*' } | "
             "Select-Object -ExpandProperty InstanceId"],
            capture_output=True, text=True, timeout=15).stdout
        dfu = 'VID_0483' in out
    except Exception:
        pass
    ports = []
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            ports.append({'port': p.device, 'desc': p.description,
                          'vid': '%04x' % p.vid if p.vid else None,
                          'kind': 'ardupilot' if p.vid == 0x1209 else ('stm32' if p.vid == 0x0483 else None)})
    except Exception:
        pass
    return jsonify({'dfu': dfu, 'ports': ports})


@app.route('/api/setup/firmware', methods=['POST'])
def api_setup_firmware():
    """Download the right ArduCopter firmware for the configured board into fc/firmware/."""
    import urllib.request
    if request.headers.get('Sec-Fetch-Site', '') == 'cross-site':
        return jsonify({'ok': False, 'err': 'cross-site request refused'}), 403
    board = ((request.get_json(force=True) or {}).get('board') or SETTINGS.get('board') or '').strip()
    if not board or not board.replace('_', '').replace('-', '').isalnum():
        return jsonify({'ok': False, 'err': 'bad board id'}), 400
    url = 'https://firmware.ardupilot.org/Copter/stable/%s/arducopter_with_bl.hex' % board
    dest = os.path.join(ROOT, 'fc', 'firmware', board, 'arducopter_with_bl.hex')
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'FireFightingDronePortal/1.0'})
        with urllib.request.urlopen(req, timeout=60) as r:
            data = r.read()
        if len(data) < 100000 or not data.lstrip().startswith(b':'):
            return jsonify({'ok': False, 'err': 'server returned something that is not a firmware hex'}), 502
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        with open(dest, 'wb') as f:
            f.write(data)
    except Exception as e:
        msg = str(e)
        if '404' in msg:
            msg = 'no ArduCopter firmware for board id "%s" - check the id on firmware.ardupilot.org/Copter/stable' % board
        return jsonify({'ok': False, 'err': msg}), 502
    return jsonify({'ok': True, 'path': dest, 'bytes': len(data), 'url': url})


# ---- which boards can run ArduPilot (official firmware manifest, cached) ----
BOARDS_CACHE = os.path.join(ROOT, 'fc', 'firmware', 'copter-boards.json')
BOARDS_OFFLINE = os.path.join(ROOT, 'fc', 'ardupilot-copter-boards.json')


def copter_boards(refresh=False):
    try:
        if not refresh and os.path.exists(BOARDS_CACHE) and time.time() - os.path.getmtime(BOARDS_CACHE) < 7 * 86400:
            return json.load(open(BOARDS_CACHE, encoding='utf-8')), 'manifest (cached)'
    except Exception:
        pass
    try:
        import gzip
        import urllib.request
        req = urllib.request.Request('https://firmware.ardupilot.org/manifest.json.gz',
                                     headers={'User-Agent': 'FireFightingDronePortal/1.0'})
        raw = urllib.request.urlopen(req, timeout=60).read()
        fw = json.loads(gzip.decompress(raw))['firmware']
        boards = sorted({f['platform'] for f in fw
                         if f.get('vehicletype') == 'Copter' and f.get('mav-firmware-version-type') == 'OFFICIAL'})
        os.makedirs(os.path.dirname(BOARDS_CACHE), exist_ok=True)
        json.dump(boards, open(BOARDS_CACHE, 'w', encoding='utf-8'))
        return boards, 'manifest (fresh)'
    except Exception:
        return json.load(open(BOARDS_OFFLINE, encoding='utf-8')), 'offline copy'


@app.route('/api/boards')
def api_boards():
    boards, src = copter_boards(refresh=request.args.get('refresh') == '1')
    return jsonify({'boards': boards, 'source': src, 'count': len(boards)})


@app.route('/api/board_serials')
def api_board_serials():
    """SERIAL_ORDER of an ArduPilot board (which UART is SERIAL1, SERIAL2, ...), from the cached hwdef."""
    board = (request.args.get('board') or SETTINGS.get('board') or '').strip()
    if not re.fullmatch(r'[A-Za-z0-9_\-]{1,40}', board):
        return jsonify({'ok': False, 'board': board, 'order': []})
    sys.path.insert(0, os.path.join(ROOT, 'fc'))
    import bf_migrate
    order = bf_migrate.hwdef_serial_order(board)
    return jsonify({'ok': order is not None, 'board': board, 'order': order or []})


@app.route('/api/bf_board')
def api_bf_board():
    """Betaflight board name -> ArduPilot board id + can it run ArduPilot at all."""
    sys.path.insert(0, os.path.join(ROOT, 'fc'))
    import bf_migrate
    name = (request.args.get('name') or '').strip()
    boards, _ = copter_boards()
    ap, how = bf_migrate.ap_board_for(name, boards)
    note = ''
    if 'F411' in name.upper():
        note = 'F411 boards have very little flash: ArduPilot support is partial or missing - prefer an F405/F7/H7 board.'
    return jsonify({'betaflight': name, 'ardupilot': ap, 'supported': bool(ap), 'how': how, 'note': note})


def _latest_bf_diff():
    base = os.path.join(ROOT, 'fc', 'backups')
    cands = []
    if os.path.isdir(base):
        for d in os.listdir(base):
            fp = os.path.join(base, d, 'diff_all.txt')
            if d.startswith('betaflight-') and os.path.exists(fp):
                cands.append(fp)
    return max(cands, key=os.path.getmtime) if cands else None


@app.route('/api/setup/migrate')
def api_setup_migrate():
    """Translate the newest Betaflight backup into ArduPilot parameters (preview)."""
    sys.path.insert(0, os.path.join(ROOT, 'fc'))
    import bf_migrate
    fp = _latest_bf_diff()
    if not fp:
        return jsonify({'ok': False, 'err': 'no Betaflight backup found - run "Back up Betaflight" first'}), 404
    text = open(fp, encoding='utf-8', errors='replace').read()
    boards, _ = copter_boards()
    res = bf_migrate.translate(text, SETTINGS.get('board'), boards)
    res.update(ok=True, source=os.path.relpath(fp, ROOT))
    return jsonify(res)


@app.route('/api/setup/migrate', methods=['POST'])
def api_setup_migrate_apply():
    if request.headers.get('Sec-Fetch-Site', '') == 'cross-site':
        return jsonify({'ok': False, 'err': 'cross-site request refused'}), 403
    pv = (request.get_json(force=True) or {}).get('params') or {}
    if not pv:
        return jsonify({'ok': False, 'err': 'nothing selected'}), 400
    out = os.path.join(ROOT, 'fc', 'generated')
    os.makedirs(out, exist_ok=True)
    fp = os.path.join(out, 'bf-migration.param')
    clean = {str(k): float(v) for k, v in pv.items() if re.fullmatch(r'[A-Z0-9_]{1,16}', str(k))}
    with open(fp, 'w', encoding='utf-8') as f:      # kept as a record of what was applied
        f.write('# generated by the portal from the Betaflight backup\n')
        for k, v in clean.items():
            f.write('%s,%g\n' % (k, v))
    if drone.state['armed']:
        return jsonify({'ok': False, 'err': 'aircraft is armed'}), 409
    if not drone.state['connected']:
        return jsonify({'ok': False, 'err': 'drone not linked - power it and wait for LINK OK'}), 409
    drone.cmds.put(('set_params', clean))     # written + verified over the console link (Bluetooth or USB)
    return jsonify({'ok': True, 'count': len(clean), 'file': os.path.relpath(fp, ROOT)})


ESTATE_LOCK = threading.Lock()


@app.route('/api/estate')
def api_estate():
    with ESTATE_LOCK:
        snap = json.dumps(ESTATE)
    return Response(snap, mimetype='application/json')


def save_estate():
    tmp = ESTATE_PATH + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(ESTATE, f, ensure_ascii=False, indent=2)
    os.replace(tmp, ESTATE_PATH)


@app.route('/api/estate', methods=['POST'])
def api_estate_edit():
    if request.headers.get('Sec-Fetch-Site', '') == 'cross-site':
        return jsonify({'ok': False, 'err': 'cross-site request refused'}), 403
    d = request.get_json(force=True)
    op = d.get('op')
    with ESTATE_LOCK:
        facs = ESTATE['factories']
        if op == 'add':
            nums = [int(x['id'][1:]) for x in facs
                    if len(x['id']) > 1 and x['id'][1:].isdigit()]
            fid = 'F%02d' % (max(nums) + 1 if nums else 1)
            facs.append({'id': fid, 'name': d.get('name') or fid,
                         'lat': float(d['lat']), 'lon': float(d['lon'])})
        elif op == 'delete':
            ESTATE['factories'] = [x for x in facs if x['id'] != d.get('id')]
        elif op == 'move':
            for x in facs:
                if x['id'] == d.get('id'):
                    x['lat'], x['lon'] = float(d['lat']), float(d['lon'])
        elif op == 'rename':
            for x in facs:
                if x['id'] == d.get('id'):
                    x['name'] = d.get('name') or x['name']
        else:
            return jsonify({'ok': False, 'err': 'unknown op'}), 400
        save_estate()
        snap = json.dumps({'ok': True, 'estate': ESTATE})
    return Response(snap, mimetype='application/json')


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    # a browser page from another site can POST here (CSRF), refuse it
    if request.headers.get('Sec-Fetch-Site', '') == 'cross-site':
        return jsonify({'ok': False, 'err': 'cross-site request refused'}), 403
    try:
        return _route_cmd(request.get_json(force=True))
    except (KeyError, TypeError, ValueError) as e:
        return jsonify({'ok': False, 'err': f'bad request: {e}'}), 400


def _bench_blocked():
    """None when bench commands may run; otherwise the reason (a real arm, or a motor test still spinning)."""
    if not drone.state['armed']:
        return None
    if time.time() < drone.motor_test_until:
        return 'motor test still running - try again in a few seconds'
    return 'aircraft is armed'


def _route_cmd(d):
    a = d.get('action')
    if a == 'dispatch':
        drone.cmds.put(('dispatch', d['factory']))
    elif a == 'mission':
        # flying a real aircraft off one POST needs an explicit confirm word
        if d.get('confirm') != 'FLY':
            return jsonify({'ok': False, 'err': 'mission not confirmed'}), 403
        # real missions fly with the ball locked in; anything else needs an
        # explicit operator override (test flights without the ball)
        if drone.state.get('link_kind') == 'usb' or usb_board_port():
            return jsonify({'ok': False, 'err': 'USB cable is plugged into the drone - unplug it; '
                            'missions fly over Bluetooth'}), 409
        if drone.release_state != 'locked' and not d.get('allow_unlocked'):
            return jsonify({'ok': False, 'err': 'release_not_locked',
                            'release': drone.release_state}), 409
        drone.cmds.put(('mission', d['factory']))
    elif a == 'goto':
        drone.cmds.put(('goto', float(d['lat']), float(d['lon']),
                        float(d.get('alt', CRUISE_ALT))))
    elif a == 'nudge':
        drone.cmds.put(('nudge', float(d.get('dn', 0)), float(d.get('de', 0)),
                        float(d.get('dd', 0))))
    elif a == 'servo':
        pwm = int(d['pwm'])
        if not 800 <= pwm <= 2500:
            return jsonify({'ok': False, 'err': 'pwm out of range'}), 400
        if _bench_blocked():
            return jsonify({'ok': False, 'err': _bench_blocked() + ' - servo control refused'}), 409
        drone.cmds.put(('servo', pwm))
    elif a == 'servo_off':
        if _bench_blocked():
            return jsonify({'ok': False, 'err': _bench_blocked()}), 409
        drone.cmds.put(('servo_off',))
    elif a == 'get_params':
        if _bench_blocked():
            return jsonify({'ok': False, 'err': _bench_blocked() + ' - parameter reads wait'}), 409
        drone.cmds.put(('get_params', [str(x) for x in d.get('names', [])]))
    elif a == 'set_params':
        pv = d.get('params') or {}
        if not isinstance(pv, dict) or not pv:
            return jsonify({'ok': False, 'err': 'params must be a non-empty object'}), 400
        if _bench_blocked():
            return jsonify({'ok': False, 'err': _bench_blocked()}), 409
        drone.cmds.put(('set_params', {str(k): float(v) for k, v in pv.items()}))
    elif a in ('reboot', 'banner', 'accelcal_start', 'levelcal', 'magcal_start',
               'magcal_cancel', 'magcal_accept', 'magcal_yaw'):
        if a != 'banner' and _bench_blocked():
            return jsonify({'ok': False, 'err': _bench_blocked()}), 409
        drone.cmds.put((a,))
    elif a == 'accelcal_pos':
        pos = int(d.get('pos', 0))
        if not 1 <= pos <= 6:
            return jsonify({'ok': False, 'err': 'pos 1-6'}), 400
        drone.cmds.put(('accelcal_pos', pos))
    elif a == 'motor_test':
        motor = str(d.get('motor', '')).upper()
        if motor not in ('A', 'B', 'C', 'D', 'ALL'):
            return jsonify({'ok': False, 'err': 'motor must be A, B, C, D or ALL'}), 400
        ack = d.get('ack')
        cap = {'PROPS OFF': 20.0, 'PROPS ON LOW': 1.5}.get(ack)
        if cap is None:
            return jsonify({'ok': False, 'err': 'safety acknowledgement missing'}), 403
        pct, secs = float(d.get('pct', 5)), float(d.get('secs', 2))
        if not (0.1 <= pct <= cap) or not (0.5 <= secs <= 4):
            return jsonify({'ok': False, 'err': 'throttle max %g%% in this mode, 0.5-4 s' % cap}), 400
        # a motor test ARMS the FC for its duration: the next motor may be requested meanwhile
        if drone.state['armed'] and time.time() > drone.motor_test_until:
            return jsonify({'ok': False, 'err': 'aircraft is armed'}), 409
        drone.cmds.put(('motor_test', motor, pct, secs))
    elif a in ('hold', 'rtl', 'calgyro', 'lock'):
        drone.cmds.put((a,))
    elif a == 'arm_drop':
        DROP_ARM['until'] = time.time() + 8.0
    elif a == 'drop':
        # a lone POST can't fire the release: it must follow arm_drop within
        # 8 s AND carry the confirm word (CSRF/misclick protection)
        if time.time() > DROP_ARM['until'] or d.get('confirm') != 'DROP':
            return jsonify({'ok': False, 'err': 'drop not armed/confirmed'}), 403
        DROP_ARM['until'] = 0.0
        drone.cmds.put(('drop',))
    else:
        return jsonify({'ok': False, 'err': 'unknown action'}), 400
    return jsonify({'ok': True})


BRIDGE_STATUS = os.path.join(ROOT, 'console', 'bridge_status.json')
_bridge_cache = {'mtime': 0, 'data': None}


def bridge_status():
    """What fc/bt_bridge.py last wrote about itself (None when it never ran)."""
    try:
        mt = os.path.getmtime(BRIDGE_STATUS)
        if mt != _bridge_cache['mtime']:
            with open(BRIDGE_STATUS, encoding='utf-8') as f:
                _bridge_cache['data'] = json.load(f)
            _bridge_cache['mtime'] = mt
        return _bridge_cache['data']
    except Exception:
        return None


@app.route('/api/state')
def api_state():
    """One-shot snapshot of the drone state (for pages that poll instead of streaming)."""
    snap = dict(drone.state)
    snap['bridge'] = bridge_status()
    return Response(json.dumps(snap), mimetype='application/json')


@app.route('/api/ble_scan')
def api_ble_scan():
    """Six-second Bluetooth LE scan so a first-time user can pick the drone by name."""
    try:
        import asyncio
        from bleak import BleakScanner

        async def scan():
            found = await BleakScanner.discover(timeout=8.0, return_adv=True)
            out = []
            for addr, (dev, adv) in found.items():
                drone = ('000000ff-0000-1000-8000-00805f9b34fb' in (adv.service_uuids or [])
                         and 28717 in (adv.manufacturer_data or {}))      # SpeedyBee module fingerprint
                out.append({'name': (dev.name or adv.local_name or ''), 'address': addr, 'rssi': adv.rssi,
                            'drone': drone})
            return out
        devs = asyncio.run(scan())
    except Exception as e:
        return jsonify({'ok': False, 'err': 'Bluetooth scan failed (%s). Is Bluetooth switched on in Windows '
                        'settings? Does this laptop have Bluetooth at all?' % str(e)[:100]})
    devs.sort(key=lambda d: (not d['drone'], -(d['rssi'] if d['rssi'] is not None else -999), d['name']))
    return jsonify({'ok': True, 'devices': devs})


@app.route('/events')
def events():
    def gen():
        while True:
            yield f'data: {json.dumps(drone.state)}\n\n'
            time.sleep(0.5)
    return Response(gen(), mimetype='text/event-stream')


def main():
    global drone
    ap = argparse.ArgumentParser()
    ap.add_argument('--link', default=None,
                    help='serial port or tcp/udp url. Default: --sim uses SITL tcp; '
                         'otherwise auto-detect the FC on USB (vid 0x1209)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--sim', action='store_true', help='boot SITL over the estate')
    ap.add_argument('--speedup', type=int, default=1)
    ap.add_argument('--port', type=int, default=8008)
    args = ap.parse_args()

    resolver = None
    if args.link is None:
        if args.sim:
            args.link = 'tcp:127.0.0.1:5760'
        else:
            resolver = resolve_link          # bluetooth / usb / radio from the settings, live

    if args.sim:
        if not os.path.exists(SITL_EXE):
            ap.error(f'SITL binary not found: {SITL_EXE}')
        os.makedirs(SIM_DIR, exist_ok=True)
        pad = ESTATE.get('launch_pad') or SETTINGS.get('site')
        if not pad:
            ap.error('no site yet: set it in Operations (MENU, Set site location), then start --sim again')
        subprocess.Popen(
            [SITL_EXE, '--model', 'quad', '-w',
             '--home', f"{pad['lat']},{pad['lon']},15,90",
             '--defaults', SITL_DEFAULTS,
             '--speedup', str(args.speedup), '-I0'],
            cwd=SIM_DIR, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3)

    drone = DroneLink(args.link, args.baud, sim=args.sim, resolver=resolver)
    drone.start()
    print(f'Fire Console: http://127.0.0.1:{args.port}   (link: {args.link or SETTINGS.get("link")})')
    app.run(host='127.0.0.1', port=args.port, threaded=True)


if __name__ == '__main__':
    main()
