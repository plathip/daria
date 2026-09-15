"""Prearm + vitals check — Step 3 helper. Read-only apart from asking the FC
to re-run its own prearm checks (command 401, does not arm anything).

One-shot mode prints: firmware banner, mode, arm state, battery voltage,
GPS fix/sats/hdop, RC status, then forces a prearm re-check and reports any
"PreArm:" complaints.

Watch mode streams mode / battery / GPS / RC and every STATUSTEXT until
Ctrl+C — use it for the radio-off throttle-failsafe test: switch the radio
off and you should see the FC react within a couple of seconds.

Usage:
    python fc/prearm_check.py           # one-shot
    python fc/prearm_check.py --watch   # stream until Ctrl+C
"""
import argparse
import sys
import time

from pymavlink import mavutil
from serial.tools import list_ports

RUN_PREARM_CHECKS = getattr(mavutil.mavlink, 'MAV_CMD_RUN_PREARM_CHECKS', 401)


def find_fc():
    for p in list_ports.comports():
        if p.vid == 0x1209:          # ArduPilot USB vendor id
            return p.device
    raise SystemExit('No ArduPilot board found on USB. Is it plugged in and powered?')


def from_vehicle(m, msg):
    return (msg is not None and msg.get_srcSystem() == m.target_system
            and msg.get_srcComponent() == 1)


def gather(m, seconds=3.0):
    state = {}
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = m.recv_match(blocking=True, timeout=1)
        if not from_vehicle(m, msg):
            continue
        t = msg.get_type()
        if t == 'HEARTBEAT':
            state['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            state['mode'] = mavutil.mode_string_v10(msg)
        elif t == 'SYS_STATUS':
            state['volt'] = msg.voltage_battery / 1000.0
            state['curr'] = msg.current_battery / 100.0 if msg.current_battery >= 0 else None
        elif t == 'GPS_RAW_INT':
            state['fix'] = msg.fix_type
            state['sats'] = msg.satellites_visible
            state['hdop'] = msg.eph / 100.0 if msg.eph != 65535 else None
        elif t == 'RC_CHANNELS':
            state['rc'] = msg.chancount
        elif t == 'STATUSTEXT':
            state.setdefault('texts', []).append(msg.text)
    return state


def show(state):
    fixnames = {0: 'none', 1: 'none', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'RTK-f', 6: 'RTK'}
    print('  mode: %s   armed: %s' % (state.get('mode', '?'), state.get('armed', '?')))
    v = state.get('volt')
    note = ''
    if v is not None:
        if v < 1.0:
            note = '  (USB only — no battery)'
        elif not 19.8 <= v <= 25.5:
            note = '  <-- NOT SANE for 6S (expect ~21-25.2 V)'
    print('  battery: %s V%s' % ('%.2f' % v if v is not None else '?', note))
    print('  gps: fix=%s  sats=%s  hdop=%s'
          % (fixnames.get(state.get('fix'), state.get('fix', '?')),
             state.get('sats', '?'), state.get('hdop', '?')))
    rc = state.get('rc')
    print('  rc: %s' % ('%d channels' % rc if rc else
                        'NO RC INPUT (radio off, or ELRS in WiFi mode — '
                        'radio ON first, then power the aircraft)'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--watch', action='store_true')
    args = ap.parse_args()

    port = find_fc()
    print('connecting on %s ...' % port)
    m = mavutil.mavlink_connection(port, baud=115200, source_system=255)
    if m.wait_heartbeat(timeout=10) is None:
        raise SystemExit('No heartbeat in 10 s. Cold power-cycle the board and retry.')
    print('connected: system %d\n' % m.target_system)

    # make sure the streams we read actually flow (not saved; reverts on reboot)
    for msg_id in (1, 24, 65):        # SYS_STATUS, GPS_RAW_INT, RC_CHANNELS
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, 500000, 0, 0, 0, 0, 0)

    if args.watch:
        print('watching — Ctrl+C to stop\n')
        while True:
            state = gather(m, 2.0)
            for t in state.get('texts', []):
                print('  [FC] %s' % t)
            v = state.get('volt')
            print('mode=%-10s armed=%-5s batt=%-6s rc=%-3s gps fix=%s sats=%s'
                  % (state.get('mode', '?'), state.get('armed', '?'),
                     ('%.2fV' % v) if v is not None else '?',
                     state.get('rc', '?'), state.get('fix', '?'), state.get('sats', '?')))

    state = gather(m, 3.0)
    show(state)
    for t in state.get('texts', []):
        print('  [FC] %s' % t)

    print('\nforcing prearm re-check (cmd 401) ...')
    m.mav.command_long_send(m.target_system, m.target_component,
                            RUN_PREARM_CHECKS, 0, 0, 0, 0, 0, 0, 0, 0)
    complaints = []
    deadline = time.time() + 6
    while time.time() < deadline:
        msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if from_vehicle(m, msg):
            print('  [FC] %s' % msg.text)
            if 'PreArm' in msg.text:
                complaints.append(msg.text)
    if complaints:
        print('\n%d PreArm complaint(s) — resolve before flying.' % len(complaints))
        sys.exit(1)
    print('\nNo PreArm complaints reported.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\ninterrupted')
