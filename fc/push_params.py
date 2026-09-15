"""Upload the base parameter file to the FC with per-parameter verification,
then reboot and print a bench health report.

Usage: python push_params.py [paramfile]
"""
import os
import sys
import time

from pymavlink import mavutil
from serial.tools import list_ports

PARAM_FILE = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'speedybee-f405v5-base.param')


def find_port():
    for p in list_ports.comports():
        if p.vid == 0x1209 and p.pid == 0x5741:
            return p.device
    raise SystemExit('ArduPilot USB port not found')


def connect():
    port = find_port()
    m = mavutil.mavlink_connection(port, baud=115200, source_system=255)
    hb = None
    for _ in range(8):
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        hb = m.wait_heartbeat(timeout=4)
        if hb is not None:
            break
    if hb is None:
        raise SystemExit('no heartbeat')
    return m


def read_params(path):
    out = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            name, val = line.split(',', 1)
            out.append((name.strip(), float(val.split('#')[0].strip())))   # inline '# comments' allowed
    return out


def set_verify(m, name, value, tries=4):
    for _ in range(tries):
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), value,
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if msg and msg.param_id == name:
                if abs(msg.param_value - value) < 1e-4 or \
                        (value != 0 and abs(msg.param_value / value - 1) < 1e-4):
                    return True
                break
    return False


def main():
    m = connect()
    print(f'Connected. sysid={m.target_system}')
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SEND_BANNER, 0,
                            0, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < 4:
        msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if msg:
            print('  [FC]', msg.text)

    params = read_params(PARAM_FILE)
    print(f'\nUploading {len(params)} parameters...')
    failed = []
    for name, value in params:
        ok = set_verify(m, name, value)
        print(f'  {"OK " if ok else "FAIL"} {name} = {value:g}')
        if not ok:
            failed.append(name)

    if failed:
        print(f'\nWARNING: {len(failed)} params failed: {failed}')
    else:
        print('\nAll parameters verified. Rebooting FC...')
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                            0, 1, 0, 0, 0, 0, 0, 0)
    m.close()
    time.sleep(8)

    m = connect()
    print('\nReconnected after reboot. Watching health for 25 s...')
    seen = set()
    t0 = time.time()
    while time.time() - t0 < 25:
        msg = m.recv_match(blocking=True, timeout=2)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'STATUSTEXT' and msg.text not in seen:
            seen.add(msg.text)
            print('  [FC]', msg.text)
        elif t == 'GPS_RAW_INT' and 'gps' not in seen:
            seen.add('gps')
            print(f'  GPS message flowing (fix_type={msg.fix_type}, sats={msg.satellites_visible})')
        elif t == 'RAW_IMU' and 'mag' not in seen:
            seen.add('mag')
            print(f'  Mag raw: x={msg.xmag} y={msg.ymag} z={msg.zmag} '
                  f'{"(COMPASS DETECTED)" if any((msg.xmag, msg.ymag, msg.zmag)) else "(no compass data!)"}')
        elif t == 'RC_CHANNELS' and 'rc' not in seen and msg.chancount > 0:
            seen.add('rc')
            print(f'  RC input detected: {msg.chancount} channels, ch1={msg.chan1_raw}')
    print('\nDone.')


if __name__ == '__main__':
    main()
