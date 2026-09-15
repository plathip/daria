"""Get / set / search FC parameters over USB.

Usage:
    python fc/param_tool.py get MOT_THST_HOVER [MORE_NAMES...]
    python fc/param_tool.py set BATT_LOW_VOLT 21.0
    python fc/param_tool.py find RTL          # substring search, case-insensitive
    python fc/param_tool.py find rally

ArduCopter 4.7 renamed several parameters (RTL_ALT -> RTL_ALT_M,
LAND_SPEED -> LAND_SPD_MS, ARMING_CHECK -> ARMING_SKIPCHK with INVERTED
meaning, WPNAV_* speeds gone). If `get`/`set` says a name is unknown,
use `find` to search for the new name — do not guess.

`set` writes ONE parameter and reads it back to verify. Take a snapshot
first (python fc/param_snapshot.py) if you are changing anything risky.
"""
import sys
import time

from pymavlink import mavutil
from serial.tools import list_ports


def find_fc():
    for p in list_ports.comports():
        if p.vid == 0x1209:          # ArduPilot USB vendor id
            return p.device
    raise SystemExit('No ArduPilot board found on USB. Is it plugged in and powered?')


def connect():
    port = find_fc()
    m = mavutil.mavlink_connection(port, baud=115200, source_system=255)
    if m.wait_heartbeat(timeout=10) is None:
        raise SystemExit('No heartbeat in 10 s. Cold power-cycle the board and retry.')
    return m


def from_vehicle(m, msg):
    return (msg is not None and msg.get_srcSystem() == m.target_system
            and msg.get_srcComponent() == 1)


def read_param(m, name, timeout=3.0):
    """Returns (value, mav_type) or (None, None) if the FC does not know it."""
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if not from_vehicle(m, msg):
            continue
        got = (msg.param_id if isinstance(msg.param_id, str)
               else msg.param_id.decode()).rstrip('\x00')
        if got == name:
            return msg.param_value, msg.param_type
    return None, None


def fetch_all(m):
    m.mav.param_request_list_send(m.target_system, m.target_component)
    params, count, last_rx = {}, None, time.time()
    while True:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if msg is None:
            if (count is not None and len(params) >= count) or time.time() - last_rx > 10:
                break
            continue
        if not from_vehicle(m, msg):
            continue
        last_rx = time.time()
        count = msg.param_count
        name = (msg.param_id if isinstance(msg.param_id, str)
                else msg.param_id.decode()).rstrip('\x00')
        params[name] = msg.param_value
        if count is not None and len(params) >= count:
            break
    return params


def fmt(v):
    return str(int(v)) if float(v) == int(v) else ('%.10g' % v)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    cmd = sys.argv[1].lower()
    m = connect()

    if cmd == 'get':
        ok = True
        for name in sys.argv[2:]:
            name = name.upper()
            value, _ = read_param(m, name)
            if value is None:
                print('%s: NOT FOUND (renamed in 4.7? try: python fc/param_tool.py find %s)'
                      % (name, name.split('_')[0]))
                ok = False
            else:
                print('%s = %s' % (name, fmt(value)))
        sys.exit(0 if ok else 1)

    elif cmd == 'set':
        if len(sys.argv) != 4:
            raise SystemExit('usage: param_tool.py set NAME VALUE')
        name, want = sys.argv[2].upper(), float(sys.argv[3])
        old, ptype = read_param(m, name)
        if old is None:
            raise SystemExit('%s: NOT FOUND — use `find` to locate the 4.7 name, do not guess.'
                             % name)
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), want, ptype)
        time.sleep(0.3)
        new, _ = read_param(m, name)
        if new is None or abs(new - want) > max(1e-6, abs(want) * 1e-6):
            raise SystemExit('%s: write FAILED (still %s). Read-only while armed? '
                             'Value out of range?' % (name, fmt(new) if new is not None else '?'))
        print('%s: %s -> %s  (verified)' % (name, fmt(old), fmt(new)))

    elif cmd == 'find':
        needle = sys.argv[2].upper()
        print('fetching full parameter list ...')
        params = fetch_all(m)
        hits = sorted(n for n in params if needle in n)
        if not hits:
            print('no parameter name contains "%s" (%d params searched)' % (needle, len(params)))
            sys.exit(1)
        for n in hits:
            print('%s = %s' % (n, fmt(params[n])))

    else:
        raise SystemExit('unknown command %r — use get / set / find' % cmd)


if __name__ == '__main__':
    main()
