"""Dump ALL parameters from the FC to a timestamped .param file.

Run this BEFORE changing anything in the field — it is the restore point.
Output: fc/backups/params-YYYYMMDD-HHMMSS.param  (Mission Planner format,
one "NAME,VALUE" per line, loadable by MP or by fc/param_tool.py).

Read-only. Safe any time the board is powered.

Usage:
    python fc/param_snapshot.py
"""
import os
import sys
import time

from pymavlink import mavutil
from serial.tools import list_ports


def find_fc():
    for p in list_ports.comports():
        if p.vid == 0x1209:          # ArduPilot USB vendor id
            return p.device
    raise SystemExit('No ArduPilot board found on USB. Is it plugged in and powered?')


def fmt(v):
    return str(int(v)) if float(v) == int(v) else ('%.10g' % v)


def main():
    port = find_fc()
    print('connecting on %s ...' % port)
    m = mavutil.mavlink_connection(port, baud=115200, source_system=255)
    if m.wait_heartbeat(timeout=10) is None:
        raise SystemExit('No heartbeat in 10 s. Cold power-cycle the board and retry.')
    print('connected: system %d' % m.target_system)

    m.mav.param_request_list_send(m.target_system, m.target_component)
    params = {}          # index -> (name, value)
    count = None
    last_rx = time.time()
    while True:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if msg is None:
            if count is not None and len(params) >= count:
                break
            if time.time() - last_rx > 10:
                break                       # stalled; fill gaps below
            continue
        if msg.get_srcSystem() != m.target_system or msg.get_srcComponent() != 1:
            continue
        last_rx = time.time()
        count = msg.param_count
        name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
        params[msg.param_index] = (name.rstrip('\x00'), msg.param_value)
        if len(params) % 200 == 0:
            print('  %d / %s ...' % (len(params), count))
        if count is not None and len(params) >= count:
            break

    # fetch any indices the bulk transfer dropped
    if count:
        for attempt in range(3):
            missing = [i for i in range(count) if i not in params]
            if not missing:
                break
            print('re-requesting %d missed parameters (pass %d)...' % (len(missing), attempt + 1))
            for i in missing:
                m.mav.param_request_read_send(m.target_system, m.target_component, b'', i)
                msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
                if msg and msg.get_srcSystem() == m.target_system and msg.get_srcComponent() == 1:
                    name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
                    params[msg.param_index] = (name.rstrip('\x00'), msg.param_value)

    if not params:
        raise SystemExit('No parameters received.')

    got = len(params)
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'backups')
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, time.strftime('params-%Y%m%d-%H%M%S.param'))
    with open(path, 'w') as f:
        f.write('# full parameter snapshot, %s, %d/%s params\n'
                % (time.strftime('%Y-%m-%d %H:%M:%S'), got, count))
        for name, value in sorted(params.values()):
            f.write('%s,%s\n' % (name, fmt(value)))

    print('\nsaved %d/%s parameters -> %s' % (got, count, path))
    if count and got < count:
        print('WARNING: %d parameters still missing after retries — '
              'run it again before trusting this as a restore point.' % (count - got))
        sys.exit(1)


if __name__ == '__main__':
    main()
