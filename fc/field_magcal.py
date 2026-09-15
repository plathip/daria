"""Fixed-yaw Large Vehicle MagCal — Step 1. NO rotating or tumbling.

The quad sits STILL on the ground, props off, battery in, nose pointing a
known direction (default: true north). The calibration is computed from the
known heading plus the GPS position (world magnetic model). Needs a 3D GPS
fix first; this script waits for one.

Stay away from cars, rebar and steel structures. A phone compass is
accurate enough to find true north for this method.

Usage:
    python fc/field_magcal.py               # nose pointing true NORTH (yaw 0)
    python fc/field_magcal.py --yaw 90      # nose pointing EAST instead
    python fc/field_magcal.py --no-confirm  # skip the Enter prompt (scripted use)
"""
import argparse
import time

from pymavlink import mavutil
from serial.tools import list_ports

MAV_CMD_FIXED_MAG_CAL_YAW = getattr(mavutil.mavlink, 'MAV_CMD_FIXED_MAG_CAL_YAW', 42006)
RUN_PREARM_CHECKS = getattr(mavutil.mavlink, 'MAV_CMD_RUN_PREARM_CHECKS', 401)


def find_fc():
    for p in list_ports.comports():
        if p.vid == 0x1209:          # ArduPilot USB vendor id
            return p.device
    raise SystemExit('No ArduPilot board found on USB. Is it plugged in and powered?')


def from_vehicle(m, msg):
    return (msg is not None and msg.get_srcSystem() == m.target_system
            and msg.get_srcComponent() == 1)


def read_param(m, name, timeout=3.0):
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
            return msg.param_value
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaw', type=float, default=0.0,
                    help='true heading of the NOSE in degrees (0=N, 90=E, 180=S, 270=W)')
    ap.add_argument('--no-confirm', action='store_true')
    args = ap.parse_args()

    port = find_fc()
    print('connecting on %s ...' % port)
    m = mavutil.mavlink_connection(port, baud=115200, source_system=255)
    if m.wait_heartbeat(timeout=10) is None:
        raise SystemExit('No heartbeat in 10 s. Cold power-cycle the board and retry.')
    print('connected: system %d' % m.target_system)

    print('waiting for 3D GPS fix ...')
    last = 0.0
    while True:
        msg = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if not from_vehicle(m, msg):
            continue
        if time.time() - last > 2:
            last = time.time()
            print('  fix_type=%d  sats=%d' % (msg.fix_type, msg.satellites_visible))
        if msg.fix_type >= 3:
            print('3D fix, %d satellites.' % msg.satellites_visible)
            break

    print('\nAircraft still, LEVEL, nose pointing %.0f deg true (0 = north).' % args.yaw)
    print('Away from cars / rebar / steel. Props off.')
    if not args.no_confirm:
        input('Press Enter when it is in position ... ')

    m.mav.command_long_send(
        m.target_system, m.target_component,
        MAV_CMD_FIXED_MAG_CAL_YAW, 0,
        args.yaw,      # known yaw, degrees
        0,             # compass mask: 0 = all compasses
        0, 0,          # lat, lon: 0 = use current GPS position
        0, 0, 0)

    ok = None
    deadline = time.time() + 10
    while time.time() < deadline:
        msg = m.recv_match(blocking=True, timeout=1)
        if not from_vehicle(m, msg):
            continue
        t = msg.get_type()
        if t == 'STATUSTEXT':
            print('  [FC] %s' % msg.text)
        elif t == 'COMMAND_ACK' and msg.command == MAV_CMD_FIXED_MAG_CAL_YAW:
            ok = (msg.result == 0)
            print('  ack result=%d (%s)' % (msg.result, 'ACCEPTED' if ok else 'REJECTED'))
            break

    if ok is None:
        raise SystemExit('No acknowledgement — try again.')
    if not ok:
        raise SystemExit('Calibration REJECTED. Usual causes: no position estimate yet '
                         '(wait longer after the fix), or compass unhealthy. Fix and rerun.')

    print('\nnew compass offsets:')
    for axis in 'XYZ':
        v = read_param(m, 'COMPASS_OFS_%s' % axis)
        flag = ''
        if v is not None and abs(v) > 600:
            flag = '   <-- LARGE, move further from metal and redo'
        print('  COMPASS_OFS_%s = %s%s' % (axis, '%.1f' % v if v is not None else '?', flag))

    print('\nre-running prearm checks ...')
    m.mav.command_long_send(m.target_system, m.target_component,
                            RUN_PREARM_CHECKS, 0, 0, 0, 0, 0, 0, 0, 0)
    quiet = True
    deadline = time.time() + 6
    while time.time() < deadline:
        msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if from_vehicle(m, msg):
            print('  [FC] %s' % msg.text)
            if 'PreArm' in msg.text:
                quiet = False
    print('\nDone. %s' % ('No PreArm complaints — compass accepted.' if quiet
                          else 'PreArm complaints above — resolve before flying.'))
    print('Verify in the air: Stabilize hop first, then AltHold, then Loiter.')
    print('If Loiter toilet-bowls, land and redo this further from metal.')


if __name__ == '__main__':
    main()
