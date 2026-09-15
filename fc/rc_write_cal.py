"""Write RC calibration (from recorded ranges) + switch mapping, then re-check prearm."""
import time

from pymavlink import mavutil
from serial.tools import list_ports


def connect():
    port = next(p.device for p in list_ports.comports() if p.vid == 0x1209)
    m = mavutil.mavlink_connection(port, baud=115200, source_system=255)
    for _ in range(8):
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if m.wait_heartbeat(timeout=4):
            return m
    raise SystemExit('no heartbeat')


def setp(m, name, val):
    for _ in range(4):
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), float(val),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        while r is not None and r.param_id != name:
            r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if r is not None and abs(r.param_value - val) < 0.5:
            print(f'  OK {name} = {val}')
            return
    print(f'  FAIL {name}')


m = connect()
print('Connected. Writing RC calibration...')

# sticks: measured ELRS ranges from the recording session
cal = {1: (988, 2010, 1499), 2: (988, 2011, 1502),
       3: (988, 2011, 1500), 4: (988, 2011, 1500)}
for ch, (lo, hi, trim) in cal.items():
    setp(m, f'RC{ch}_MIN', lo)
    setp(m, f'RC{ch}_MAX', hi)
    setp(m, f'RC{ch}_TRIM', trim)
# switch channels: nominal ELRS range
for ch in (5, 6, 7, 8, 9, 10, 11, 12):
    setp(m, f'RC{ch}_MIN', 988)
    setp(m, f'RC{ch}_MAX', 2012)
    setp(m, f'RC{ch}_TRIM', 1500)

print('Switch mapping...')
setp(m, 'FLTMODE3', 2)      # mode-switch middle lands AltHold in either slot
setp(m, 'RC9_OPTION', 0)    # old AltHold/PosHold switch: unassigned for now
setp(m, 'RC11_OPTION', 4)   # old GPS Rescue switch: RTL (same finger as before)

print('Prearm re-check...')
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, 0,
                        0, 0, 0, 0, 0, 0, 0)
t0 = time.time()
seen = set()
while time.time() - t0 < 8:
    msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
    if msg and msg.text not in seen:
        seen.add(msg.text)
        print(f'  [FC] {msg.text}')
m.close()
print('Done.')
