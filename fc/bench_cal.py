"""Bench step: RC snapshot + simple accel cal + level trim + prearm re-check.
Quad must be LEVEL and MOTIONLESS. USB power only.
"""
import time

from pymavlink import mavutil
from serial.tools import list_ports


def connect():
    port = next(p.device for p in list_ports.comports()
                if p.vid == 0x1209 and p.pid == 0x5741)
    m = mavutil.mavlink_connection(port, baud=115200, source_system=255)
    for _ in range(8):
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if m.wait_heartbeat(timeout=4):
            return m
    raise SystemExit('no heartbeat')


def watch(m, seconds, label):
    t0 = time.time()
    seen = set()
    while time.time() - t0 < seconds:
        msg = m.recv_match(type=['STATUSTEXT', 'COMMAND_ACK'], blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == 'STATUSTEXT':
            if msg.text not in seen:
                seen.add(msg.text)
                print(f'  [{label}] {msg.text}')
        else:
            print(f'  [{label}] ACK cmd={msg.command} result={msg.result}')


m = connect()
print('Connected.')

# --- RC snapshot ---
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        65, 200000, 0, 0, 0, 0, 0)
rc = None
t0 = time.time()
while time.time() - t0 < 6:
    msg = m.recv_match(type='RC_CHANNELS', blocking=True, timeout=2)
    if msg and msg.chancount > 0:
        rc = msg
        break
if rc:
    print(f'RC LINK OK: {rc.chancount} channels')
    for i in range(1, 13):
        print(f'  ch{i}: {getattr(rc, f"chan{i}_raw")}')
else:
    print('RC LINK: no channels seen (is the receiver bound/powered?)')

# --- simple accel calibration (vehicle level + still) ---
print('\nSimple accel calibration...')
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                        0, 0, 0, 0, 4, 0, 0)
watch(m, 15, 'accel')

# --- board level trim ---
print('Level trim...')
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                        0, 0, 0, 0, 2, 0, 0)
watch(m, 10, 'level')

# --- re-run prearm checks ---
print('Prearm re-check...')
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, 0,
                        0, 0, 0, 0, 0, 0, 0)
watch(m, 8, 'prearm')
m.close()
print('\nDone.')
