"""Record RC channel min/max/current for the radio range calibration.
Writes rc_record.json continuously; delete-safe, exits after 4 minutes."""
import json
import os
import time

from pymavlink import mavutil
from serial.tools import list_ports

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rc_record.json')

port = next(p.device for p in list_ports.comports() if p.vid == 0x1209)
m = mavutil.mavlink_connection(port, baud=115200, source_system=255)
for _ in range(8):
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    if m.wait_heartbeat(timeout=4):
        break
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        65, 100000, 0, 0, 0, 0, 0)

stats = {i: {'min': 99999, 'max': 0, 'now': 0} for i in range(1, 13)}
t0 = time.time()
last_dump = 0.0
while time.time() - t0 < 240:
    msg = m.recv_match(type='RC_CHANNELS', blocking=True, timeout=2)
    if msg is None or msg.chancount == 0:
        continue
    for i in range(1, 13):
        v = getattr(msg, f'chan{i}_raw')
        if v and v != 65535:
            s = stats[i]
            s['min'] = min(s['min'], v)
            s['max'] = max(s['max'], v)
            s['now'] = v
    if time.time() - last_dump > 1.5:
        with open(OUT, 'w') as f:
            json.dump({'elapsed': round(time.time() - t0), 'ch': stats}, f)
        last_dump = time.time()
m.close()
with open(OUT, 'w') as f:
    json.dump({'elapsed': 'finished', 'ch': stats}, f)
print('recorder finished')
